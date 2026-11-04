"""The AerialVLA pilot on the Project AirSim + ArduPilot rail
(demo/pas_ardupilot/perception_node.py --pilot vla) and the rail's VLA
cameras, checked without the simulator, ArduPilot, the GPU or the model.

Run either way:
    pytest tests/test_pas_vla.py -v
    python tests/test_pas_vla.py

WHAT IS BEING CLAIMED (to-do VLA-03 and VLA-07, wave V2b, 2026-10-23)

1. The rail's robot config carries AerialVLA's training cameras, VLAFront
   and VLADown, exactly as the CityLife VLA configs do (256x256, 90 deg,
   pitch 0 / -90, every 0.1 s, never streamed), beside the detector's own
   FrontCamera, which is unchanged; the camera subscribes both into the
   VLA frame store and refuses a robot config without them.
2. `--pilot` defaults to the detector pilot (follow_vlm's servo law); the
   options a VLA run cannot fly honestly with are refused before anything
   starts: no server configuration (attach with --vla-server, or start one
   naming --vla-precision, --vla-attn and --vla-merge / --no-vla-merge, with
   no default), a configuration given to an attached server, a GOTO without
   its goal, a goal or a VLA mission with the detector pilot, the ROS 2
   camera, object words the server would refuse. A server the node starts
   names its whole configuration.
3. A GOTO flown by a scripted protocol-v2 server (tests/vla_fake_server.py,
   loopback, an ephemeral port): three warm-up requests before the aircraft
   is armed, then the step tracker's action is the raw action, the Shield
   filters it and the escalation FSM steps on it exactly as on the detector
   pilot's path - only the Shield's output reaches the autopilot - and the
   model's LAND ends the mission through the node's own mission-end LAND
   (link.land), never a mode request from the pilot. The records are the
   city rail's: vla_steps.jsonl, rows naming who authored each tick,
   metrics.pilot kind vla with the server's reported digest, the manifest's
   vla_model_hash, and tools/todo.py's classifier reads the run as a VLA
   flight. The start gate (a detector-subject gate) is skipped and
   recorded, and no detector runs.
4. An FSM hold suspends the tracker: nothing it decided before the hold
   flies after it, and the next tick that flies resumes it.
5. The warm-up gate refuses a server that refuses before arming (nothing
   armed, nothing sent), judges the LAST of its three requests, waits out
   the client's backoff between them, and keeps the autopilot link read
   while an answer is awaited.
6. A dead server makes the tracker hover, the mission end after
   --dead-timeout-s and the aircraft land; a stand-in is a stub, never a
   VLA, and is refused unless --allow-stand-in.
7. The APPROACH's hint is the detector box's phrase (hint_source detector),
   and the box still gives the Shield its subject.
8. A Ctrl+C in the VLA mission lands, closes the VLA records and ends the
   Dashboard row "stopped"; a crash in the VLA's set-up after the take-off
   is landed by the net (land_on_the_way_out).
9. The HUD panel: the city rail's (vla_mission.hud_lines and annotate_vla,
   moved there from demo/vla_fly.py, not copied), fed per tick on this
   rail, and drawn on its frames where Pillow is installed (SKIP on CI).

10. The review of 4a0da6d (2026-10-23): a slow cold first warm-up answer
    gets the timeout recorded for it, each request waits for a ready
    client, and a request can carry its own answer timeout (P1-1, P2-8,
    P3-15); a refused warm-up is on record - metrics, manifest, captures,
    mission_end, a "failed" Dashboard row (P2-3); a GOTO is no unscored
    follow (P2-2); the VLA cameras render only for a VLA run, said so, and
    an attached node refuses a scene without them (P2-4); pipelined is
    refused unless asked for, and each capture's time and pose basis is
    counted (P2-5); the documented goal is clear (P2-6, lab-only map); the
    GPU's memory is recorded (P2-7); holds pause the mission's stop rules
    (P3-9), stale spans suspend the tracker and the FSM stepping on its zero
    is counted (P3-13), held ticks are not VLA-flown (P3-12); the start gate
    and detector are named as they ran (P3-10); an approach is scored on the
    subject's true range (P3-11); the panel keeps the rail's label (P3-14);
    the captures survive a crash (P3-15). Run against 4a0da6d, 17 of this
    file's 28 tests then failed and both new tests of test_vla_fly.py did.

Shown failing first: against 5b00e6b (the tree before this change) every
test below failed (0/16 of the first version of this file) - the parser
had no --pilot, the robot config no VLA camera, vla_mission no hud_lines or
run_warmup, perception_node no VLAPilot. The warm-up's backoff wait
(vla_mission.run_warmup) was found by test 5's second case: without it the
cold-start request's server error made the next two "not sent".
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "tests", ROOT / "demo", PAS, ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import autopilot as ap                                               # noqa: E402
import perception_node as pn                                         # noqa: E402
import rail                                                          # noqa: E402
import test_pas_ardupilot as T                                       # noqa: E402
import vla_fake_server as F                                          # noqa: E402
import vla_mission as VM                                             # noqa: E402
from guardrail.vla_link import (AUTHORS, PHRASES, Capture, Pose,     # noqa: E402
                                RemoteVLA)

SKIP = "SKIP"
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"
# A model identity that looks like the real server's: the dashboard's digest
# rule refuses the fake server's "0" * 64 (tests/test_vla_fly.py REAL_MODEL).
REAL_MODEL = {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
              "weights_sha256": hashlib.sha256(b"base shards + adapter").hexdigest(),
              "precision": "bf16", "attn": "sdpa", "merged": False,
              "agreement": {"measured": True, "probe_exact": 0.96}}
# FakeLink takes off at the CityLife spawn (35, -20), 8 m up, facing North.
START = (35.0, -20.0)
GOAL = (90.0, -20.0)          # 55 m due North: about 11 stop-and-go steps
ANCHOR = (35.6895, 139.6917, 40.0)


def _real(respond, model=REAL_MODEL):
    """`respond` with every answer naming `model` (the identity the client
    pinned from the hello)."""
    def r(req):
        out = respond(req)
        if isinstance(out, dict) and "model" in out:
            out = dict(out, model=dict(model))
        return out
    return r


class VLALink(T.PreArmLink):
    """PreArmLink (a pre-arm step, as PymavlinkLink and MavrosLink have) whose
    heading follows the yaw rate it is sent - the step tracker steers the
    heading - and which counts what the server had received when it armed."""

    def __init__(self, clock, anchor, on_arm=None, **kw):
        super().__init__(clock, anchor, **kw)
        self.on_arm = on_arm
        self.requests_at_arm = None
        self.armed_in_before_arm = None

    def bring_up(self, alt, before_arm=None):
        self.snap.armed = False
        self.pump()
        if before_arm is not None:
            self.armed_in_before_arm = self.snap.armed
            before_arm()
        self.requests_at_arm = None if self.on_arm is None else self.on_arm()
        self._mode("GUIDED")
        self.snap.armed = True
        self.pump()

    def pump(self):
        t = self.clock.t
        if self.t_last is not None and self.snap.mode == "GUIDED":
            self.yaw = ((self.yaw + self.v.yaw_rate * (t - self.t_last) + math.pi)
                        % (2 * math.pi) - math.pi)
        super().pump()


class FakeVLAFrames:
    """vla_mission.VLAFrames' interface on the stand-in link: a capture is the
    link's pose now and two JPEG stand-ins; `commit` keeps the sent ones.
    `interrupt_at`: the N-th capture raises KeyboardInterrupt (a Ctrl+C)."""

    def __init__(self, link, interrupt_at=None):
        self.link, self.interrupt_at = link, interrupt_at
        self.log: list = []
        self.n = 0

    def capture(self):
        self.n += 1
        if self.interrupt_at is not None and self.n == self.interrupt_at:
            raise KeyboardInterrupt
        st = self.link.state(self.link.anchor)
        return Capture(self.link.clock.t, Pose(st.x, st.y, st.up, math.radians(st.yaw_deg)),
                       JPG, JPG)

    def commit(self, capture, hint=None, seq=None, **extra):
        self.log.append({"seq": seq, "phrase": None if hint is None else hint.phrase,
                         "hint_source": None if hint is None else hint.source, **extra})

    def stats(self):
        return {"counts": {"committed": len(self.log), "captures": self.n}}


def _no_gpu_query(cmd, **kw):
    """The stand-in run's `run_cmd`: no nvidia-smi is asked (the GPU memory
    query records why)."""
    raise FileNotFoundError(f"{cmd[0]}: not run in tests")


def _vfly(respond, model=REAL_MODEL, *, mission="goto", goal=GOAL, policy=T.CAP_POLICY,
          shield="on", max_s="90", extra=(), link_cls=VLALink, grounder=None,
          frames_kw=None, recorder_cls=None, camera=None, live_dir=None, raises=None,
          client_kw=None, link_kw=None, run_cmd=None):
    """One VLA flight of perception_node.run on the stand-ins, against a
    scripted protocol-v2 server on an ephemeral port, attached with
    --vla-server. The client is injected on the test's clock, and the
    test's sleep waits for an outstanding answer before it advances that
    clock, so a fake-clock flight sees every answer within one tick.
    `raises`: the exception type run() must raise (returned under .exc)."""
    out = T._tmp()
    pol = out / "policy.yaml"
    pol.write_text(policy, encoding="utf-8")
    srv = F.FakeVLAServer(respond, model=model)
    srv.start()
    exc = None
    try:
        argv = ["--out-root", str(out), "--tag", "run", "--policy", str(pol),
                "--max-s", max_s, "--hfov", "90", "--citymap", str(out / "no_map.npz"),
                "--network", "nat", "--windows-ip", T.WIN, "--wsl-ip", T.WSL,
                "--shield", shield, "--dev-yaml", "--start-when-seen", "0",
                "--pilot", "vla", "--mission", mission,
                "--vla-server", f"127.0.0.1:{srv.port}"]
        if mission == "goto":
            argv += ["--goal", f"{goal[0]},{goal[1]}"]
        args = pn.build_parser().parse_args(argv + list(extra))
        clk = T.Clock()
        anchor = ap.FrameAnchor(*ANCHOR, "scene.home-geo-point")
        link = link_cls(clk, anchor, on_arm=lambda: len(srv.requests), **(link_kw or {}))
        client = RemoteVLA("127.0.0.1", srv.port, **{
            "response_timeout_s": 2.0, "reconnect_backoff_s": 0.5,
            "strict_identity": VM.identity_is_vla(model)[0], "clock": clk,
            **(client_kw or {})})

        def sleep(dt):
            client.wait_idle(3.0)
            clk.sleep(dt)
        frames = FakeVLAFrames(link, **(frames_kw or {}))
        saved = os.environ.get("VLAGUARD_LIVE_DIR")
        if live_dir is not None:
            os.environ["VLAGUARD_LIVE_DIR"] = str(live_dir)
        rc = None
        try:
            rc = pn.run(args, link=link, camera=camera or T.FakeCamera(link),
                        grounder=grounder(clk) if callable(grounder) else grounder,
                        obs=T.FakeObs(), clock=clk, sleep=sleep, vla_client=client,
                        vla_frames=frames, recorder_cls=recorder_cls,
                        live_status=live_dir is not None,
                        run_cmd=run_cmd or _no_gpu_query)
        except BaseException as e:                                   # noqa: BLE001
            if raises is None or not isinstance(e, raises):
                raise
            exc = e
        finally:
            if live_dir is not None:
                if saved is None:
                    os.environ.pop("VLAGUARD_LIVE_DIR", None)
                else:
                    os.environ["VLAGUARD_LIVE_DIR"] = saved
        if raises is not None and exc is None:
            raise AssertionError(f"run() did not raise {raises.__name__}")
    finally:
        srv.stop()
    return SimpleNamespace(rc=rc, run=out / "run", link=link, srv=srv, frames=frames,
                           client=client, exc=exc, out=out)


def _rows(run):
    return [json.loads(l) for l in (run / "flight_log.jsonl").read_text().splitlines()]


def _json(run, name):
    return json.loads((run / name).read_text())


def _events(run):
    return [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]


def _steps(run):
    p = run / "vla_steps.jsonl"
    return [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines()] if p.is_file() else []


def _args(*extra):
    return pn.build_parser().parse_args(["--network", "mirrored", *extra])


# =========================================================================== #
# 1. The cameras (VLA-03)
# =========================================================================== #

def _sensors(path):
    return {s["id"]: s for s in rail.load_jsonc(path)["sensors"]}


def test_the_rail_carries_aerialvlas_training_cameras_beside_the_detectors():
    """VLAFront and VLADown, exactly the CityLife VLA configs' (robot_vla_quad
    and robot_vla_quad_det): 256x256, 90 deg, pitch 0 and -90, every 0.1 s,
    scene only and never streamed. FrontCamera (OWL-ViT's 768x432 at -20
    deg, with its depth) is unchanged, the robot still passes the ArduPilot
    check, and the rendered and attach configs carry the pair."""
    ours = _sensors(rail.ROBOT_TEMPLATE)
    for name in ("robot_vla_quad.jsonc", "robot_vla_quad_det.jsonc"):
        city = _sensors(ROOT / "demo" / "pas_config" / name)
        for cid in ("VLAFront", "VLADown"):
            assert cid in ours, f"{cid} is not on the ArduPilot rail"
            assert ours[cid] == city[cid], (cid, name, ours[cid], city[cid])
    for cid, pitch in (("VLAFront", "0"), ("VLADown", "-90")):
        s = ours[cid]
        cs = s["capture-settings"]
        assert len(cs) == 1 and cs[0]["image-type"] == 0, cs
        assert (cs[0]["width"], cs[0]["height"], cs[0]["fov-degrees"]) == (256, 256, 90)
        assert cs[0]["capture-enabled"] is True and cs[0]["streaming-enabled"] is False
        assert s["capture-interval"] == 0.1 and s["enabled"] is True
        assert s["origin"]["rpy-deg"].split()[1] == pitch, s["origin"]
    fc = ours["FrontCamera"]
    assert [(c["image-type"], c["width"], c["height"]) for c in fc["capture-settings"]] == \
        [(0, 768, 432), (2, 768, 432)]
    assert fc["origin"]["rpy-deg"] == "0 -20.0 0"
    assert rail.check_robot_config(rail.load_jsonc(rail.ROBOT_TEMPLATE)) == []
    # The to-do's word for the down camera is in a key, not only a comment.
    text = rail.ROBOT_TEMPLATE.read_text(encoding="utf-8")
    assert '"id": "VLADown"' in text and '"id": "VLAFront"' in text
    assert pn.VLA_CAMERAS == {"front": "VLAFront", "down": "VLADown"}


def test_the_vla_cameras_render_only_for_a_vla_run_and_the_run_says_so():
    """Review of 4a0da6d, P2-4: in the template they rendered on EVERY run -
    two more 256x256 captures every 0.1 s beside FrontCamera and Chase on
    each detector-pilot flight, unmeasured, with only robot_sha256 to show
    it. The per-run rendered robot config now carries them only for --pilot
    vla (or --vla-cameras, the scene step an attaching VLA node relies on),
    and the record says which (sim_config.vla_cameras)."""
    for writer in (lambda d, **k: rail.write_sim_configs(d, rail.plan_network("mirrored"), **k),
                   lambda d, **k: rail.write_attach_configs(d, **k)):
        for on in (False, True):
            cfg = writer(T._tmp(), vla_cameras=on)
            robot = rail.load_jsonc(Path(cfg["dir"]) / cfg["robot"])
            ids = {s["id"] for s in robot["sensors"]}
            assert {"FrontCamera", "Chase"} <= ids, ids
            assert ({"VLAFront", "VLADown"} <= ids) is on and (not ids & {"VLAFront", "VLADown"}
                                                               if not on else True), (on, ids)
            assert cfg["vla_cameras"] is on
        assert writer(T._tmp())["vla_cameras"] is False, "the default renders no VLA camera"
    out = T._tmp()
    for argv, want in ((["--pilot", "vla", "--goal", "50,0", "--vla-server", "127.0.0.1:1"], True),
                       ([], False), (["--scene-only", "--vla-cameras"], True)):
        a = _args(*argv)
        pn.check_args(a)
        cfg, _ = pn.sim_configs(a, out / str(len(argv)))
        assert cfg["vla_cameras"] is want, (argv, cfg["vla_cameras"])
    # A servo run's metrics name it.
    rc, run, _ = T._fly("on")
    assert _json(run, "metrics.json")["sim_config"]["vla_cameras"] is False
    # The launcher's scene step can ask for them (-VlaCameras).
    ps1 = (ROOT / "scripts" / "run_pas_ardupilot.ps1").read_text(encoding="utf-8")
    assert "[switch]$VlaCameras" in ps1 and '"--vla-cameras"' in ps1


def test_a_request_can_carry_its_own_answer_timeout():
    """guardrail/vla_link.RemoteVLA.submit(response_timeout_s=...): one
    request's answer timeout instead of the client's, reported in force on
    its result (LinkResult.timeout_s) - what the warm-up records (review of
    4a0da6d, P1-1)."""
    from guardrail.vla_link import build_infer_fields, Hint
    base = _real(F.goto_responder(GOAL, 8.0))
    srv = F.FakeVLAServer(lambda req: F.Delay(1.2, base(req)), model=REAL_MODEL)
    srv.start()
    try:
        c = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=0.5, reconnect_backoff_s=0.0)
        cap = Capture(1.0, Pose(35.0, -20.0, 8.0, 0.0), JPG, JPG)
        fields, _ = build_infer_fields(cap, Hint("straight ahead ", "goal"), VM.GOTO_OBJECT_TEXT)
        assert c.submit(fields, response_timeout_s=5.0) is not None and c.wait_idle(8.0)
        res = c.poll()
        assert res.failure is None and res.reply.kind == "step" and res.timeout_s == 5.0, res
        assert c.submit(fields) is not None and c.wait_idle(8.0)
        res = c.poll()
        assert res.failure == "timeout" and res.timeout_s == 0.5 and "0.5 s" in res.detail, res
        c.close()
    finally:
        srv.stop()


def test_an_attached_vla_node_refuses_a_scene_loaded_without_the_vla_cameras():
    """Attach mode builds its robot from the record, not from what the desktop
    loaded: the live topic list must carry VLAFront / VLADown."""
    cfg = rail.write_attach_configs(T._tmp(), vla_cameras=True)
    base = "/Sim/SceneCityLifeArduPilot/robots/Drone1/sensors/"
    sensors = {"FrontCamera": {"scene_camera": "fs", "depth_camera": "fd"},
               "VLAFront": {"scene_camera": "vf"}, "VLADown": {"scene_camera": "vd"}}

    class Store:
        def put_front(self, m):
            pass

        def put_down(self, m):
            pass
    for topics, ok in (([base + "FrontCamera/scene_camera"], False),
                       ([base + "FrontCamera/scene_camera", base + "VLAFront/scene_camera",
                         base + "VLADown/scene_camera"], True)):
        pas, h = _cam_pas(sensors, topics=topics)
        cam = pn.ProjectAirSimCamera(T.FakeObs(), cfg, projectairsim=pas, attach=True,
                                     vla_frames=Store())
        try:
            cam.start()
        except RuntimeError as exc:
            assert not ok and "--vla-cameras" in str(exc), exc
        else:
            assert ok, "a scene loaded without the VLA cameras was attached to"


class _SubClient(T.FakeClient2):
    def __init__(self, address="127.0.0.1", topics=None):
        super().__init__(address, topics)
        self.cbs = {}

    def subscribe(self, topic, cb):
        super().subscribe(topic, cb)
        self.cbs[topic] = cb


def _cam_pas(sensors, topics=None):
    class Drone(T.FakeDrone):
        def __init__(self, client, world, name):
            super().__init__(client, world, name)
            self.sensors = dict(sensors)
    holder = {}

    def client(address):
        holder["c"] = _SubClient(address, topics)
        return holder["c"]
    return SimpleNamespace(ProjectAirSimClient=client, World=T.FakeWorld, Drone=Drone), holder


def test_the_camera_feeds_the_vla_pair_into_the_frame_store_and_refuses_a_robot_without_it():
    class Store:
        def __init__(self):
            self.front, self.down = [], []

        def put_front(self, m):
            self.front.append(m)

        def put_down(self, m):
            self.down.append(m)
    cfg = rail.write_sim_configs(T._tmp(), rail.plan_network("mirrored"))
    sensors = {"FrontCamera": {"scene_camera": "fs", "depth_camera": "fd"},
               "VLAFront": {"scene_camera": "vf"}, "VLADown": {"scene_camera": "vd"}}
    pas, h = _cam_pas(sensors)
    store = Store()
    cam = pn.ProjectAirSimCamera(T.FakeObs(), cfg, projectairsim=pas, vla_frames=store)
    cam.start()
    h["c"].cbs["vf"](None, {"m": "front"})
    h["c"].cbs["vd"](None, {"m": "down"})
    assert store.front == [{"m": "front"}] and store.down == [{"m": "down"}]
    assert cam.scene_record["vla_cameras"] == {"front": "VLAFront", "down": "VLADown"}
    # The detector pilot subscribes no VLA camera.
    pas, h = _cam_pas(sensors)
    cam = pn.ProjectAirSimCamera(T.FakeObs(), cfg, projectairsim=pas)
    cam.start()
    assert "vf" not in h["c"].subs and "vd" not in h["c"].subs
    # A robot config without the pair cannot fly the VLA: refused at start.
    pas, h = _cam_pas({"FrontCamera": {"scene_camera": "fs", "depth_camera": "fd"}})
    cam = pn.ProjectAirSimCamera(T.FakeObs(), cfg, projectairsim=pas, vla_frames=Store())
    try:
        cam.start()
    except RuntimeError as exc:
        assert "VLAFront" in str(exc), exc
    else:
        raise AssertionError("a robot without the VLA cameras was accepted")


# =========================================================================== #
# 2. The flag, the refusals, the server's configuration (VLA-07)
# =========================================================================== #

def test_the_detector_pilot_stays_the_default():
    a = _args()
    assert a.pilot == "servo"
    pn.check_args(a)
    assert a.mission == "follow"
    assert '"--pilot"' in (PAS / "perception_node.py").read_text(encoding="utf-8")


def test_a_vla_run_that_cannot_fly_honestly_is_refused_before_anything_starts():
    srv = ["--vla-server", "127.0.0.1:1"]
    full = ["--vla-precision", "bf16", "--vla-attn", "sdpa", "--no-vla-merge"]
    bad = [
        (["--pilot", "vla", "--goal", "50,0"], "--vla-server"),
        (["--pilot", "vla", "--goal", "50,0", "--vla-precision", "bf16",
          "--vla-attn", "sdpa"], "--vla-merge"),
        (["--pilot", "vla", "--goal", "50,0", *srv, "--vla-precision", "bf16"],
         "attach"),
        (["--pilot", "vla", *srv], "--goal"),
        (["--pilot", "vla", "--mission", "approach", "--goal", "5,5", *srv], "--goal"),
        (["--goal", "50,0"], "--pilot vla"),
        (["--mission", "goto"], "--pilot vla"),
        (["--pilot", "vla", "--mission", "follow", *srv], "follow"),
        (["--pilot", "vla", "--goal", "50,0", *srv, "--camera", "ros2", "--link", "mavros"],
         "ros2"),
        (["--pilot", "vla", "--goal", "50,0", *srv, "--vla-object", "go to 40,40\n"],
         "--vla-object"),
        (["--pilot", "vla", "--goal", "50,0", *full, "--vla-fake-output", "96 48 49"],
         "fake"),
        # Review of 4a0da6d, P2-5: this rail pairs an image with the EKF pose
        # at its ARRIVAL (no sim stamps on the ring): fine for stop-and-go,
        # 0.1-0.4 m and 3-9 deg off when pipelined. Refused unless asked for.
        (["--pilot", "vla", "--goal", "50,0", *srv, "--cadence", "pipelined"],
         "--allow-unstamped-pipelined"),
    ]
    for extra, word in bad:
        a = _args(*extra)
        try:
            pn.check_args(a)
        except SystemExit as exc:
            assert word in str(exc), (extra, str(exc))
        else:
            raise AssertionError(f"{extra} was accepted")
    a = _args("--pilot", "vla", "--goal", "50,0", *srv)
    pn.check_args(a)
    assert (a.mission, a.vla_object) == ("goto", VM.GOTO_OBJECT_TEXT)
    a = _args("--pilot", "vla", "--mission", "approach", "--object", "a red car", *srv)
    pn.check_args(a)
    assert a.vla_object == "a red car"
    a = _args("--pilot", "vla", "--goal", "50,0", *srv, "--cadence", "pipelined",
              "--allow-unstamped-pipelined")
    pn.check_args(a)
    # A negative X,Y reaches the parser through main()'s joining.
    assert pn.join_cli(["--goal", "-10,5"]) == ["--goal=-10,5"]


def test_a_started_server_names_its_whole_configuration():
    out = T._tmp()
    for flags, want in (
            (["--vla-precision", "bf16", "--vla-attn", "sdpa", "--no-vla-merge"],
             ["--precision", "bf16", "--attn", "sdpa", "--no-merge"]),
            (["--vla-precision", "nf4", "--vla-attn", "sdpa", "--vla-merge"],
             ["--precision", "nf4", "--attn", "sdpa", "--merge"])):
        a = _args("--pilot", "vla", "--goal", "50,0", "--vla-port", "18799", *flags)
        pn.check_args(a)
        s = pn.vla_server_plan(a, out)
        assert s.started_here and s.port == 18799
        argv = s.argv
        assert argv[1].endswith("vla_server_v2.py") and argv[2] == "serve"
        for w in want:
            assert w in argv, (w, argv)
        assert argv[argv.index("--missing-down") + 1] == "refuse"
        assert "--fake-output" not in argv
    a = _args("--pilot", "vla", "--goal", "50,0", "--vla-fake-output", "96 48 49</s>")
    pn.check_args(a)
    s = pn.vla_server_plan(a, out)
    assert "--fake-output" in s.argv and "--precision" not in s.argv
    a = _args("--pilot", "vla", "--goal", "50,0", "--vla-server", "10.0.0.5:8791")
    pn.check_args(a)
    s = pn.vla_server_plan(a, out)
    assert not s.started_here and (s.host, s.port) == ("10.0.0.5", 8791)


# =========================================================================== #
# 3. A GOTO through the Shield and the FSM, landed on the model's LAND
# =========================================================================== #

def test_a_vla_goto_flies_the_trackers_action_through_the_shield_and_lands_on_its_land():
    import todo as TODO
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)))
    assert r.rc == 0
    m = _json(r.run, "metrics.json")
    assert m["end_reason"] == "vla_land", m["end_reason"]
    # The warm-up: three requests, all before the aircraft was armed.
    assert r.link.armed_in_before_arm is False and r.link.requests_at_arm == 3
    w = m["vla"]["warmup"]
    assert w["pass"] is True and len(w["attempts"]) == 3 and w["judged"] == "last of attempts"
    assert all(e.get("warmup") for e in r.frames.log[:3])
    # The flight: only the Shield's output reached the autopilot, the FSM
    # stepped on every tick, every row names its author.
    rows = _rows(r.run)
    assert len(rows) > 100 and len(r.link.sent) == len(rows)
    for row, sent in zip(rows, r.link.sent):
        assert sent.model_dump() == row["flown_action"] == row["emitted"]
        assert row["pilot"] == "vla" and row["author"] in AUTHORS
        assert row["fsm_state_after"] is not None and row["setpoint"] == "pass"
    authors = [x["author"] for x in rows]
    assert authors.count("vla_step") > len(rows) / 2, authors[:40]
    steps = [x for x in rows if x["author"] == "vla_step"]
    assert all(x["hint_source"] == "goal" and x["phrase"] in PHRASES and x["step"]
               and x["vla_seq"] is not None for x in steps)
    # The step tracker's world-frame action IS the raw action (nothing in
    # between): a vla_step tick's raw is the tracker's speed, at most cruise.
    assert max(math.hypot(x["raw"]["vx"], x["raw"]["vy"]) for x in steps) <= 1.0 + 1e-6
    # LAND: the model's flag, landed by the node's mission-end LAND.
    assert rows[-1]["land_requested"] is True
    assert r.link.requested == ["LAND"], r.link.requested
    assert m["mission_end_land_accepted"] is True
    assert math.hypot(r.link.x - GOAL[0], r.link.y - GOAL[1]) < 3.5, (r.link.x, r.link.y)
    # The requests: goal phrases, no coordinates, one outstanding at a time.
    assert r.srv.refused == [] and r.srv.overlaps == 0
    for q in r.srv.requests:
        assert q["hint_source"] == "goal" and not ({"target", "goal", "xy"} & set(q))
    assert r.srv.requests[-1]["phrase"] == ""
    # The records, as on the city rail.
    lines = _steps(r.run)
    assert len(lines) == len(r.srv.requests) - 3, (len(lines), len(r.srv.requests))
    p = m["pilot"]
    assert p["kind"] == "vla" and p["model_id"] == VM.MODEL_ID
    assert p["reported_digest"] == REAL_MODEL["weights_sha256"]
    s = p["summary"]
    assert s["decisions_accepted"] >= 10 and s["land"] == 1 and s["parse_fail"] == 0
    assert p["cadence"] == "stop_and_go" and s["vla_authored_tick_share"] > 0.5
    man = _json(r.run, "manifest.json")
    assert man["vla_model_hash"] == p["hash"]
    assert m["reached_goal"] is True and m["land"]["requested_by_vla"] is True
    # Review of 4a0da6d, P2-2: a GOTO follows nothing, so no follow score is
    # owed: kpi.json decides the mission on reached_goal, not on an
    # "unscored follow" (which made a successful GOTO's mission_success None).
    assert m["follow_scored"] is None and m["frac_within_30m"] is None, m["follow_scored"]
    k = _json(r.run, "kpi.json")
    assert "follow_not_scored" not in k["mission_fail_reasons"], k["mission_fail_reasons"]
    assert k["mission_success"] is True, (k["mission_success"], k["mission_fail_reasons"])
    # P2-5: how each capture's time and pose were taken, said.
    fr = m["vla"]["frames"]
    assert fr["capture_time_basis"].startswith("arrival"), fr
    # P2-7: the GPU's memory at hello and after the warm-up, best effort (no
    # nvidia-smi asked here: the error is recorded, never a made-up number).
    g = m["vla"]["gpu_memory"]
    assert set(g) == {"at_hello", "after_warmup"} and "error" in g["at_hello"], g
    g = m["start_gate"]
    assert g["enabled"] is False and "goto" in g["why"], g
    assert m["detector_ready"]["outcome"] == "not_used" and m["det_hz"] is None
    assert m["params"]["pilot"] == "vla" and m["params"]["mission"] == "goto"
    ev = TODO.vla_evidence(r.run)
    assert TODO.classify_run(man["vla_model_hash"], ev) == ("vla", None), ev
    import build_dashboard as BD
    fr = BD.flight_record(ROOT, r.run)
    assert fr["pilot"]["class"] == "vla" and fr["pilot"]["metrics_kind"] == "vla", fr["pilot"]
    assert fr["end_reason"] == "vla_land" and fr["shield"] == "on"
    # The mission's CSP context is the goal, not a follow.
    mission = next(e for e in _events(r.run) if e["kind"] == "mission")
    assert mission["target"] == list(GOAL) and "goal" in mission["task"]


def test_the_control_arm_flies_the_trackers_action_clamped_only_to_the_sim_band():
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)), shield="off", max_s="20")
    m = _json(r.run, "metrics.json")
    rows = _rows(r.run)
    assert rows and all(x["flown_action"] == x["raw"] and x["setpoint"] == "pass" for x in rows)
    assert m["vla"]["pilot_alt_band"]["clamp"] == "sim_safety_band"
    assert m["vla"]["pilot_alt_band"]["applied"]["band"] == list(VM.SIM_SAFETY_BAND_M)


# =========================================================================== #
# 4. An FSM hold suspends the tracker
# =========================================================================== #

def test_an_fsm_hold_suspends_the_tracker_and_nothing_decided_before_it_flies():
    """LOITER_POLICY's ceiling (6 m) is below the take-off height (8 m), with
    a direct loiter: the FSM goes Normal -> Loiter (flown as BRAKE) on the
    first tick and holds. The tracker is suspended from the first held tick
    on: every later row says so (author none, suspended), its raw action is
    zero, no request goes out while it is held, and the step behind the
    first held tick ended "suspended" without its action ever being flown."""
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)), policy=T.LOITER_POLICY, max_s="4")
    m = _json(r.run, "metrics.json")
    rows = _rows(r.run)
    assert r.link.requested[0] == "BRAKE", r.link.requested
    held = [x for x in rows if x["pilot_held"]]
    assert held and len(rows) > 20, [(x["setpoint"], x["stop_advised"]) for x in rows][:10]
    later = rows[rows.index(held[0]) + 1:]
    assert later and all(x["author"] == "none" and x["author_reason"] == "suspended"
                         and x["pilot_held"] for x in later), \
        [(x["author"], x["author_reason"]) for x in later][:5]
    assert all(x["raw"] == {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
               for x in later)
    assert m["vla"]["held_ticks"] == len(held)
    assert m["pilot"]["vla"]["tracker"].get("suspends", 0) >= 1
    lines = _steps(r.run)
    assert len(r.srv.requests) - 3 == len(lines) <= 1, (len(r.srv.requests), lines)
    assert all(not l["accepted"] or (l["outcome"] or {}).get("end") == "suspended"
               for l in lines), [(l.get("refused"), l.get("outcome")) for l in lines]
    for x in held:
        if x["author"] == "vla_step":
            assert x["flown_action"] is None or x["flown_action"] != x["raw"], x
    # Review of 4a0da6d, P3-12: a held tick is no VLA-flown tick, whoever
    # authored its raw action: the shares count none of them.
    s = m["pilot"]["summary"]
    assert s["ticks_held_by_rail"] == len(held) and s["vla_step_tick_share"] == 0.0, s
    assert s["vla_authored_tick_share"] == 0.0, s
    # P3-13: the FSM stepped on a raw action the rail itself had zeroed (the
    # suspended tracker's): said per row and counted.
    assert all(x["raw_zeroed_by_hold"] for x in later)
    assert m["vla"]["fsm_ticks_on_zeroed_raw"] == len(later), m["vla"]["fsm_ticks_on_zeroed_raw"]


def test_a_hold_pauses_the_vla_missions_stop_rules():
    """Review of 4a0da6d, P3-9. The mission monitor kept judging while the
    rail held the aircraft, so the VLA mission's own rules could end - and
    name - a mission the FSM was holding: here the aircraft is held inside
    the arrive radius of a goal 1 m away by an answer that never lands, and
    `arrived_no_land` ended the run 2 s into the FSM's Loiter, which then
    never got to escalate. Its timers now stop while the tracker is
    suspended: the run ends on the rail's own (the FSM's) end."""
    r = _vfly(_real(lambda req: F.step_reply(req, 0.0, 0.0, 0.0)), goal=(36.0, -20.0),
              policy=T.LOITER_POLICY, max_s="8", extra=("--arrive-grace-s", "2"))
    m = _json(r.run, "metrics.json")
    assert m["end_reason"] != "arrived_no_land" and r.rc == 0, (m["end_reason"], r.rc)
    assert m["end_reason"].startswith(("escalation FSM", "max_s")), m["end_reason"]
    mon = m["vla"]["mission"]["monitor"]
    assert mon["end_reason"] is None and mon["held_s"] > 2.0, mon


class FreezingVLALink(VLALink):
    """VLALink whose SITL goes quiet from `die_at` to `back_at` s after the
    arming (tests/test_pas_ardupilot.FreezingLink's span, airborne only)."""

    def __init__(self, clock, anchor, on_arm=None, die_at=4.0, back_at=6.0, **kw):
        super().__init__(clock, anchor, on_arm=on_arm, **kw)
        self.die_at, self.back_at = die_at, back_at
        self.t_arm = None

    def bring_up(self, alt, before_arm=None):
        super().bring_up(alt, before_arm=before_arm)
        self.t_arm = self.clock.t

    def pump(self):
        if self.t_arm is not None and self.die_at <= self.clock.t - self.t_arm < self.back_at:
            return
        super().pump()


def test_a_stale_link_span_suspends_the_tracker():
    """Review of 4a0da6d, P3-13. Through a stale-link span nothing is
    commanded but one zero hold (STALE_HOLD); the step tracker went on as if
    its step were flying. It is now suspended for the span: the first row
    after it shows the tracker suspended, then a fresh answer is awaited."""
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)), max_s="9", link_cls=FreezingVLALink)
    m = _json(r.run, "metrics.json")
    rows = _rows(r.run)
    assert m["ticks_state_stale"] > 0, m["link_freshness"]
    gap = next(i for i in range(1, len(rows)) if rows[i]["t"] - rows[i - 1]["t"] > 0.5)
    after = rows[gap]
    assert (after["author"], after["author_reason"]) == ("none", "suspended"), after
    assert rows[gap + 1]["author_reason"] in ("awaiting_fresh_decision", "translate", "rotate",
                                              "step_complete"), rows[gap + 1]["author_reason"]
    assert m["pilot"]["vla"]["tracker"]["suspends"] >= 1


def test_the_pilot_is_suspended_while_held_and_resumed_when_it_flies_again():
    """VLAPilot.after's wiring, on a stand-in backend: a tick whose action did
    not fly suspends the tracker, the next one that flies resumes it (the
    tracker then refuses every answer to an image from before:
    vla_link.StepTracker.resume, tests/test_vla_link.py)."""
    from guardrail.models import Action4D, State
    from guardrail.vla_link import Tick

    class Backend:
        cadence = "stop_and_go"
        last_link_failure = None
        tracker = SimpleNamespace(current_record=None)

        def __init__(self):
            self.calls = []
            self.last_tick = None

        def act(self, st):
            self.last_tick = Tick(Action4D(), "none", "no_decision")
            return Action4D()

        def suspend(self):
            self.calls.append("suspend")

        def resume(self):
            self.calls.append("resume")
    be = Backend()
    p = pn.VLAPilot(be, VM.MissionMonitor(VM.MonitorConfig(mission="goto"), 0.0),
                    mission="goto", goal_xy=(10.0, 0.0), vm=VM)
    st = State(x=0.0, y=0.0, up=8.0, yaw_deg=0.0)
    for i, flies in enumerate((True, False, False, True)):
        raw, body, info = p.step(0.1 * i, st)
        end, row, hud = p.after(0.1 * i, st, flies=flies, touched=False, shield_on=True,
                                t_mission=0.1 * i)
        assert end is None and row["pilot_held"] is (not flies) and row["author"] == "none"
        assert info["mode"] == "vla" and info["fresh"] is False and body == (0.0, 0.0, 0.0, 0.0)
    assert be.calls == ["resume", "suspend", "suspend", "resume"] and p.n_held == 2


# =========================================================================== #
# 5. The warm-up gate
# =========================================================================== #

def test_the_warm_up_refuses_a_server_that_refuses_and_nothing_is_armed():
    """Nothing is armed or sent, and (review of 4a0da6d, P2-3) the refusal is
    on record as the city rail records one: metrics.json (end_reason
    warmup_refused, the warm-up, the pilot record), manifest.json (so the
    Dashboard lists it), vla_captures.jsonl with the warm-up captures, the
    episode's mission_end, the server's record, and a Dashboard row that
    ends "failed" saying why; exit 4."""
    import build_dashboard as BD
    live = T._tmp() / "live"
    r = _vfly(_real(lambda req: F.error_reply(req, "inference failed: CUDA out of memory")),
              live_dir=live)
    assert r.rc == pn.EXIT_REFUSED, r.rc
    assert r.link.snap.armed is False and r.link.requests_at_arm is None
    assert not r.link.sent and "GUIDED" not in [e.get("mode") for e in r.link.events]
    ev = [e for e in _events(r.run) if e["kind"] == "vla_warmup"]
    assert len(ev) == 1 and ev[0]["pass"] is False and len(ev[0]["attempts"]) == 3, ev
    assert len(r.srv.refused) == 0 and len(r.srv.requests) == 3
    m = _json(r.run, "metrics.json")
    assert m["end_reason"] == "warmup_refused" and m["ticks"] == 0, m["end_reason"]
    assert m["refused"]["kind"] == "warmup_refused" and "refused the warm-up" in m["refused"]["why"]
    assert m["vla"]["warmup"]["pass"] is False and m["pilot"]["kind"] == "vla"
    assert m["pilot"]["server"]["mode"] == "attached"
    man = _json(r.run, "manifest.json")
    assert man["vla_model_hash"] == m["pilot"]["hash"]
    caps = [json.loads(l) for l in (r.run / "vla_captures.jsonl").read_text().splitlines()]
    assert len(caps) == 3 and all(c.get("warmup") for c in caps), caps
    end = [e for e in _events(r.run) if e["kind"] == "mission_end"]
    assert len(end) == 1 and "warmup_refused" in end[0]["why"], end
    done = sorted((live / "_done").glob("*.json"))
    rec = json.loads(done[0].read_text(encoding="utf-8"))
    assert rec["status"] == "failed" and "refused" in (rec.get("detail") or ""), rec
    fr = BD.flight_record(ROOT, r.run)
    assert fr is not None and fr["end_reason"] == "warmup_refused", fr


def test_a_slow_cold_first_answer_gets_its_own_timeout_and_the_warm_up_passes():
    """Review of 4a0da6d, P1-1. The first warm-up request is the model's
    first inference ever (a fresh server does not warm itself) and was
    promised first_decision_timeout_s (60 s) - recorded so - while the
    client's socket gave up at its own response timeout (the flight's
    decision timeout), then drained the late answer, so the next two
    requests were "not sent" and a healthy server was refused. Now every
    warm-up request carries its own response timeout, the one recorded, and
    each waits until the client is ready (vla_mission.wait_client_ready).
    Here: a 3 s first answer, a client that times out at 1 s and waits 120 s
    for late answers, as the flight's does."""
    base = _real(F.goto_responder(GOAL, 8.0))
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        out = base(req)
        return F.Delay(3.0, out) if n["k"] == 1 else out
    r = _vfly(respond, max_s="3",
              client_kw={"response_timeout_s": 1.0, "late_reply_wait_s": 120.0})
    w = _json(r.run, "metrics.json")["vla"]["warmup"]
    assert w["pass"] is True, w["why"]
    assert [a["timeout_s"] for a in w["attempts"]] == [60.0, 10.0, 10.0], w["attempts"]
    assert [a["kind"] for a in w["attempts"]] == ["step"] * 3
    assert all(a["client_ready"]["ready"] for a in w["attempts"]), w["attempts"]
    assert r.link.requests_at_arm == 3 and r.rc == 0


def test_the_warm_up_judges_the_last_of_its_three_requests():
    """The first request pays the model's cold start; only the last decides."""
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        if n["k"] == 1:
            return F.error_reply(req, "warming up")
        return F.goto_responder(GOAL, 8.0)(req)
    r = _vfly(_real(respond), max_s="5")
    w = _json(r.run, "metrics.json")["vla"]["warmup"]
    assert w["pass"] is True and w["attempts"][0]["kind"] == "server_error", w["attempts"]
    # The helper itself (vla_mission.run_warmup): the last attempt refuses.
    seq = iter(range(1, 10))

    class Client:
        late = []

        def submit(self, f, response_timeout_s=None):
            return next(seq)

        def wait_idle(self, t):
            return True

        def poll(self):
            return None
    cap = Capture(1.0, Pose(0, 0, 8, 0), JPG, JPG)
    res = VM.run_warmup(Client(), lambda: cap, VM.goto_hint_source(GOAL, 3.0), "goto",
                        VM.GOTO_OBJECT_TEXT, decision_timeout_s=10.0,
                        first_decision_timeout_s=60.0, max_latency_s=5.0)
    assert res["pass"] is False and len(res["attempts"]) == 3
    assert [a["timeout_s"] for a in res["attempts"]] == [60.0, 10.0, 10.0]
    res = VM.run_warmup(Client(), lambda: None, None, "goto", VM.GOTO_OBJECT_TEXT,
                        decision_timeout_s=10.0, first_decision_timeout_s=60.0,
                        max_latency_s=5.0)
    assert res["pass"] is False and "camera pair" in res["why"]

    # While an answer is awaited the rail's `idle` (its link pump) runs: a
    # cold first inference of a minute leaves no minute of MAVLink unread.
    class SlowClient(Client):
        def __init__(self):
            self.waits = 0

        def wait_idle(self, t):
            self.waits += 1
            return self.waits % 3 == 0          # busy twice, then answered
    pumped = []
    VM.run_warmup(SlowClient(), lambda: cap, VM.goto_hint_source(GOAL, 3.0), "goto",
                  VM.GOTO_OBJECT_TEXT, decision_timeout_s=10.0, first_decision_timeout_s=60.0,
                  max_latency_s=5.0, idle=lambda: pumped.append(1))
    assert len(pumped) == 6, len(pumped)


def test_the_client_readiness_wait_covers_every_way_a_client_cannot_send():
    """vla_mission.wait_client_ready (review of 4a0da6d, P1-1, P2-8, P3-15):
    a request in flight, a timed-out request's late answer still being
    awaited, an unpolled result and the backoff after a failure all keep a
    warm-up request from going out; the wait covers each, bounded, calling
    `idle` (the rail's link pump) meanwhile, and says what it waited on."""
    class C:
        def __init__(self, states):
            self.states = list(states)          # (busy, draining, backing_off) per look
            self.polled = 0

        def _s(self):
            return self.states[0] if self.states else (False, False, False)

        @property
        def busy(self):
            return self._s()[0]

        @property
        def draining(self):
            return self._s()[1]

        @property
        def backing_off(self):
            return self._s()[2]

        def _step(self):
            if self.states:
                self.states.pop(0)

        def wait_idle(self, t):
            return True                         # nothing in flight

        def wait_drained(self, t):
            self._step()
            return True

        def poll(self):
            self.polled += 1
            self._step()
            return None
    clk = T.Clock()
    idles = []
    c = C([(True, True, False), (True, True, False), (True, False, False),
           (False, False, True), (False, False, True)])

    def sleep(dt):
        c._step()
        clk.sleep(dt)
    got = VM.wait_client_ready(c, clock=clk, sleep=sleep, idle=lambda: idles.append(1))
    assert got["ready"] is True and len(idles) == 5, (got, len(idles))
    assert got["waited_on"] == {"late_reply": 2, "unpolled_result": 1, "backoff": 2}, got
    # Bounded: a client that never becomes ready is reported, not waited on.
    stuck = C([(False, False, True)] * 1000)
    got = VM.wait_client_ready(stuck, clock=clk, sleep=lambda dt: clk.sleep(dt), max_wait_s=1.0)
    assert got["ready"] is False and "backoff" in got["why"], got


def test_the_gpu_memory_is_read_best_effort():
    """Review of 4a0da6d, P2-7: free VRAM at the hello and after the warm-up
    (bf16 ~17 GB + OWL-ViT + Unreal on 24 GB is unmeasured); nvidia-smi
    with a short timeout, never a GPU library in the flight process."""
    def smi(cmd, **kw):
        assert cmd[0] == "nvidia-smi" and kw.get("timeout"), (cmd, kw)
        assert "--query-gpu=memory.free,memory.used,memory.total" in cmd
        return SimpleNamespace(returncode=0, stdout="9000, 15000, 24564\n", stderr="")
    g = pn.gpu_memory(smi)
    assert (g["free_mib"], g["used_mib"], g["total_mib"], g["source"]) == \
        (9000, 15000, 24564, "nvidia-smi"), g
    g = pn.gpu_memory(_no_gpu_query)
    assert set(g) == {"error"} and "FileNotFoundError" in g["error"], g
    g = pn.gpu_memory(lambda cmd, **kw: SimpleNamespace(returncode=9, stdout="", stderr="no"))
    assert "error" in g and "9" in g["error"], g


def test_the_frame_store_says_how_each_capture_was_timed_and_posed():
    """Review of 4a0da6d, P2-5: vla_mission.VLAFrames' stats count each sent
    capture's pose source and capture-time basis: "arrival" where the pose
    ring has no sim stamps (this rail: EKF poses), "stamp" where the frame's
    sim stamp mapped through the ring."""
    class Obs:
        def __init__(self, wall):
            self.wall = wall

        def pose_at(self, t, stamp=None):
            return {"x": 1.0, "y": 2.0, "up": 8.0, "yaw": 0.0, "src": "ring"}

        def wall_at_stamp(self, stamp):
            return self.wall
    for wall, basis in ((None, "arrival"), (9.95, "stamp")):
        fr = VM.VLAFrames(Obs(wall), encode=lambda m: b"jpg", clock=lambda: 10.0)
        fr.put_front({"time_stamp": 5e9, "width": 256, "height": 256})
        fr.put_down({"time_stamp": 5e9, "width": 256, "height": 256})
        cap = fr.capture()
        fr.commit(cap, None, 1)
        s = fr.stats()
        assert s["pose_src"] == {"ring": 1} and s["t_capture_basis"] == {basis: 1}, s


def test_an_approach_is_scored_on_the_subjects_true_range():
    """Review of 4a0da6d, P3-11: the approach's reached_goal comes from the
    subject's simulator range per row (evaluation only), None with why when
    the truth did not score."""
    rows = [{"subject_range_m": 30.0}, {"subject_range_m": None}, {"subject_range_m": 8.0}]
    s = pn.approach_scores(rows, 10.0, scored=True)
    assert s["reached_goal"] is True and s["target_truth_dist_end_m"] == 8.0
    assert s["target_truth_dist_min_m"] == 8.0 and s["reached_radius_m"] == 10.0
    s = pn.approach_scores(rows[:1], 10.0, scored=True)
    assert s["reached_goal"] is False
    s = pn.approach_scores(rows, 10.0, scored=False)
    assert s["reached_goal"] is None and "subject" in s["reached_goal_why"], s


# The example goal docs/DESIGN-vla-pilot.md section 14 gives for this rail
# (review of 4a0da6d, P2-6: the first one, 60,-20, was an occupied cell; the
# second, 50,-78, lay 17 m off the map of its day and its line crosses a
# building on the map of the whole level, 2026-11-04).
DOC_EXAMPLE_GOAL = (46.0, -52.0)


def test_the_documented_example_goal_is_clear_from_the_spawn():
    """On the Shield's own distance field over the 6-14 m band map (gitignored
    demo/out/citymap_citylife: SKIP where it is absent), as test_vla_fly
    checks the city rail's: the straight line from this rail's spawn (35,
    -20) to the example goal keeps 3 m, the goal is clear and on the street
    mask (4.08 m on the line, the spawn's own, and 10.0 m at the goal,
    measured 2026-11-04 on the whole level's map and on the 360 x 200 m map
    before it). The goal must lie ON the map: off it the Shield's clearance
    is an estimate that assumes every obstacle is on the map, which is how
    50,-78 read 17.0 m."""
    import numpy as np
    from guardrail import Shield
    from guardrail.models import load_policy
    doc = (ROOT / "docs" / "DESIGN-vla-pilot.md").read_text(encoding="utf-8")
    assert "--goal 46,-52" in doc and "60,-20" not in doc and "50,-78" not in doc
    d = ROOT / "demo" / "out" / "citymap_citylife"
    if not (d / "occ_day_flightband_6to14.npz").is_file():
        print("      (no demo/out/citymap_citylife here: the level's map is gitignored; "
              "the doc names the checked goal)")
        return SKIP
    z = np.load(d / "occ_day_flightband_6to14.npz")
    occ = {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["origin_x"]),
           "oy": float(z["origin_y"])}
    sh = Shield(load_policy(ROOT / "policies" / "vla_goto_citylife.yaml"), obstacle_map=occ,
                escalation=False)
    (sx, sy), (gx, gy) = START, DOC_EXAMPLE_GOAL
    line = [sh.clearance_at(sx + f * (gx - sx), sy + f * (gy - sy)) for f in np.linspace(0, 1, 400)]
    assert min(line) >= 3.0 and sh.clearance_at(gx, gy) >= 9.0, (min(line), sh.clearance_at(gx, gy))
    assert not sh.off_map(gx, gy), "the goal is off the map: its clearance is not measured"
    st = np.load(d / "street.npz")["street"]
    assert st[int(round((gx - occ["ox"]) / 2.0)), int(round((gy - occ["oy"]) / 2.0))]


# =========================================================================== #
# 6. A dead server, a stand-in
# =========================================================================== #

def test_a_dead_server_hovers_ends_the_mission_after_its_timeout_and_lands():
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        return F.goto_responder(GOAL, 8.0)(req) if n["k"] <= 3 else F.CLOSE
    r = _vfly(_real(respond), extra=("--dead-timeout-s", "2"), max_s="30")
    m = _json(r.run, "metrics.json")
    assert m["end_reason"] in ("server_dead", "server_slow"), m["end_reason"]
    assert r.rc == pn.EXIT_VLA_FAILURE
    assert r.link.requested == ["LAND"]
    rows = _rows(r.run)
    assert "vla_step" not in {x["author"] for x in rows}
    assert {x["author"] for x in rows} <= {"none", "hover"}
    assert m["pilot"]["summary"]["link_failure"] >= 1
    assert m["vla"]["mission"]["monitor"]["vla_failure"] is True


def test_a_stand_in_is_refused_unless_allowed_and_then_recorded_as_a_stub():
    import todo as TODO
    stand_in = dict(F.MODEL)                       # weights "0" * 64: no model
    r = _vfly(F.goto_responder(GOAL, 8.0), stand_in, raises=SystemExit)
    assert "not AerialVLA" in str(r.exc) and r.link.snap.armed is not True, r.exc
    assert not r.link.sent and not r.srv.requests
    r = _vfly(F.goto_responder(GOAL, 8.0), stand_in, extra=("--allow-stand-in",), max_s="20")
    m = _json(r.run, "metrics.json")
    assert m["pilot"]["kind"] == "stub" and "stand-in" in m["pilot"]["not_a_vla_because"]
    man = _json(r.run, "manifest.json")
    assert man["vla_model_hash"].startswith("fake:")
    assert TODO.classify_run(man["vla_model_hash"], TODO.vla_evidence(r.run))[0] != "vla"
    assert any("stand-in" in w for w in _json(r.run, "kpi.json")["kpi_grade_reasons"])


# =========================================================================== #
# 7. The approach: the detector box's phrase
# =========================================================================== #

def test_the_approach_asks_with_the_detector_boxs_phrase_and_keeps_the_subject():
    def grounder(clk):
        return T.FakeDetections(clk, cx=400.0, w=60.0)
    r = _vfly(_real(lambda req: F.step_reply(req, 2.0, 0.0, 0.0)), mission="approach",
              grounder=grounder, max_s="12", extra=("--object", "a red car"))
    assert r.rc == 0
    flown = r.srv.requests[3:]
    assert flown and all(q["hint_source"] == "detector" for q in flown), flown[:2]
    assert {q["phrase"] for q in flown} <= {"straight ahead ", "forward-right ",
                                            "forward-left "}
    assert all(q["object"] == "a red car" for q in flown)
    m = _json(r.run, "metrics.json")
    assert m["subject_known_ticks"] > 0, "the box no longer gives the Shield its subject"
    assert m["vla"]["mission"]["hint_source"] == "detector"
    assert m["vla"]["mission"]["detector_hint"]["hints"] >= len(flown)
    # Review of 4a0da6d, P3-10: what ran is said, not a source-code label;
    # P3-11: no subject truth here (the stand-in camera reads none), so no
    # approach score, and why.
    assert m["vla"]["mission"]["detector"] == pn._fv().DETECTOR_ID
    assert m["vla"]["start_gate"].startswith("not run"), m["vla"]["start_gate"]
    assert m["reached_goal"] is None and "subject" in m["reached_goal_why"], m.get("reached_goal_why")
    rows = _rows(r.run)
    assert any(x["det"] is not None for x in rows)


# =========================================================================== #
# 8. A Ctrl+C, a crash after the take-off
# =========================================================================== #

def test_a_ctrl_c_in_the_vla_mission_lands_closes_its_records_and_ends_the_row_stopped():
    live = T._tmp() / "live"
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)), frames_kw={"interrupt_at": 6},
              live_dir=live)
    assert r.rc == 0
    m = _json(r.run, "metrics.json")
    assert m["end_reason"] == "interrupted" and r.link.requested == ["LAND"]
    lines = _steps(r.run)
    assert len(lines) == len(r.srv.requests) - 3, (len(lines), len(r.srv.requests))
    done = sorted((live / "_done").glob("*.json"))
    assert len(done) == 1, list(live.rglob("*"))
    rec = json.loads(done[0].read_text(encoding="utf-8"))
    assert rec["status"] == "stopped", rec


def test_a_crash_in_the_vla_set_up_after_the_take_off_is_landed_by_the_net():
    real = VM.MissionMonitor

    def boom(*a, **k):
        raise RuntimeError("monitor exploded")
    VM.MissionMonitor = boom
    try:
        r = _vfly(_real(F.goto_responder(GOAL, 8.0)), raises=RuntimeError)
    finally:
        VM.MissionMonitor = real
    assert "monitor exploded" in str(r.exc)
    assert r.link.requested == ["LAND"], r.link.requested
    end = [e for e in _events(r.run) if e["kind"] == "mission_end"]
    assert len(end) == 1 and "crashed during the mission's start" in end[0]["why"], end
    # Review of 4a0da6d, P3-15: the captures sent are on record after a
    # crash too (written on the way out).
    caps = [json.loads(l) for l in (r.run / "vla_captures.jsonl").read_text().splitlines()]
    assert len(caps) == 3 and all(c.get("warmup") for c in caps), caps


# =========================================================================== #
# 9. The HUD panel
# =========================================================================== #

class SpyRecorder:
    made: list = []

    def __init__(self, obs, out_dir, annotate_fn, hz=20.0, height=720, quality=88,
                 live_view=False, live_height=480):
        self.annotate = annotate_fn
        self.huds = []
        SpyRecorder.made.append(self)

    def start(self):
        pass

    def set_hud(self, hud, det):
        self.huds.append(hud)

    def stop(self):
        pass

    def join(self, timeout=None):
        pass

    def write_sidecar(self, path):
        s = {"frames": 0, "target_hz": 20.0, "achieved_hz": None, "frame_t": []}
        Path(path).write_text(json.dumps(s), encoding="utf-8")
        return s


class FrameCam(T.FakeCamera):
    def get_front_native(self):
        return None


def _hud_run():
    SpyRecorder.made.clear()
    out = _vfly(_real(F.goto_responder(GOAL, 8.0)), max_s="15", recorder_cls=SpyRecorder,
                extra=("--save-view", "--no-policy-hud"), camera=FrameCam(None))
    assert len(SpyRecorder.made) == 1
    return out, SpyRecorder.made[0]


def test_the_hud_panel_is_the_city_rails_fed_per_tick():
    import vla_fly
    assert vla_fly.hud_lines is VM.hud_lines and vla_fly.annotate_vla is VM.annotate_vla
    out, rec = _hud_run()
    assert rec.annotate is VM.annotate_vla
    huds = rec.huds
    assert huds and len(huds) == len(_rows(out.run))
    h = huds[-1]
    for k in ("t", "alt", "author", "reason", "seq", "age_s", "phrase", "hint_source",
              "step", "land", "shield_on", "cadence", "goal_dist_m", "policy"):
        assert k in h, k
    lines = VM.hud_lines(h)
    assert len(lines) == 5 and "stop-and-go" in lines[1]
    assert any(VM.AUTHOR_LABEL["vla_step"] in VM.hud_lines(x)[4] for x in huds)
    # Review of 4a0da6d, P3-14: the rail's own label (the FSM's state, the
    # autopilot taking over, SHIELD OFF) stays on the panel.
    assert h["rail_label"] == "FSM Normal", h.get("rail_label")
    assert VM.panel_lines(h) == lines + ["FSM Normal"]
    assert VM.panel_lines({k: v for k, v in h.items() if k != "rail_label"}) == lines


def test_the_vla_panel_draws_on_this_rails_frames():
    """The recorder thread's annotate (vla_mission.annotate_vla) on the HUD
    dicts this rail hands it: Pillow only (lab; CI installs none: SKIP)."""
    try:
        from PIL import Image
    except ImportError:
        print("      (Pillow is not installed: the panel is not drawn here)")
        return SKIP
    _, rec = _hud_run()
    base = Image.new("RGB", (768, 432), (40, 40, 40))
    for h in (rec.huds[0], rec.huds[len(rec.huds) // 2], rec.huds[-1]):
        im = rec.annotate(base, None, h)
        assert im.size == base.size and im is not base
        assert im.tobytes() != base.tobytes(), "nothing was drawn"


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
        except BaseException as e:                   # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
