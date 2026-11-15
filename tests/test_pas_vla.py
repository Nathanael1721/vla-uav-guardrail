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

11. THE VLA FOLLOW (2026-11-06): --pilot vla --mission follow flies the
    lab's AIOT-VLA (a scripted server announcing an aiotvla identity that
    reads the detector's camera) on demo/vla_fly.py's M3 parts - its hint
    (memory, search, the close rule, each hint's kind) and its follow
    tracker, built by vla_fly itself; a close box holds the horizontal
    motion; a follow that never asks ends subject_not_acquired, not as a
    VLA failure; --vla-front-source detector routes the FrontCamera frame
    into the request and needs no VLAFront; the server's declared camera is
    checked against the flag; the server's own host is the run's vla_host
    and decides whether it can be hil; the object words are recorded as
    AIOT-VLA's grounding phrase; the observation rate counts distinct
    front frames sent, apart from duplicates and hint-gated spans; and
    FrontCamera's capture interval renders only when asked, recorded.

12. CAMERA TO SETPOINT (2026-11-11; WP3-13): every row carries its
    setpoint's publication time `t_pub` and `tick_ms`, both on the loop
    clock; a VLA row whose decision's action flew carries `cam_age_ms` =
    the publication minus that decision's t_capture (vla_steps.jsonl), and
    `cam_to_setpoint_ms` on the first tick that flies each decision only;
    a held tick (the FSM's hold) carries neither; metrics.json
    `camera_to_setpoint` summarises the rows (nearest rank, the count over
    100 ms) and says which clock and that capture time is arrival time.
    Shown failing first: against 0df7ee8 none of the fields existed.

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
import vla_server_foreign as SF                                      # noqa: E402
import topology as topo                                              # noqa: E402
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
# Where a VLA server runs, as its hello reports it (guardrail.manifest.
# collect_host_evidence's keys): the AGX Orin, and the desktop.
ORIN_HOST = {"host_arch": "aarch64", "device_model": "NVIDIA Jetson AGX Orin Developer Kit",
             "l4t_release": "R39.2.1", "jetpack": None}
DESKTOP_HOST = {"host_arch": "AMD64", "device_model": None, "l4t_release": None,
                "jetpack": None}


def _aiot_model(front="detector", host=ORIN_HOST):
    """The hello of demo/vla_server_foreign.py serving AIOT-VLA (v10b reads
    the detector's FrontCamera), built by that server's own identity
    function, with the host it reports (None: a hello without one)."""
    ident = SF.foreign_identity("aiotvla", name="v10b", folder="v10b", revision=None,
                                digest=hashlib.sha256(b"aiotvla v10b head.pt").hexdigest(),
                                precision="fp16", attn="sdpa", front_source=front,
                                facts={"horizon_s": 0.5})
    return ident if host is None else {**ident, "host": dict(host)}


# The flags the hil follow is flown with (deploy/orin/run_pas_perception.sh).
FOLLOW_FLAGS = ("--vla-front-source", "detector", "--vla-step-as-rate", "0.5",
                "--vla-rate-yaw", "rate", "--cadence", "pipelined",
                "--allow-unstamped-pipelined")


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


class StampedFrames(FakeVLAFrames):
    """FakeVLAFrames whose sent captures carry what vla_mission.VLAFrames
    logs for the observation summary: the capture's wall time and its front
    frame's sim stamp, the camera capturing every `interval_s` of the test's
    clock (so two requests inside one interval re-send one frame)."""

    def __init__(self, link, interval_s=0.1, **kw):
        super().__init__(link, **kw)
        self.interval_s = float(interval_s)

    def commit(self, capture, hint=None, seq=None, **extra):
        k = math.floor((capture.t_capture - 1000.0) / self.interval_s + 1e-9)
        super().commit(capture, hint, seq, t_capture=capture.t_capture,
                       stamp_ns=int(round(k * self.interval_s * 1e9)), **extra)


def _no_gpu_query(cmd, **kw):
    """The stand-in run's `run_cmd`: no nvidia-smi is asked (the GPU memory
    query records why)."""
    raise FileNotFoundError(f"{cmd[0]}: not run in tests")


def _vfly(respond, model=REAL_MODEL, *, mission="goto", goal=GOAL, policy=T.CAP_POLICY,
          shield="on", max_s="90", extra=(), link_cls=VLALink, grounder=None,
          frames_kw=None, recorder_cls=None, camera=None, live_dir=None, raises=None,
          client_kw=None, link_kw=None, run_cmd=None, frames_cls=None, obs=None):
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
        frames = (frames_cls or FakeVLAFrames)(link, **(frames_kw or {}))
        saved = os.environ.get("VLAGUARD_LIVE_DIR")
        if live_dir is not None:
            os.environ["VLAGUARD_LIVE_DIR"] = str(live_dir)
        rc = None
        try:
            rc = pn.run(args, link=link, camera=camera or T.FakeCamera(link),
                        grounder=grounder(clk) if callable(grounder) else grounder,
                        obs=(obs(clk) if callable(obs) else
                             obs if obs is not None else T.FakeObs()),
                        clock=clk, sleep=sleep, vla_client=client,
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
        # THE VLA FOLLOW's options (2026-11-06): the VLA's, and the follow's.
        (["--vla-step-as-rate", "0.5"], "--pilot vla"),
        (["--vla-front-source", "detector"], "--pilot vla"),
        (["--follow-memory-s", "6"], "--pilot vla"),
        (["--pilot", "vla", "--goal", "50,0", *srv, "--vla-step-as-rate", "0.5"],
         "--mission follow"),
        (["--pilot", "vla", "--mission", "approach", *srv, "--close-width-frac", "0.3"],
         "--mission follow"),
        (["--pilot", "vla", "--mission", "follow", *srv, "--vla-rate-yaw", "rate"],
         "--vla-step-as-rate"),
        (["--pilot", "vla", "--mission", "follow", *srv, "--vla-step-as-rate", "0"],
         "--vla-step-as-rate"),
        (["--pilot", "vla", "--mission", "follow", *srv, "--close-bottom-frac", "1.5"],
         "--close-bottom-frac"),
        (["--pilot", "vla", "--mission", "follow", *srv, "--follow-search-s", "nan"],
         "--follow-search-s"),
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
    # The VLA follow (2026-11-06; until then refused here): its object words
    # are the detector's phrase, its lost timeout vla_fly's 90 s.
    a = _args("--pilot", "vla", "--mission", "follow", *srv)
    pn.check_args(a)
    assert (a.mission, a.vla_object, a.lost_timeout_s) == ("follow", "a red car", 90.0)
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


# =========================================================================== #
# 11. THE VLA FOLLOW: AIOT-VLA on vla_fly's M3 parts (2026-11-06)
# =========================================================================== #

class CountingObs(T.FakeObs):
    """FakeObs that says how many FrontCamera frames it was handed, as
    demo/semantic_demo.SemanticObs.stats does: one every `interval_s` of the
    test's clock."""

    def __init__(self, clock, interval_s=0.1):
        super().__init__()
        self.clock, self.interval_s = clock, float(interval_s)

    def stats(self):
        return {"frames_received": int((self.clock.t - 1000.0) / self.interval_s),
                "frames_decoded": 0}


def _follow(respond=None, model=None, *, grounder=None, extra=(), **kw):
    """One VLA follow on the stand-ins (FOLLOW_FLAGS: the hil follow's), the
    server announcing AIOT-VLA reading the detector's camera."""
    model = _aiot_model() if model is None else model
    respond = respond or (lambda req: F.step_reply(req, 2.0, 0.0, 0.05))
    return _vfly(_real(respond, model), model, mission="follow",
                 grounder=grounder or (lambda clk: T.FakeDetections(clk)),
                 extra=FOLLOW_FLAGS + tuple(extra), **kw)


def test_a_vla_follow_flies_aiotvla_on_vla_flys_hint_and_tracker():
    """--pilot vla --mission follow (refused until 2026-11-06), as the hil
    follow flies it: AIOT-VLA's hello declares the detector's camera, the
    hint is vla_fly's M3 DetectorHint (the memory, the search, the close
    rule, each hint's kind), the step tracker is vla_fly.tracker_config on
    vla_fly's own parse of this flight (the follow executor, its smoothing,
    rates over AIOT-VLA's 0.5 s horizon, the yaw rate itself), and every
    record says what flew: the model by its own id, the object words as its
    grounding phrase, the server's host, the front camera, the follow's
    configuration, the observation reaching it, FrontCamera's interval."""
    import vla_fly as VF
    from guardrail.vla_link import TrackerConfig
    r = _follow(max_s="12", frames_cls=StampedFrames, frames_kw={"interval_s": 0.066},
                obs=lambda clk: CountingObs(clk, 0.066),
                extra=("--front-capture-interval-s", "0.066"))
    assert r.rc == 0, r.rc
    m = _json(r.run, "metrics.json")
    assert m["params"]["mission"] == "follow" and m["end_reason"] in ("timeout", "max_s reached")
    # The hint: vla_fly's M3 DetectorHint, its defaults read from vla_fly.
    ref = VF.resolve_args(VF.build_arg_parser().parse_args(
        ["--mission", "follow", "--step-as-rate", "0.5", "--rate-yaw", "rate",
         "--cruise-alt", "8"]))
    dh = m["vla"]["mission"]["detector_hint"]
    assert (dh["memory_s"], dh["search_s"]) == (ref.follow_memory_s, ref.follow_search_s) \
        == (6.0, 20.0), dh
    assert dh["close_rule"] == {"bottom_frac": ref.close_bottom_frac,
                                "width_frac": ref.close_width_frac}, dh["close_rule"]
    assert dh["hints_by_kind"].get("fresh", 0) > 10, dh["hints_by_kind"]
    flown = r.srv.requests[3:]
    assert len(flown) > 50 and all(q["hint_source"] == "detector" and q["object"] == "a red car"
                                   for q in flown)
    # The tracker: vla_fly's follow tracker, field by field.
    want = VF.tracker_config(ref, None, TrackerConfig)
    trk = m["vla"]["tracker"]
    for k in ("turn_while_moving", "anchor_at_answer", "land_as_arrival", "hold_altitude_m",
              "smooth_accel_mps2", "smooth_yaw_accel_rps2", "carry_s", "rate_horizon_s",
              "rate_yaw", "cruise_mps", "speed_cap_mps", "decision_timeout_s"):
        assert trk[k] == getattr(want, k), (k, trk[k], getattr(want, k))
    assert trk["turn_while_moving"] and trk["land_as_arrival"] and trk["hold_altitude_m"] == 8.0
    assert (trk["rate_horizon_s"], trk["rate_yaw"]) == (0.5, "rate")
    assert trk["smooth_accel_mps2"] == VF.DEFAULT_SMOOTH["accel"], trk
    fol = m["vla"]["follow"]
    assert fol["executor"] is True and fol["smooth"]["on"] is True and fol["lost_timeout_s"] == 90.0
    assert m["vla"]["mission"]["monitor"]["config"]["lost_timeout_s"] == 90.0
    # The rows: the follow's (as MissionLoop's), the rate commanded, the request made.
    rows = _rows(r.run)
    assert all("close_box" in x and "close_hold" in x and "pilot_raw" in x for x in rows)
    assert not any(x["close_box"] for x in rows) and m["vla"]["close_hold_ticks"] == 0
    assert any(x.get("hint_kind") == "fresh" for x in rows)
    assert any(x.get("vla_rate") is not None for x in rows)
    assert sum(1 for x in rows if x["vla_request"] == "sent") == len(flown)
    # Who answered: AIOT-VLA under its own id, its words its grounding phrase.
    assert m["pilot"]["kind"] == "vla" and m["pilot"]["model_id"] == "aiotvla:v10b", m["pilot"]
    mis = m["vla"]["mission"]
    assert mis["object_words_are_inert"] is False and "grounding" in mis["object_words_role"]
    # Where it ran (its hello's host): the run's vla_host, beside the Orin's.
    assert m["hil_evidence"]["vla_host"] == ORIN_HOST and m["vla"]["server_host"] == ORIN_HOST
    fs = m["vla"]["front_source"]
    assert fs["ok"] and fs["flight"] == "detector" and fs["server"] == "detector", fs
    # The CSP's mission is a follow; a follow declares no goal.
    mission = next(e for e in _events(r.run) if e["kind"] == "mission")
    assert mission["task"] == "follow a red car", mission["task"]
    assert m["reached_goal"] is None and "target_truth_dist_end_m" not in m
    # No start gate here (--start-when-seen 0): no share after acquisition.
    assert m["frac_within_30m_after_acquire"] is None and "did not" in m["frac_within_30m_basis"]
    # The observation, from the sent frames' own stamps; FrontCamera's interval.
    o = m["vla_observation"]
    assert o["requests_sent"] == len(flown) and o["front_source"] == "detector"
    assert o["camera"] == "FrontCamera" and o["requests_without_stamp"] == 0
    assert o["distinct_frames"] + o["duplicate_requests"] == o["requests_with_stamp"]
    assert o["hz_per_wall_s"] == round(o["distinct_frames"] / o["mission_wall_s"], 3)
    assert o["camera_delivery"]["capture_interval_s"] == 0.066
    assert 14.0 < o["camera_delivery"]["hz"] < 16.0, o["camera_delivery"]
    assert o["hint_gated"]["spans"] == 0 and o["hz_while_asking"] is not None
    assert m["sim_config"]["front_capture_interval_s"] == 0.066
    assert m["params"]["front_capture_interval_s"] == 0.066
    assert m["params"]["vla_front_source"] == "detector" and m["params"]["vla_rate_yaw"] == "rate"


def test_a_follow_that_never_asks_ends_subject_not_acquired_not_as_a_vla_failure():
    """vla_mission.MissionMonitor ends a follow with no request sent by its
    first-decision timeout as subject_not_acquired (review of d6e7123, P2-7)
    - but only when the caller passes the request count, and this rail did
    not: a follow whose box never came would have ended no_first_decision,
    a VLA failure (exit 3) for a VLA that was never asked. The count goes to
    the monitor now. Two ways nothing is asked: no fresh box (past the
    memory and the search), and a box so close that no step is asked for -
    while its horizontal motion is held. Each tick says why no request went
    out, and the observation summary counts the time hint-gated."""
    for det_kw, why in (({"fresh": False}, "no_fresh_box"), ({"w": 300.0}, "close_box")):
        r = _follow(grounder=lambda clk, kw=det_kw: T.FakeDetections(clk, **kw), max_s="20",
                    extra=("--first-decision-timeout-s", "2"))
        m = _json(r.run, "metrics.json")
        assert m["end_reason"] == "subject_not_acquired", (det_kw, m["end_reason"])
        assert r.rc == 0, (det_kw, r.rc)
        mon = m["vla"]["mission"]["monitor"]
        assert mon["requests_sent"] == 0 and mon["vla_failure"] is False, mon
        assert len(r.srv.requests) == 3, "only the warm-up was asked"
        assert not any("VLA failure" in w for w in _json(r.run, "kpi.json")["kpi_grade_reasons"])
        rows = _rows(r.run)
        assert rows and all(x["vla_request"] == "no_hint" and x["vla_no_hint_why"] == why
                            for x in rows), {(x["vla_request"], x.get("vla_no_hint_why"))
                                             for x in rows}
        o = m["vla_observation"]
        assert o["requests_sent"] == 0 and o["distinct_frames"] == 0 and o["hz_per_wall_s"] == 0.0
        assert o["hint_gated"]["spans"] == 1 and o["hint_gated"]["why_ticks"] == {why: len(rows)}
        assert o["hz_while_asking"] is None and o["asking_s"] == 0.0
        if why == "close_box":
            # The close hold, every tick: inside the tracker (it smooths).
            assert all(x["close_box"] and x["close_hold"] for x in rows)
            assert m["vla"]["close_hold_ticks"] == len(rows)
            assert m["pilot"]["summary"]["ticks_close_held"] == len(rows)


def test_the_close_hold_holds_the_horizontal_motion_and_each_tick_says_what_was_asked():
    """VLAPilot.step on a stand-in backend (MissionLoop's rule): a fresh
    close box holds the horizontal motion - inside the tracker when it
    smooths (act(hold_horizontal=True)), else by zeroing the tracker's vx /
    vy and keeping its climb and turn; a far box holds nothing. The row
    keeps the tracker's own action (pilot_raw), and what the backend did
    about a request (request_outcome, from its counts)."""
    from guardrail.models import Action4D, State
    from guardrail.vla_link import Tick

    class Backend:
        cadence = "pipelined"
        last_link_failure = None

        def __init__(self, smoothing):
            self.tracker = SimpleNamespace(current_record=None, smoothing=smoothing,
                                           cfg=SimpleNamespace(rate_horizon_s=None))
            self.counts = {"requests": 0, "no_hint": 0, "no_capture": 0, "request_errors": 0}
            self.calls, self.last_tick = [], None

        def act(self, st, hold_horizontal=False):
            self.calls.append(hold_horizontal)
            self.counts["no_hint"] += 1
            self.last_tick = Tick(Action4D(vx=1.0, vy=0.5, vz_up=0.2, yaw_rate=0.1),
                                  "vla_step", "translate", seq=1)
            return self.last_tick.action

        def suspend(self):
            pass

        def resume(self):
            pass

    class Det:
        def __init__(self, w):
            self.w = w

        def latest(self):
            return {"det": (400.0, 216.0, self.w, 12.0, 0.3, 768, 432, 0.5), "t_det": 9.95,
                    "seq": 1}
    st = State(x=0.0, y=0.0, up=8.0, yaw_deg=0.0)
    for smoothing in (False, True):
        for w, close in ((300.0, True), (20.0, False)):
            be = Backend(smoothing)
            dh = SimpleNamespace(fresh_s=1.0, last_kind="close" if close else None,
                                 record_kind=True, box_ignored=lambda t: False,
                                 last_box_age_s=None, last_yaw_correction_rad=None,
                                 last_detector_latency_s=None)
            p = pn.VLAPilot(be, VM.MissionMonitor(VM.MonitorConfig(mission="follow",
                                                                   lost_timeout_s=90.0), 0.0),
                            mission="follow", detections=Det(w), detector_hint=dh,
                            hint_source=SimpleNamespace(last={"kind": None}),
                            close_hold=VM.BoxClose(), vm=VM)
            raw, body, info = p.step(10.0, st)
            assert be.calls == [close and smoothing], (smoothing, w, be.calls)
            held = close and not smoothing
            assert (raw.vx, raw.vy) == ((0.0, 0.0) if held else (1.0, 0.5)), (smoothing, w, raw)
            assert (raw.vz_up, raw.yaw_rate) == (0.2, 0.1)
            end, row, hud = p.after(10.0, st, flies=True, touched=False, shield_on=True,
                                    t_mission=10.0)
            assert end is None and row["close_box"] is close and row["close_hold"] is close
            assert row["pilot_raw"]["vx"] == 1.0 and p.n_close_hold == int(close)
            assert row["vla_request"] == "no_hint"
            assert row["vla_no_hint_why"] == ("close_box" if close else "no_fresh_box")
            # The follow's request count reached the stop rules.
            assert p.monitor.requests_sent == 0
    # request_outcome reads the change in the backend's counts.
    c = {"requests": 1, "no_hint": 2, "no_capture": 0, "request_errors": 0}
    assert pn.request_outcome(c, {**c, "requests": 2}) == "sent"
    assert pn.request_outcome(c, {**c, "no_capture": 1}) == "no_capture"
    assert pn.request_outcome(c, {**c, "request_errors": 1}) == "error"
    assert pn.request_outcome(c, dict(c)) is None and pn.request_outcome(None, c) is None


def test_the_detector_front_source_feeds_the_request_from_the_detectors_camera():
    """--vla-front-source detector (demo/vla_fly.py's): the ONE FrontCamera
    subscription feeds the detector and the VLA frame store (a pointer each);
    VLAFront is not subscribed and not required, VLADown still is - in the
    robot config and, attached, among the live topics. The default source
    keeps VLAFront as the request's front frame."""
    class Obs(T.FakeObs):
        def __init__(self):
            super().__init__()
            self.front = []

        def put_front(self, m):
            self.front.append(m)

        def put_depth(self, m):
            pass

    class Store:
        def __init__(self):
            self.front, self.down = [], []

        def put_front(self, m):
            self.front.append(m)

        def put_down(self, m):
            self.down.append(m)
    cfg = rail.write_sim_configs(T._tmp(), rail.plan_network("mirrored"), vla_cameras=True)
    no_vlafront = {"FrontCamera": {"scene_camera": "fs", "depth_camera": "fd"},
                   "VLADown": {"scene_camera": "vd"}}
    pas, h = _cam_pas(no_vlafront)
    obs, store = Obs(), Store()
    cam = pn.ProjectAirSimCamera(obs, cfg, projectairsim=pas, vla_frames=store,
                                 vla_front_source="detector")
    cam.start()
    assert "fs" in h["c"].subs and "vd" in h["c"].subs and "vf" not in h["c"].subs
    h["c"].cbs["fs"](None, {"m": "front"})
    h["c"].cbs["vd"](None, {"m": "down"})
    assert obs.front == [{"m": "front"}] and store.front == [{"m": "front"}]
    assert store.down == [{"m": "down"}]
    assert (cam.scene_record["vla_front_source"], cam.scene_record["vla_front_camera"]) == \
        ("detector", "FrontCamera")
    # The default source needs VLAFront: this robot is refused ...
    cam = pn.ProjectAirSimCamera(Obs(), cfg, projectairsim=_cam_pas(no_vlafront)[0],
                                 vla_frames=Store())
    try:
        cam.start()
    except RuntimeError as exc:
        assert "VLAFront" in str(exc), exc
    else:
        raise AssertionError("a vla front source flew without VLAFront")
    # ... and with it, FrontCamera feeds the detector alone.
    full = {**no_vlafront, "VLAFront": {"scene_camera": "vf"}}
    pas, h = _cam_pas(full)
    obs, store = Obs(), Store()
    cam = pn.ProjectAirSimCamera(obs, cfg, projectairsim=pas, vla_frames=store)
    cam.start()
    h["c"].cbs["fs"](None, {"m": "front"})
    h["c"].cbs["vf"](None, {"m": "vlafront"})
    assert obs.front == [{"m": "front"}] and store.front == [{"m": "vlafront"}]
    # Attached: the cameras this source needs must be live.
    acfg = rail.write_attach_configs(T._tmp(), vla_cameras=True)
    base = "/Sim/SceneCityLifeArduPilot/robots/Drone1/sensors/"
    for live, src, missing in (
            ([base + "FrontCamera/scene_camera", base + "VLADown/scene_camera"], "detector", None),
            ([base + "FrontCamera/scene_camera", base + "VLADown/scene_camera"], "vla", "VLAFront"),
            ([base + "FrontCamera/scene_camera", base + "VLAFront/scene_camera"], "detector",
             "VLADown")):
        cam = pn.ProjectAirSimCamera(Obs(), acfg, projectairsim=_cam_pas(full, topics=live)[0],
                                     attach=True, vla_frames=Store(), vla_front_source=src)
        try:
            cam.start()
        except RuntimeError as exc:
            assert missing is not None and missing in str(exc), (src, exc)
        else:
            assert missing is None, f"{src}: attached to a scene without {missing}"
    try:
        pn.ProjectAirSimCamera(Obs(), cfg, vla_front_source="chase")
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown front source was accepted")


def test_the_front_camera_is_checked_against_the_one_the_server_declares():
    """vla_mission.front_source_check with the flight's own source (it was
    hard-coded to VLAFront, so AIOT-VLA from v3 on - which reads the
    detector's camera - was always refused): the camera the server's hello
    declares must be the one this flight sends, or nothing is armed."""
    no_source = tuple(a for a in FOLLOW_FLAGS if a not in ("--vla-front-source", "detector"))
    r = _vfly(_real(lambda req: F.step_reply(req, 2.0, 0.0, 0.0), _aiot_model()),
              _aiot_model(), mission="follow", raises=SystemExit, extra=no_source,
              grounder=lambda clk: T.FakeDetections(clk))
    assert "FrontCamera" in str(r.exc) and "--vla-front-source vla" in str(r.exc), r.exc
    assert r.link.snap.armed is not True and not r.srv.requests and not r.link.sent
    # AerialVLA reads VLAFront: the detector's frame is refused for it.
    r = _vfly(_real(lambda req: F.step_reply(req, 2.0, 0.0, 0.0)), mission="approach",
              raises=SystemExit, grounder=lambda clk: T.FakeDetections(clk),
              extra=("--object", "a red car", "--vla-front-source", "detector"))
    assert "VLAFront" in str(r.exc) and r.link.snap.armed is not True, r.exc


def test_the_servers_own_host_decides_whether_a_vla_run_can_be_hil():
    """The VLA is a separate process (the server), so the hil evidence
    carries its host, as the ROS rail carries a VLA node's: absent for the
    detector pilot (in-process), the hello's `host` for a VLA run. An Orin's
    keeps hil; the desktop's (a server reached through a tunnel) or none
    (a hello that does not say) never lets the run call itself hil."""
    class OrinMavrosLink:
        name = "mavros"

        def __init__(self):
            self.hil_evidence = {
                "ros_distro": "jazzy", "mavros_node": "/mavros", "fcu_connected": True,
                "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)", "autopilot_kind": "sitl",
                "network_link": {"fcu_url": "udp://:14555@10.42.0.1:14555", "peer": "10.42.0.1",
                                 "peer_is_loopback": False, "iface": "end0",
                                 "link_kind": "ethernet"},
                "local_processes": {"scanned": "/proc", "local_sitl": [], "local_mavros": True,
                                    "local_router": []},
                **ORIN_HOST}

        def evidence(self):
            return {"link": "mavros", "heartbeat": True, "is_ardupilot": True,
                    "heartbeat_seen": True, "ardupilot_seen": True,
                    "topology_evidence": dict(self.hil_evidence)}
    # 192.0.2.10 (TEST-NET-1): a simulator on another machine, as the Orin sees the desktop.
    args = pn.build_parser().parse_args(["--link", "mavros", "--scene-mode", "attach",
                                         "--sim-host", "192.0.2.10"])
    sim_cfg = {"controller_type": "ardupilot-api"}
    link = OrinMavrosLink()
    ev, _ = pn.rail_topology_evidence(args, sim_cfg, link)
    assert "vla_host" not in ev["topology_evidence"] and topo.classify(ev)["grant"] == "hil"
    for vh, grant, words in ((dict(ORIN_HOST), "hil", None),
                             (dict(DESKTOP_HOST), None, "not on a Jetson Orin"),
                             (None, None, "did not report")):
        ev, _ = pn.rail_topology_evidence(args, sim_cfg, link, vla_host=vh)
        assert ev["topology_evidence"]["vla_host"] == vh
        c = topo.classify(ev)
        assert c["grant"] == grant, (vh, c)
        if words:
            assert any(words in x for x in c["reasons"]), (vh, c["reasons"])
    # The hello's host, and the MAVROS link's own record of it.
    assert pn.vla_host_of({"host": dict(ORIN_HOST)}) == ORIN_HOST
    assert pn.vla_host_of({"host": {}}) is None and pn.vla_host_of({"id": "x"}) is None
    assert pn.vla_host_of(None) is None and pn.vla_host_of({"host": "orin"}) is None
    pn.note_vla_host(link, dict(DESKTOP_HOST))
    assert link.hil_evidence["vla_host"] == DESKTOP_HOST
    plain = SimpleNamespace()
    pn.note_vla_host(plain, dict(ORIN_HOST))
    assert not hasattr(plain, "hil_evidence")
    # A run whose server says nothing of its host records None, which hil
    # refuses; AerialVLA's words stay inert.
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)), max_s="2")
    m = _json(r.run, "metrics.json")
    assert "vla_host" in m["hil_evidence"] and m["hil_evidence"]["vla_host"] is None
    assert m["vla"]["mission"]["object_words_are_inert"] is True


def test_the_object_words_are_recorded_as_what_they_are_to_the_model():
    """AerialVLA's object slot is inert; AIOT-VLA grounds the words in the
    image (its backbone's phrase); DroneCATS builds its command from them.
    Decided by the server's id alone, as the panel's model label is."""
    assert VM.object_words_record(REAL_MODEL)["object_words_are_inert"] is True
    for ident in (_aiot_model(), {**_aiot_model(), "id": "ntutvla/v2@1a2b3c4d"}):
        w = VM.object_words_record(ident)
        assert w["object_words_are_inert"] is False and "grounding" in w["object_words_role"], w
    dc = SF.foreign_identity("dronecats", name="DroneCATS-Cosmos3-Edge-2B",
                             folder="DroneCATS-Cosmos3-Edge-2B", revision=None,
                             digest=hashlib.sha256(b"dc").hexdigest(), precision="bf16",
                             attn="sdpa", front_source="detector")
    w = VM.object_words_record(dc)
    assert w["object_words_are_inert"] is False and "command" in w["object_words_role"], w
    for ident in (dict(F.MODEL), {"id": "fake:scripted"}, None, {}):
        assert VM.object_words_record(ident)["object_words_are_inert"] is True, ident


def test_the_observation_rate_counts_distinct_frames_sent_not_requests():
    """vla_observation on a hand-made record: warm-ups left out, a frame
    re-sent counted as a duplicate, a request with no stamp apart; the rates
    per wall second, while the pilot was asked (not held, not hint-gated) and
    per sim second; the gaps between distinct frames; the hint-gated spans
    with why; the camera's own delivery."""
    caps = [{"seq": 1, "warmup": True, "t_capture": 990.0, "stamp_ns": 90 * 10**9},
            {"seq": 4, "t_capture": 1000.05, "stamp_ns": 100_000_000_000},
            {"seq": 5, "t_capture": 1000.15, "stamp_ns": 100_100_000_000},
            {"seq": 6, "t_capture": 1000.20, "stamp_ns": 100_100_000_000},
            {"seq": 7, "t_capture": 1000.45, "stamp_ns": 100_400_000_000},
            {"seq": 8, "t_capture": 1000.55, "stamp_ns": None},
            {"seq": 9, "t_capture": 1001.05, "stamp_ns": 101_000_000_000}]
    plan = [("sent", False, None), ("sent", False, None), (None, False, None),
            ("sent", False, None), ("no_hint", False, "close_box"),
            ("no_hint", False, "close_box"), (None, False, None), ("sent", False, None),
            (None, True, None), (None, False, None), ("no_hint", False, "no_fresh_box"),
            ("no_hint", True, "no_fresh_box"), ("no_hint", False, "no_fresh_box"),
            ("sent", False, None)]
    rows = []
    for i, (req, held, why) in enumerate(plan):
        row = {"t": round(0.1 * i, 3), "vla_request": req, "pilot_held": held}
        if why:
            row["vla_no_hint_why"] = why
        rows.append(row)
    o = pn.vla_observation(caps, rows, t0=1000.0, span_s=1.4, front_source="detector",
                           camera="FrontCamera", camera_frames=21, capture_interval_s=0.066)
    assert (o["requests_sent"], o["requests_with_stamp"], o["requests_without_stamp"]) == (6, 5, 1)
    assert (o["distinct_frames"], o["duplicate_requests"], o["duplicate_share"]) == (4, 1, 0.2)
    assert o["hz_per_wall_s"] == round(4 / 1.4, 3)
    assert (o["asking_s"], o["held_s"], o["hint_gated"]["s"], o["unrowed_s"]) == \
        (0.7, 0.2, 0.5, 0.0), o
    assert o["hz_while_asking"] == round(4 / 0.7, 3)
    g = o["hint_gated"]
    assert (g["spans"], g["longest_s"]) == (3, 0.3), g
    assert g["why_ticks"] == {"close_box": 2, "no_fresh_box": 2}, g
    assert [s["t"] for s in g["first"]] == [0.4, 1.0, 1.2] and g["first"][0]["ticks"] == 3
    assert (o["sim_span_s"], o["wall_span_s"], o["sim_per_wall"], o["hz_per_sim_s"]) == \
        (1.0, 1.0, 1.0, 3.0), o
    assert o["gap_sim_s"] == {"n": 3, "p50": 0.3, "p95": 0.57, "max": 0.6}, o["gap_sim_s"]
    assert o["gap_wall_s"]["n"] == 3 and o["gap_wall_s"]["max"] == 0.6
    assert o["below_grant"] == {"per_wall_s": True, "while_asking": True}
    assert o["camera_delivery"]["hz"] == 15.0 and o["camera_delivery"]["frames"] == 21
    assert "not the request rate" in o["basis"] and o["grant_obs_hz"] == pn.GRANT_OBS_HZ
    # Nothing sent, nothing measured: zeros and None, never an error.
    o = pn.vla_observation([], [], t0=0.0, span_s=None, front_source="vla", camera="VLAFront")
    assert o["distinct_frames"] == 0 and o["hz_per_wall_s"] is None
    assert o["gap_sim_s"] == {"n": 0, "p50": None, "p95": None, "max": None}
    assert o["below_grant"] == {"per_wall_s": None, "while_asking": None}


# =========================================================================== #
# 12. Camera to setpoint (WP3-13)
# =========================================================================== #

def test_the_camera_to_setpoint_fields_are_differences_on_one_clock():
    """The row fields, on hand-made times: the publication against the
    tick's start and the mission's, the frame's age at it, the first
    setpoint of a decision only, and nothing where nothing flew."""
    assert pn.publish_latency_fields(1000.20, None, 1000.0) == {"t_pub": None, "tick_ms": None}
    assert pn.publish_latency_fields(1000.20, 1000.2125, 1000.0) == \
        {"t_pub": 0.2125, "tick_ms": 12.5}
    f = pn.camera_age_fields(1000.2125, 999.95, flies=True, fresh=True)
    assert f == {"cam_age_ms": 262.5, "cam_to_setpoint_ms": 262.5}, f
    f = pn.camera_age_fields(1000.3125, 999.95, flies=True, fresh=False)
    assert f == {"cam_age_ms": 362.5, "cam_to_setpoint_ms": None}, f
    none = {"cam_age_ms": None, "cam_to_setpoint_ms": None}
    assert pn.camera_age_fields(1000.3, 999.9, flies=False, fresh=True) == none   # held
    assert pn.camera_age_fields(None, 999.9, flies=True, fresh=True) == none      # not sent
    assert pn.camera_age_fields(1000.3, None, flies=True, fresh=True) == none     # no decision
    assert pn.camera_age_fields(1000.3, float("nan"), flies=True, fresh=True) == none
    rows = ([{"author": "vla_step", "cam_age_ms": float(v), "cam_to_setpoint_ms": None,
              "tick_ms": 2.0} for v in range(1, 101)]
            + [{"author": "hover", "cam_age_ms": None, "tick_ms": None}])
    rows[0]["cam_to_setpoint_ms"] = 150.0
    s = pn.camera_to_setpoint_summary(rows, t_capture_basis={"arrival": 3})
    a = s["cam_age_ms"]
    assert (a["n"], a["p50"], a["p95"], a["p99"], a["max"], a["over_budget"]) == \
        (100, 50.0, 95.0, 99.0, 100.0, 0), a
    assert s["cam_to_setpoint_ms"]["n"] == 1 and s["cam_to_setpoint_ms"]["over_budget"] == 1
    assert s["tick_ms"]["n"] == 100 and s["by_author"] == {"vla_step": 100}
    assert s["t_capture_basis"] == {"arrival": 3} and "ARRIVAL" in s["capture_is"]
    empty = pn.camera_to_setpoint_summary([])["cam_to_setpoint_ms"]
    assert empty == {"n": 0, "p50": None, "p95": None, "p99": None, "max": None,
                     "over_budget": 0}, "an empty measure is None, never 0 ms"


def test_a_vla_flight_logs_its_publication_and_the_frame_behind_each_setpoint():
    """A VLA GOTO on the stand-ins (the test's clock): every row says when
    its setpoint went out; each tick a decision's action flew carries the
    age of that decision's frame at the publication, which is the
    publication minus the decision's t_capture in vla_steps.jsonl; the first
    setpoint of each decision carries cam_to_setpoint_ms and no later one
    does; the metrics summarise exactly the rows."""
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)))
    assert r.rc == 0
    m = _json(r.run, "metrics.json")
    rows = _rows(r.run)
    t0 = m["vla_observation"]["t0_wall"]
    assert all(x["t_pub"] is not None and x["tick_ms"] is not None for x in rows)
    # The test's clock stands still within a tick: published at the tick's start.
    assert all(abs(x["t_pub"] - x["t"]) < 1e-3 and x["tick_ms"] == 0.0 for x in rows)
    cap = {s["seq"]: s["t_capture"] for s in _steps(r.run)}
    aged = [x for x in rows if x["cam_age_ms"] is not None]
    assert len(aged) > len(rows) / 2, (len(aged), len(rows))
    for x in aged:
        assert x["vla_seq"] in cap and not x["pilot_held"], x
        want = (x["t_pub"] + t0 - cap[x["vla_seq"]]) * 1000.0
        assert abs(x["cam_age_ms"] - want) < 2.0, (x["cam_age_ms"], want)
        assert x["cam_age_ms"] > 0.0
    firsts = [x for x in rows if x["cam_to_setpoint_ms"] is not None]
    seqs = [x["vla_seq"] for x in firsts]
    assert seqs == sorted(set(seqs)) and set(seqs) == {x["vla_seq"] for x in aged}, seqs
    for x in firsts:
        assert x["cam_to_setpoint_ms"] == x["cam_age_ms"]
        assert x is next(y for y in rows if y["vla_seq"] == x["vla_seq"]
                         and y["cam_age_ms"] is not None)
    # The answer comes back within one tick of the test's clock: its first
    # setpoint is the next tick's.
    assert {x["cam_to_setpoint_ms"] for x in firsts} <= {100.0}, \
        sorted({x["cam_to_setpoint_ms"] for x in firsts})
    s = m["camera_to_setpoint"]
    assert s["cam_age_ms"]["n"] == len(aged) and s["cam_to_setpoint_ms"]["n"] == len(firsts)
    assert s["cam_age_ms"]["max"] == max(x["cam_age_ms"] for x in aged)
    assert s["tick_ms"]["n"] == len(rows) and s["budget_ms"] == 100.0
    assert "one clock" in s["clock"] and "ARRIVAL" in s["capture_is"]
    assert sum(s["by_author"].values()) == len(aged)


def test_a_held_tick_carries_no_camera_age():
    """The FSM's hold (LOITER_POLICY: Loiter from the first tick): the
    rail's setpoint still goes out (t_pub), but no decision's action flew,
    so no camera age is claimed for it."""
    r = _vfly(_real(F.goto_responder(GOAL, 8.0)), policy=T.LOITER_POLICY, max_s="4")
    rows = _rows(r.run)
    held = [x for x in rows if x["pilot_held"]]
    assert held, "the fixture must hold"
    assert all(x["cam_age_ms"] is None and x["cam_to_setpoint_ms"] is None for x in held)
    assert all(x["t_pub"] is not None for x in rows if x["flown_action"] is not None)
    s = _json(r.run, "metrics.json")["camera_to_setpoint"]
    assert s["cam_age_ms"]["n"] == sum(1 for x in rows if x["cam_age_ms"] is not None)


def test_the_front_capture_interval_renders_only_when_asked_and_is_recorded():
    """FrontCamera captures every 0.1 s in the template; a run may render
    another interval (--front-capture-interval-s; -FrontCaptureIntervalS on
    the launcher's scene and node steps), which changes that sensor alone
    and is recorded with where it came from. The default renders the
    template's. Out-of-range values are refused before anything starts."""
    before = rail.ROBOT_TEMPLATE.read_bytes()
    tmpl = rail.load_jsonc(rail.ROBOT_TEMPLATE)
    want = {s["id"]: s.get("capture-interval") for s in tmpl["sensors"]}
    assert want["FrontCamera"] == 0.1
    for writer in (lambda d, **k: rail.write_sim_configs(d, rail.plan_network("mirrored"), **k),
                   lambda d, **k: rail.write_attach_configs(d, **k)):
        cfg = writer(T._tmp(), vla_cameras=True)
        robot = rail.load_jsonc(Path(cfg["dir"]) / cfg["robot"])
        assert {s["id"]: s.get("capture-interval") for s in robot["sensors"]} == want
        assert (cfg["front_capture_interval_s"], cfg["front_capture_interval_source"]) == \
            (0.1, "template")
        cfg = writer(T._tmp(), vla_cameras=True, front_capture_interval_s=0.066)
        robot = rail.load_jsonc(Path(cfg["dir"]) / cfg["robot"])
        got = {s["id"]: s.get("capture-interval") for s in robot["sensors"]}
        assert got == {**want, "FrontCamera": 0.066}, got
        assert cfg["front_capture_interval_s"] == 0.066
        assert cfg["front_capture_interval_source"].startswith("--front-capture-interval-s")
    assert "desktop's scene step" in rail.write_attach_configs(
        T._tmp(), front_capture_interval_s=0.066)["front_capture_interval_source"]
    assert rail.ROBOT_TEMPLATE.read_bytes() == before, "the template was modified"
    assert rail.capture_interval(tmpl, "FrontCamera") == 0.1
    assert rail.capture_interval(tmpl, "NoSuchCamera") is None
    for bad in (0, -0.1, 1.5, float("nan"), float("inf"), "abc"):
        try:
            rail.set_capture_interval(rail.load_jsonc(rail.ROBOT_TEMPLATE), "FrontCamera", bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"capture interval {bad!r} was accepted")
    try:
        rail.set_capture_interval({"sensors": []}, "FrontCamera", 0.066)
    except ValueError as exc:
        assert "FrontCamera" in str(exc)
    else:
        raise AssertionError("a robot without FrontCamera was given an interval")
    # The node: refused out of range, passed to whichever configs it writes.
    for bad in ("0", "2", "nan", "-0.05"):
        try:
            pn.check_args(_args("--front-capture-interval-s", bad))
        except SystemExit as exc:
            assert "--front-capture-interval-s" in str(exc), exc
        else:
            raise AssertionError(f"--front-capture-interval-s {bad} was accepted")
    try:
        pn.check_args(_args("--camera", "ros2", "--link", "mavros",
                            "--front-capture-interval-s", "0.066"))
    except SystemExit as exc:
        assert "ros2" in str(exc), exc
    else:
        raise AssertionError("a ROS 2 camera was given a FrontCamera interval")
    out = T._tmp()
    for argv in (["--scene-only", "--vla-cameras"], ["--scene-mode", "attach"]):
        a = _args(*argv, "--front-capture-interval-s", "0.066")
        pn.check_args(a)
        cfg, _ = pn.sim_configs(a, out / argv[0].strip("-"))
        assert cfg["front_capture_interval_s"] == 0.066, (argv, cfg)
        a = _args(*argv)
        pn.check_args(a)
        cfg, _ = pn.sim_configs(a, out / ("d" + argv[0].strip("-")))
        assert cfg["front_capture_interval_s"] == 0.1, (argv, cfg)
    # Project AirSim's own loader takes the rendered interval (where installed).
    try:
        from projectairsim.utils import load_scene_config_as_dict
    except Exception:                                             # noqa: BLE001
        print("      (projectairsim is not installed here: its loader is not run)")
    else:
        cfg = rail.write_sim_configs(T._tmp(), rail.plan_network("mirrored"),
                                     front_capture_interval_s=0.066)
        data, _ = load_scene_config_as_dict(cfg["scene"], cfg["dir"])
        sensors = {s["id"]: s for s in data["actors"][0]["robot-config"]["sensors"]}
        assert sensors["FrontCamera"]["capture-interval"] == 0.066
    # The launcher's scene step can ask for it.
    ps1 = (ROOT / "scripts" / "run_pas_ardupilot.ps1").read_text(encoding="utf-8")
    assert "[double]$FrontCaptureIntervalS" in ps1 and '"--front-capture-interval-s"' in ps1


def test_the_wait_for_the_next_tick_pumps_the_feed_and_keeps_the_deadline():
    """--vla-feed stream (pace_feeding, 2026-11-15): the loop's wait for its
    absolute deadline is cut into slices with a pump() before each, none in
    the last FEED_GUARD_S, and it ends on the deadline like the plain sleep
    it replaces; a pump that overruns the deadline is not followed by a
    sleep."""
    t = {"now": 0.0}
    calls: list[float] = []

    def clock():
        return t["now"]

    def sleep(s):
        assert s > 0.0
        t["now"] += s

    def pump():
        calls.append(t["now"])
        t["now"] += 0.001
    n = pn.pace_feeding(0.05, pump, clock, sleep)
    assert n == len(calls) and n >= 5, calls
    assert abs(t["now"] - 0.05) < 1e-6, t
    assert all(c < 0.05 - pn.FEED_GUARD_S + 1e-9 for c in calls), calls
    assert pn.pace_feeding(0.05, pump, clock, sleep) == 0 and len(calls) == n   # nothing left

    def slow():
        calls.append(t["now"])
        t["now"] += 0.2
    slept: list[float] = []
    assert pn.pace_feeding(0.15, slow, clock, slept.append) == 1 and not slept


def test_a_request_the_feed_sent_between_ticks_is_on_the_next_row():
    """VLAPilot.step reads the backend's counts since its LAST step, so a
    request pump() sent between two ticks reaches the next row's
    `vla_request` (the observation-rate summary counts the seconds the model
    was asked from it); pump() is the backend's, and a backend without one
    is left alone."""
    from guardrail.models import Action4D, State
    from guardrail.vla_link import Tick

    class Backend:
        cadence = "pipelined"
        last_link_failure = None

        def __init__(self):
            self.tracker = SimpleNamespace(current_record=None, smoothing=False,
                                           cfg=SimpleNamespace(rate_horizon_s=None))
            self.counts = {"requests": 0, "no_hint": 0, "no_capture": 0, "request_errors": 0}
            self.last_tick = None

        def act(self, st, hold_horizontal=False):
            self.counts["no_hint"] += 1                 # the tick itself sends nothing
            self.last_tick = Tick(Action4D(vx=1.0, vy=0.0, vz_up=0.0, yaw_rate=0.0),
                                  "vla_step", "translate", seq=1)
            return self.last_tick.action

        def pump(self):
            self.counts["requests"] += 1
            return True

        def suspend(self):
            pass

        def resume(self):
            pass

    class NoPump(Backend):
        pump = None
    st = State(x=0.0, y=0.0, up=8.0, yaw_deg=0.0)

    def pilot(be):
        return pn.VLAPilot(be, VM.MissionMonitor(VM.MonitorConfig(mission="goto"), 0.0),
                           mission="goto", goal_xy=(50.0, 0.0), vm=VM)
    p = pilot(Backend())
    p.step(10.0, st)
    _, row, _ = p.after(10.0, st, flies=True, touched=False, shield_on=True, t_mission=10.0)
    assert row["vla_request"] == "no_hint", row["vla_request"]
    assert p.pump() is True                             # between the ticks
    p.step(10.1, st)
    _, row, _ = p.after(10.1, st, flies=True, touched=False, shield_on=True, t_mission=10.1)
    assert row["vla_request"] == "sent", row["vla_request"]
    p.step(10.2, st)                                    # no pump in between: the tick's own word
    _, row, _ = p.after(10.2, st, flies=True, touched=False, shield_on=True, t_mission=10.2)
    assert row["vla_request"] == "no_hint", row["vla_request"]
    assert pilot(NoPump()).pump() is False


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
