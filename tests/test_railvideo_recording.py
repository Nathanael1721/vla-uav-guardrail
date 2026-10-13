"""The perception rail's demo video (rail-video unit, part 1): --save-view on
demo/pas_ardupilot/perception_node.py, the policy HUD it hands the recorder,
and the launcher's -SaveView.

    python tests/test_railvideo_recording.py      (no pytest needed)

Every flight here is the stand-in of tests/test_pas_ardupilot.py (a point-mass
ArduPilot behind a fake link, a fake detector, a fake camera that serves
768x432 / 960x540 noise frames, the FrontCamera and Chase sizes of
robot_citylife_ardupilot.jsonc): nothing was flown in Project AirSim or SITL.
Two kinds of flight:

  * fake clock (fast): the HUD dicts handed to a spy recorder - which banner
    and which FSM state each tick carried;
  * REAL clock (about 3-4 s each): the real FrameRecorder thread writing real
    JPEGs while the loop flies, so the loop rate with and without recording
    is measured, not assumed.

Shown failing on a pristine HEAD 5cf7a08 copy (git archive) with this file
copied in: the node's parser refuses --save-view (argparse exit 2, counted as
a failure by the runner below), rail_banner / hud_decision / RailHud do not
exist, ProjectAirSimCamera takes no `chase` and serves no frames, run() takes
no recorder_cls, and the launcher has no -SaveView or Set-ViewJunction (14
errors and 1 failure; the CityLife-map timing SKIPs there, the map being
gitignored, and errors on build_policy_hud once the map is copied in).

Needs PIL (and numpy) for the recorder; a host without PIL prints SKIP for the
tests that write frames.
"""
from __future__ import annotations

import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "demo", PAS, ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import test_pas_ardupilot as T                                       # noqa: E402
import autopilot as ap                                               # noqa: E402
import perception_node as pn                                         # noqa: E402
import rail                                                          # noqa: E402

SKIP = "SKIP"
ANCHOR = (35.6895, 139.6917, 40.0)        # the CityLife scene's home-geo-point


def _have_pil() -> bool:
    try:
        import PIL                                                   # noqa: F401
        return True
    except ImportError:
        return False


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="pas_rv_"))


class RealClock:
    """The wall clock, in the shape the stand-in's link and detector read
    (`.t`) and run() calls (`clock()`, `sleep()`)."""

    @property
    def t(self) -> float:
        return time.time()

    def __call__(self) -> float:
        return time.time()

    def sleep(self, dt: float) -> None:
        time.sleep(max(0.0, dt))


class FrameCamera(T.FakeCamera):
    """The stand-in camera, serving frames at the rail's camera sizes, built
    from an array on every call as ProjectAirSimCamera's decode does."""

    def __init__(self, link, log=None):
        super().__init__(link, log)
        import numpy as np
        rng = np.random.default_rng(7)
        self.front = rng.integers(0, 255, (432, 768, 3), dtype=np.uint8)
        self.chase = rng.integers(0, 255, (540, 960, 3), dtype=np.uint8)
        self.served = {"front": 0, "chase": 0}

    def get_front_native(self):
        from PIL import Image
        self.served["front"] += 1
        return Image.fromarray(self.front)

    def get_chase_native(self):
        from PIL import Image
        self.served["chase"] += 1
        return Image.fromarray(self.chase)


class SpyRecorder:
    """FrameRecorder's interface with no thread: keeps every HUD it is handed
    (fake-clock flights finish in milliseconds, before a real recorder's
    first slot)."""
    made: list = []

    def __init__(self, obs, out_dir, annotate_fn, hz=20.0, height=720, quality=88,
                 live_view=False, live_height=480):
        self.obs, self.out_dir, self.annotate = obs, Path(out_dir), annotate_fn
        self.hz, self.height, self.live_view = hz, height, live_view
        self.huds, self.dets = [], []
        self.started = self.stopped = False
        SpyRecorder.made.append(self)

    def start(self):
        self.started = True

    def set_hud(self, hud, det):
        self.huds.append(hud)
        self.dets.append(det)

    def stop(self):
        self.stopped = True

    def join(self, timeout=None):
        pass

    def is_alive(self):
        return False

    def write_sidecar(self, path):
        s = {"frames": 0, "target_hz": self.hz, "achieved_hz": None, "frame_t": []}
        Path(path).write_text(json.dumps(s), encoding="utf-8")
        return s


class FailingRecorder(SpyRecorder):
    def set_hud(self, hud, det):
        raise RuntimeError("disk full")


def _fly(policy: str = T.CAP_POLICY, *, shield="on", max_s="3", extra=(),
         save=True, real=False, camera_cls=None, recorder_cls=None, link_kw=None,
         link_cls=None, out=None):
    out = out or _tmp()
    pol = out / "policy.yaml"
    pol.write_text(policy, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", max_s,
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", T.WIN, "--wsl-ip", T.WSL, "--shield", shield,
        "--speed-max", "4", "--dev-yaml", *(["--save-view"] if save else []), *extra])
    clk = RealClock() if real else T.Clock()
    anchor = ap.FrameAnchor(*ANCHOR, "scene.home-geo-point")
    link = (link_cls or T.FakeLink)(clk, anchor, **(link_kw or {}))
    cam = (camera_cls or FrameCamera)(link)
    rc = pn.run(args, link=link, camera=cam, grounder=T.FakeDetections(clk),
                obs=T.FakeObs(), clock=clk, sleep=clk.sleep, recorder_cls=recorder_cls)
    run = out / "run"
    return rc, run, json.loads((run / "metrics.json").read_text()), link, cam


def _jpgs(d: Path) -> list:
    return sorted(d.glob("*.jpg")) if d.is_dir() else []


# =========================================================================== #
# The banner and the panel's decision (pure)
# =========================================================================== #

OK = {"level": "ok", "text": "GUARDRAIL ACTIVE - all rules satisfied"}
ACT = {"level": "act", "text": "SHIELD CORRECTED COMMAND - speed capped (kin-caps)"}


def test_the_banner_names_the_fsm_state_and_the_hold_mode_it_flies():
    """Brake streams the Shield's own action on most ticks (guardrail/fsm.py:
    'the Shield intervening inside GUIDED'), so its banner keeps the rule
    event and says the state; a zero setpoint, Loiter, RTL and Land replace
    it. Loiter is shown with the ArduCopter mode it is flown as."""
    F = pn.FSMState
    b, k = pn.rail_banner(ACT, shield_on=True, fsm_state=F.NORMAL)
    assert (b, k) == (ACT, "policy")
    b, k = pn.rail_banner(ACT, shield_on=True, fsm_state=F.BRAKE, setpoint="pass")
    assert k == "fsm_brake" and b["text"] == "FSM Brake | " + ACT["text"] \
        and b["level"] == "act", b
    b, k = pn.rail_banner(ACT, shield_on=True, fsm_state=F.BRAKE, setpoint="brake")
    assert k == "fsm" and b["level"] == "brake" and "ZERO SETPOINT" in b["text"], b
    b, _ = pn.rail_banner(OK, shield_on=True, fsm_state=F.LOITER, hold_ap_mode="BRAKE")
    assert b["text"].startswith("FSM Loiter(BRAKE)") and b["level"] == "hold", b
    b, _ = pn.rail_banner(OK, shield_on=True, fsm_state=F.LOITER, hold_ap_mode="LOITER")
    assert b["text"].startswith("FSM Loiter(LOITER)"), b
    b, _ = pn.rail_banner(OK, shield_on=True, fsm_state=F.RTL)
    assert b["text"].startswith("FSM RTL") and b["level"] == "brake", b
    b, _ = pn.rail_banner(OK, shield_on=True, fsm_state=F.LAND)
    assert b["text"].startswith("FSM Land"), b
    # Every level is one policy_hud can colour.
    from policy_hud import COLOURS
    for st in F:
        for sp in ("pass", "brake", "none"):
            assert pn.rail_banner(ACT, shield_on=True, fsm_state=st,
                                  setpoint=sp)[0]["level"] in COLOURS


def test_the_control_arm_always_says_shield_off_and_never_monitored():
    b, k = pn.rail_banner(OK, shield_on=False)
    assert k == "shield_off" and b["text"] == pn.SHIELD_OFF_IDLE and b["level"] == "watch"
    esc = {"level": "brake", "text": "P0 RULE STILL BROKEN AFTER SHIELD (alt-band)"}
    b, _ = pn.rail_banner(esc, shield_on=False)
    assert b["text"] == "SHIELD OFF - P0 RULE BROKEN, FLOWN UNCORRECTED (alt-band)", b
    w = {"level": "watch", "text": "RULE BROKEN - MONITORED, NOT CORRECTED (kin-caps)"}
    b, _ = pn.rail_banner(w, shield_on=False)
    assert "MONITORED" not in b["text"] and b["text"].startswith("SHIELD OFF - "), b
    # Above the Shield: the autopilot's takeover, then an FSM fault.
    b, k = pn.rail_banner(OK, shield_on=False, takeover_mode="RTL")
    assert k == "takeover" and "(RTL)" in b["text"], b
    b, k = pn.rail_banner(OK, shield_on=True, fsm_state=pn.FSMState.LOITER,
                          fsm_fault="refused")
    assert k == "fault" and "BRAKE" in b["text"], b


def test_the_shield_off_panel_sees_no_repair_and_the_raw_action_flown():
    """policy_hud shows ACTING and counts 'Shield corrections' from the
    decision's repairs; a Shield-off flight must show neither."""
    from guardrail.models import Action4D
    d = SimpleNamespace(violations=["v1"], repairs=["r1"], braked=True, touched=True,
                        emitted=Action4D(vx=1.0), emitted_violations=[])
    raw = Action4D(vx=3.0)
    assert pn.hud_decision(d, raw, shield_on=True) is d
    v = pn.hud_decision(d, raw, shield_on=False)
    assert v.repairs == [] and v.braked is False and v.emitted == raw
    assert v.emitted_violations == ["v1"], "the raw action's violations are what flew"
    assert pn.hud_decision(d, raw, shield_on=False, setpoint="none").emitted_violations == []


# =========================================================================== #
# The HUD the loop hands the recorder (fake clock, spy recorder)
# =========================================================================== #

def _huds(**kw):
    SpyRecorder.made.clear()
    rc, run, m, link, cam = _fly(recorder_cls=SpyRecorder, **kw)
    assert rc == 0
    assert len(SpyRecorder.made) == 1
    return SpyRecorder.made[0], run, m, link, cam


def test_every_tick_hands_the_recorder_the_hud_after_the_setpoint():
    if not _have_pil():
        return SKIP
    spy, run, m, link, cam = _huds()
    rows = T._rows(run)
    assert spy.started and spy.stopped, "the recorder must be started and stopped"
    assert spy.obs is cam, "the frames come from the camera (get_front_native)"
    assert len(spy.huds) == len(rows) == len(link.sent), (len(spy.huds), len(rows))
    for h, r, sent in zip(spy.huds, rows, link.sent):
        # follow_vlm.annotate's keys, all present.
        for k in ("t", "sep", "brg", "fwd", "alt", "shield", "query", "mode",
                  "mode_label", "tier", "fence_d", "fence_hold", "fence_mode",
                  "fence_kind", "policy"):
            assert k in h, k
        assert abs(h["t"] - r["t"]) < 1e-6 and h["query"] == "a red car"
        # The forward speed on screen is what was FLOWN (capped at 2 m/s by
        # the Shield), not the 2.6 m/s the pilot asked for.
        assert abs(h["fwd"] - (sent.vx * math.cos(math.radians(r["yaw_deg"]))
                               + sent.vy * math.sin(math.radians(r["yaw_deg"])))) < 1e-6
        assert h["fwd"] <= 2.0 + 1e-6
        assert h["policy"]["rows"] and "map" in h["policy"]
    assert max(math.hypot(r["raw"]["vx"], r["raw"]["vy"]) for r in rows) > 2.5
    assert spy.dets[0] is not None and spy.dets[0][5] == 768, "the box, in FrontCamera pixels"
    rec = m["recording"]
    assert rec["enabled"] and rec["hud_ms"]["n"] == len(rows) and rec["hud_failed_ticks"] == 0
    assert (run / "view" / "recorder.json").is_file()


def test_the_hud_shows_the_fsm_state_brake_loiter_and_rtl():
    if not _have_pil():
        return SKIP
    # The speed cap repaired every tick: the FSM sits in Brake streaming the
    # Shield's action, and the banner keeps the rule event.
    spy, run, m, *_ = _huds(policy=T.CAP_POLICY)
    rows = T._rows(run)
    for h, r in zip(spy.huds, rows):
        assert h["mode_label"].endswith(f"| FSM {r['fsm_state_after']}"), h["mode_label"]
    brake = [h for h, r in zip(spy.huds, rows) if r["fsm_state_after"] == "Brake"]
    assert brake and all(h["policy"]["banner"]["text"].startswith("FSM Brake | ")
                         for h in brake), brake[0]["policy"]["banner"]
    assert m["recording"]["policy_hud"]["banners_shown"].get("fsm_brake:Brake") == len(brake)
    # Loiter, flown as ArduCopter's BRAKE (the default hold mode).
    spy, run, m, link, _ = _huds(policy=T.LOITER_POLICY)
    assert "BRAKE" in link.requested
    loit = [h for h in spy.huds if "Loiter" in h["mode_label"]]
    assert loit and all(h["mode_label"].endswith("| FSM Loiter(BRAKE)") for h in loit)
    assert any(h["policy"]["banner"]["text"].startswith("FSM Loiter(BRAKE)")
               and h["policy"]["banner"]["level"] == "hold" for h in loit)
    # RTL.
    spy, run, m, *_ = _huds(policy=T.RTL_POLICY, max_s="6")
    rtl = [h for h in spy.huds if h["mode_label"].endswith("| FSM RTL")]
    assert rtl and all(h["policy"]["banner"]["text"].startswith("FSM RTL")
                       and h["policy"]["banner"]["level"] == "brake" for h in rtl)
    assert m["recording"]["policy_hud"]["banners_shown"].get("fsm:RTL") == len(rtl)


def test_the_hud_of_the_control_arm_says_shield_off_and_counts_no_correction():
    if not _have_pil():
        return SKIP
    spy, run, m, *_ = _huds(shield="off")
    assert spy.huds
    for h in spy.huds:
        ban = h["policy"]["banner"]
        assert ban["text"].startswith("SHIELD OFF"), ban
        assert "MONITORED" not in ban["text"] and "AFTER SHIELD" not in ban["text"]
        assert h["mode_label"].endswith("| SHIELD OFF")
        assert h["policy"]["counts"]["repaired"] == 0, "no Shield corrected anything"
        assert all(r["status"] != "act" for r in h["policy"]["rows"])
    # The speed cap broken and flown: the panel shows it, the banner names it.
    assert any(r["status"] in ("breach", "watch") for h in spy.huds
               for r in h["policy"]["rows"] if r["id"] == "kin-caps")
    assert m["recording"]["policy_hud"]["banners_shown"] == {"shield_off": len(spy.huds)}


def test_the_hud_says_when_the_autopilot_took_the_aircraft():
    if not _have_pil():
        return SKIP
    spy, run, m, *_ = _huds(link_kw={"takeover_at": (1.0, "RTL")}, max_s="40")
    t_take = m["autopilot_took_over"]["t"]
    after = [h for h in spy.huds if h["t"] >= t_take]
    assert after and all("AUTOPILOT TOOK THE AIRCRAFT (RTL)" in h["policy"]["banner"]["text"]
                         and h["mode_label"].endswith("| AUTOPILOT RTL") for h in after)
    assert all(h["fwd"] == 0.0 for h in after), "nothing is sent after a takeover"


def test_policy_hud_draws_the_fsm_and_shield_off_banners():
    """The dicts above, drawn by the overlay the video uses
    (follow_vlm.annotate -> policy_hud): the banner strip carries the
    level's colour."""
    if not _have_pil():
        return SKIP
    import numpy as np
    from PIL import Image
    from policy_hud import COLOURS
    fv = pn._fv()

    def banner_pixels(hud, colour):
        im = np.asarray(fv.annotate(Image.new("RGB", (1280, 720)), None, hud)).astype(int)
        top = im[:60, 300:980]
        # the banner is drawn at alpha 215 over black
        want = np.array(colour) * 215 / 255
        return int((np.abs(top - want).max(axis=2) < 12).sum())

    spy, *_ = _huds(policy=T.RTL_POLICY, max_s="6")
    rtl = next(h for h in spy.huds if h["mode_label"].endswith("| FSM RTL"))
    assert banner_pixels(rtl, COLOURS["brake"]) > 2000
    spy, *_ = _huds(shield="off")
    off = dict(spy.huds[0])
    off["policy"] = dict(off["policy"], banner={"level": "watch", "text": pn.SHIELD_OFF_IDLE})
    assert banner_pixels(off, COLOURS["watch"]) > 2000
    assert banner_pixels(off, COLOURS["brake"]) == 0


def test_a_recording_fault_never_costs_the_flight():
    if not _have_pil():
        return SKIP
    SpyRecorder.made.clear()
    rc, run, m, link, _ = _fly(recorder_cls=FailingRecorder)
    rows = T._rows(run)
    assert rc == 0 and len(link.sent) == len(rows) >= 25
    assert m["recording"]["hud_failed_ticks"] == len(rows)


def test_without_a_frame_source_the_flight_flies_and_says_why():
    rc, run, m, link, _ = _fly(camera_cls=T.FakeCamera)
    assert rc == 0 and len(link.sent) >= 25
    assert m["recording"]["enabled"] is False and "get_front_native" in m["recording"]["why"]
    assert not (run / "view" / "recorder.json").exists()
    # And without --save-view nothing is recorded or measured for it.
    rc, run, m, *_ = _fly(save=False)
    assert m["recording"] == {"enabled": False, "why": "--save-view not given"}
    assert not (run / "view").exists()


class LevelIndicator:
    """policy_hud.PolicyIndicator's interface, its banner level set per tick."""
    hold_s = 1.5
    shield = SimpleNamespace(has_obstacle_map=False, off_map=lambda x, y: False)

    def __init__(self):
        self.level = "ok"

    def update(self, now, decision, state, **kw):
        return {"banner": {"level": self.level, "text": f"POLICY {self.level.upper()}"},
                "rows": [], "counts": {}}


def test_a_one_tick_escalation_stays_on_screen_for_the_hold_time():
    """A Brake with a zero setpoint for ONE tick (0.1 s, three frames at
    20 Hz) is latched for the indicator's hold_s, so the video shows it; a
    policy banner at HOLD or above is never hidden under the latch."""
    from guardrail.models import State
    ind = LevelIndicator()
    hud = pn.RailHud(ind, query="a red car", shield_on=True)
    st, F = State(x=0.0, y=0.0, up=8.0), pn.FSMState

    def tick(now, fsm, setpoint="pass"):
        return hud.tick(now, decision=None, state=st, yaw_rad=0.0, info={"mode": "track"},
                        fsm_state=fsm, setpoint=setpoint)["policy"]["banner"]
    b = tick(0.0, F.BRAKE, "brake")
    assert "ZERO SETPOINT" in b["text"] and b["level"] == "brake", b
    b = tick(0.5, F.NORMAL)
    assert "ZERO SETPOINT" in b["text"], "a one-tick Brake vanished from the screen"
    ind.level = "hold"
    assert tick(1.0, F.NORMAL)["text"] == "POLICY HOLD", "the latch hid a HOLD banner"
    ind.level = "ok"
    assert "ZERO SETPOINT" in tick(1.4, F.NORMAL)["text"]
    assert tick(1.6, F.NORMAL)["text"] == "POLICY OK", "latched past hold_s"
    assert hud.shown["fsm:Brake"] == 1 and hud.shown["policy_latched"] == 0
    assert hud.shown["fsm:Brake_latched"] == 2, dict(hud.shown)


def test_the_hud_speed_is_the_flown_velocity_along_the_heading():
    """Facing east (yaw 90 deg), 2 m/s east is 2 m/s forward and 2 m/s north
    is none: the scene-frame action is projected on the heading, not read
    off its north component."""
    from guardrail import State
    from guardrail.models import Action4D
    hud = pn.RailHud(None, query="a red car", shield_on=True)
    st = State(x=0.0, y=0.0, up=8.0, yaw_deg=90.0)

    def fwd(a):
        return hud.tick(0.0, decision=None, state=st, yaw_rad=math.pi / 2,
                        info={"mode": "track"}, flown=a)["fwd"]
    assert abs(fwd(Action4D(vx=0.0, vy=2.0)) - 2.0) < 1e-9
    assert abs(fwd(Action4D(vx=2.0, vy=0.0))) < 1e-9
    assert fwd(None) == 0.0


def test_run_subscribes_the_chase_camera_only_with_save_view():
    """run()'s own wiring, not just the class: the camera it builds is asked
    for the Chase view exactly when --save-view is given."""
    class Built(Exception):
        pass
    made = []

    class SpyCamera:
        def __init__(self, obs, cfg, **kw):
            made.append(kw)
            raise Built()
    orig = pn.ProjectAirSimCamera
    pn.ProjectAirSimCamera = SpyCamera
    try:
        for save in (False, True):
            out = _tmp()
            pol = out / "policy.yaml"
            pol.write_text(T.CAP_POLICY, encoding="utf-8")
            args = pn.build_parser().parse_args([
                "--out-root", str(out), "--tag", "run", "--policy", str(pol),
                "--citymap", str(out / "no_map.npz"), "--network", "nat",
                "--windows-ip", T.WIN, "--wsl-ip", T.WSL, "--dev-yaml",
                *(["--save-view"] if save else [])])
            clk = T.Clock()
            try:
                pn.run(args, camera=None, obs=T.FakeObs(), grounder=T.FakeDetections(clk),
                       clock=clk, sleep=clk.sleep)
            except Built:
                pass
            else:
                raise AssertionError("run() never built its camera")
    finally:
        pn.ProjectAirSimCamera = orig
    assert [kw.get("chase") for kw in made] == [False, True], made


def test_live_view_needs_save_view():
    args = pn.build_parser().parse_args(["--live-view"])
    try:
        pn.check_args(args)
    except SystemExit as exc:
        assert "--save-view" in str(exc)
    else:
        raise AssertionError("--live-view without --save-view was accepted")
    a = pn.build_parser().parse_args(["--save-view"])
    assert a.record_hz == 20.0 and a.record_height == 720 and not a.no_policy_hud


# =========================================================================== #
# The camera: the Chase view, subscribed for the video only
# =========================================================================== #

class ChaseDrone(T.FakeDrone):
    def __init__(self, client, world, name):
        super().__init__(client, world, name)
        self.sensors = {"FrontCamera": {"scene_camera": "s", "depth_camera": "d"},
                        "Chase": {"scene_camera": "chase"}}


class Obs:
    def __init__(self):
        self.chase = None

    def put_front(self, m):
        pass

    def put_depth(self, m):
        pass

    def put_chase(self, m):
        self.chase = m

    def get_front_native(self):
        return "front-image"

    def get_chase_native(self):
        return None if self.chase is None else "chase-image"


def test_the_chase_camera_is_subscribed_for_the_video_only():
    live = ["/Sim/SceneCityLifeArduPilot/robots/Drone1/sensors/FrontCamera/scene_camera"]
    cfg = rail.write_sim_configs(_tmp(), rail.plan_network("mirrored"))
    pas = SimpleNamespace(ProjectAirSimClient=lambda address: T.FakeClient2(address, live),
                          World=T.FakeWorld, Drone=ChaseDrone)
    cam = pn.ProjectAirSimCamera(Obs(), cfg, projectairsim=pas)
    cam.start()
    assert "chase" not in cam.client.subs and cam.get_chase_native() is None
    assert "chase_subscribed" not in cam.scene_record
    obs = Obs()
    cam = pn.ProjectAirSimCamera(obs, cfg, projectairsim=pas, chase=True)
    cam.start()
    assert "chase" in cam.client.subs and cam.have_chase
    assert cam.scene_record["chase_subscribed"] is True
    assert cam.get_front_native() == "front-image"
    assert cam.get_chase_native() is None, "no message buffered yet"
    obs.put_chase({"data": b"x"})
    assert cam.get_chase_native() == "chase-image"
    # A robot config without the Chase camera: the first-person view alone.
    pas2 = SimpleNamespace(ProjectAirSimClient=lambda address: T.FakeClient2(address, live),
                           World=T.FakeWorld, Drone=T.FakeDrone)
    cam = pn.ProjectAirSimCamera(Obs(), cfg, projectairsim=pas2, chase=True)
    cam.start()
    assert cam.have_chase is False and cam.scene_record["chase_subscribed"] is False
    assert cam.get_chase_native() is None and cam.get_front_native() == "front-image"
    # The rendered robot config this rail flies has that camera.
    robot = rail.load_jsonc(Path(cfg["dir"]) / cfg["robot"])
    assert any(s.get("id") == "Chase" for s in robot["sensors"])


# =========================================================================== #
# Real clock: real frames, and the loop rate with and without them
# =========================================================================== #

def test_recording_writes_both_views_and_the_rate_the_video_takes():
    if not _have_pil():
        return SKIP
    out = _tmp()
    try:
        _check_recorded_flight(out)
    finally:
        shutil.rmtree(out, ignore_errors=True)       # the flight's noise JPEGs


def _check_recorded_flight(out: Path) -> None:
    from PIL import Image
    rc, run, m, link, cam = _fly(real=True, max_s="3", out=out)
    assert rc == 0
    view = run / "view"
    fpv, tps = _jpgs(view / "fpv"), _jpgs(view / "tps")
    side = json.loads((view / "recorder.json").read_text())
    assert side["frames"] == len(fpv) == len(tps) > 20, (side["frames"], len(fpv), len(tps))
    assert side["failed"] == 0 and side["empty_captures"] == 0
    # tools/make_demo_video.py takes its fps from achieved_hz: it must be the
    # rate measured over the frames written.
    assert side["achieved_hz"] and 10.0 <= side["achieved_hz"] <= 22.0, side["achieved_hz"]
    assert len(side["frame_t"]) == side["frames"] and side["frame_t"][0] < 0.2
    with Image.open(fpv[0]) as im:
        assert im.size == (1280, 720), "the FrontCamera frame enlarged to --record-height"
    with Image.open(tps[0]) as im:
        assert im.size == (960, 540), "the Chase frame as captured"
    rec = m["recording"]
    assert rec["recorder"]["frames"] == side["frames"] and "frame_t" not in rec["recorder"]
    assert rec["view_dir"] == str(view)


def test_recording_does_not_slow_the_loop():
    """The loop rate with the recorder writing 20 Hz of 720p JPEGs is held to
    within 5 % of the same flight without it, on the wall clock. Printed by
    this test on the desktop, 2026-10-14, two runs: 9.135 / 9.146 Hz without,
    9.173 / 9.139 Hz with; the HUD's per-tick cost on the loop p95 0.210 /
    0.223 ms; the recorder at 20.16 Hz both times. (The stand-in, like the
    rail - pas_ap_20261008_224440 metrics.json loop_hz 9.164 - sleeps TICK
    minus the tick's work, so neither reaches 10.0.)"""
    if not _have_pil():
        return SKIP
    out0, out1 = _tmp(), _tmp()
    try:
        _, _, m0, *_ = _fly(real=True, max_s="4", save=False, out=out0)
        _, _, m1, *_ = _fly(real=True, max_s="4", out=out1)
    finally:
        shutil.rmtree(out0, ignore_errors=True)
        shutil.rmtree(out1, ignore_errors=True)
    hz0, hz1 = m0["loop_hz"], m1["loop_hz"]
    print(f"      loop_hz without recording {hz0}, with {hz1}; work p95 "
          f"{m0['tick_work_ms']['p95']} / {m1['tick_work_ms']['p95']} ms; HUD p95 "
          f"{m1['recording']['hud_ms']['p95']} ms; recorder "
          f"{m1['recording']['recorder']['achieved_hz']} Hz")
    assert hz1 >= 0.95 * hz0, (hz0, hz1)
    assert m1["tick_work_ms"]["p95"] < 100.0, m1["tick_work_ms"]
    assert m1["recording"]["hud_ms"]["p95"] < 20.0, m1["recording"]["hud_ms"]
    # The recorder really ran alongside: frames for most of the flight.
    r = m1["recording"]["recorder"]
    assert r["frames"] >= 0.5 * 20.0 * m1["mission_s"], r


class BusyDetector:
    """A stand-in for the grounder's CPU share, on its own thread beside the
    loop and the recorder: per cycle a FrontCamera-sized frame resized to
    OWL-ViT's 768x768 input and normalised in numpy (the preprocessing that
    holds the GIL; follow_vlm.Grounder._worker times it at 26.7 ms offline),
    then a 36 ms sleep for the GPU forward pass (36.2 ms there), which
    releases it. It counts its cycles; it feeds the pilot nothing."""

    def __init__(self):
        import threading
        import numpy as np
        self.frame = np.random.default_rng(3).integers(0, 255, (432, 768, 3), dtype=np.uint8)
        self.n, self._stop = 0, threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.t0 = self.t1 = None

    def _run(self):
        import numpy as np
        from PIL import Image
        self.t0 = time.time()
        while not self._stop.is_set():
            im = Image.fromarray(self.frame).resize((768, 768), Image.BICUBIC)
            a = np.asarray(im, dtype=np.float32) / 255.0
            _ = ((a - a.mean(axis=(0, 1))) / (a.std(axis=(0, 1)) + 1e-6)).transpose(2, 0, 1)
            time.sleep(0.036)
            self.n += 1
        self.t1 = time.time()

    def __enter__(self):
        self.th.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self.th.join(5)

    @property
    def hz(self) -> float:
        return self.n / max(1e-6, (self.t1 or time.time()) - self.t0)


def test_recording_beside_a_cpu_bound_detector_thread():
    """REVIEW (2026-10-14): the loop-rate test above has no detector thread,
    and on the rail the recorder's decode, HUD draw and JPEG encode share the
    CPU and the GIL with the grounder. Here a thread doing the grounder's CPU
    share (BusyDetector) runs beside both flights: the loop holds 95 % of its
    rate and the detector stand-in 85 % of its own. Printed by this test on
    the desktop, 2026-10-14, two runs: loop 9.151 / 9.16 Hz without
    recording, 9.124 / 9.128 Hz with; the detector stand-in 13.34 / 12.81 Hz
    without, 12.78 / 12.78 Hz with; the recorder 20.11 / 20.1 Hz. A
    stand-in, not OWL-ViT: the first -SaveView flight's det_hz and
    tick_work_ms against a flight without it is the real measure."""
    if not _have_pil():
        return SKIP
    out0, out1 = _tmp(), _tmp()
    try:
        with BusyDetector() as d0:
            _, _, m0, *_ = _fly(real=True, max_s="4", save=False, out=out0)
        with BusyDetector() as d1:
            _, _, m1, *_ = _fly(real=True, max_s="4", out=out1)
    finally:
        shutil.rmtree(out0, ignore_errors=True)
        shutil.rmtree(out1, ignore_errors=True)
    print(f"      with a detector stand-in: loop_hz {m0['loop_hz']} without recording, "
          f"{m1['loop_hz']} with; work p95 {m0['tick_work_ms']['p95']} / "
          f"{m1['tick_work_ms']['p95']} ms; detector stand-in {d0.hz:.2f} / {d1.hz:.2f} Hz; "
          f"recorder {m1['recording']['recorder']['achieved_hz']} Hz")
    assert m1["loop_hz"] >= 0.95 * m0["loop_hz"], (m0["loop_hz"], m1["loop_hz"])
    assert d1.hz >= 0.85 * d0.hz, (d0.hz, d1.hz)
    assert m1["recording"]["recorder"]["frames"] >= 0.5 * 20.0 * m1["mission_s"]


def test_the_hud_tick_is_cheap_on_the_citylife_map():
    """The stand-in flies with no obstacle map. The rail flies CityLife's
    (demo/out/citymap_citylife, gitignored: SKIP without it) under
    follow_car_citylife_nfz.yaml, and the indicator reads the Shield's
    clearance field each tick - timed here over 300 ticks of a pass along
    the street, with the real map and the minimap's base."""
    if not _have_pil() or not pn.DEFAULT_CITYMAP.is_file():
        return SKIP
    import statistics
    from guardrail import load_policy, State
    from guardrail.models import Action4D
    from guardrail.replay import rail_shield
    policy = load_policy(ROOT / "policies" / "follow_car_citylife_nfz.yaml")
    smap, note = pn.load_obstacle_map(policy, pn.DEFAULT_CITYMAP)
    assert smap is not None, note
    shield = rail_shield(policy, lookahead_s=pn.LOOKAHEAD_S, dt=pn.LOOKAHEAD_DT_S,
                         obstacle_map=smap)
    ind = pn.build_policy_hud(policy, shield, smap, pn.DEFAULT_CITYMAP)
    assert ind.base_map is not None, "the minimap has the map under it"
    hud = pn.RailHud(ind, query="a red car", shield_on=True)
    ms = []
    for k in range(300):
        st = State(x=35.0 + 0.3 * k, y=-20.0, up=8.0, yaw_deg=10.0)
        d = shield.filter(st, Action4D(vx=3.0, vy=0.5, vz_up=0.0, yaw_rate=0.1))
        t0 = time.perf_counter()
        h = hud.tick(k * 0.1, decision=d, state=st, yaw_rad=math.radians(10.0),
                     info={"mode": "track", "bearing": 0.1}, subject_xy=(st.x + 15, st.y),
                     subject_class="car", fsm_state=pn.FSMState.NORMAL, flown=d.emitted,
                     sep_m=15.0)
        ms.append((time.perf_counter() - t0) * 1000.0)
    ms.sort()
    p95 = ms[int(0.95 * len(ms)) - 1]
    print(f"      HUD tick on the CityLife map: median {statistics.median(ms):.3f} ms, "
          f"p95 {p95:.3f} ms, max {ms[-1]:.3f} ms over {len(ms)} ticks")
    assert hud.error is None and h["policy"]["rows"]
    assert p95 < 5.0, p95


# =========================================================================== #
# The launcher: -SaveView
# =========================================================================== #

def _pwsh():
    return shutil.which("pwsh") or shutil.which("powershell")


def test_the_launcher_records_through_a_c_drive_junction_and_prints_the_video_command():
    src = (ROOT / "scripts" / "run_pas_ardupilot.ps1").read_text(encoding="utf-8")
    node = src[src.index('"node" {'):src.index('"g1" {')]
    assert "Set-ViewJunction $Tag" in node and '$a += @("--save-view")' in node
    assert "make_demo_video.py --tag $Tag --height 720" in node
    assert "make_compare_video.py" in node
    assert "[switch]$SaveView" in src and "vla_drone_offload" in src
    if _pwsh() is None or not sys.platform.startswith("win"):
        return SKIP
    fn = src[src.index("function Set-ViewJunction"):src.index("if ($SaveView -and $Step")]
    out, off = _tmp(), _tmp()

    def run_fn(tag):
        ps = "\n".join([
            '$ErrorActionPreference = "Stop"', f"$Out = '{out}'", f"$OffloadRoot = '{off}'",
            'function Say($m) { [Console]::Out.WriteLine("SAY $m") }',
            'function Warn($m) { [Console]::Out.WriteLine("WARN $m") }',
            fn, f"Set-ViewJunction '{tag}'", ""])
        f = _tmp() / "junction.ps1"
        f.write_text(ps, encoding="utf-8")
        r = subprocess.run([_pwsh(), "-NoProfile", "-NonInteractive", "-File", str(f)],
                           capture_output=True, text=True, timeout=120)
        assert r.returncode == 0, r.stderr
        return r.stdout

    said = run_fn("t1")
    link = out / "t1" / "view"
    assert link.is_dir() and (off / "t1" / "view").is_dir(), said
    (link / "probe.txt").write_text("x", encoding="utf-8")
    assert (off / "t1" / "view" / "probe.txt").is_file(), "view/ is not a junction to the offload"
    said = run_fn("t1")
    assert "(junction)" in said, said
    # A real folder that already holds frames is left alone, and warned about.
    real = out / "t2" / "view" / "fpv"
    real.mkdir(parents=True)
    (real / "00000.jpg").write_bytes(b"jpg")
    said = run_fn("t2")
    assert "WARN" in said and (real / "00000.jpg").is_file(), said
    assert not (off / "t2").exists()


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
        except (Exception, SystemExit) as e:          # noqa: BLE001  (argparse exits)
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
