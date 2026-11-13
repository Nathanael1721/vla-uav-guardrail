"""Perception stress on the Project AirSim + ArduPilot rail, checked offline.

Run either way:
    pytest tests/test_pas_perception_stress.py -v
    python tests/test_pas_perception_stress.py

WHAT IS TESTED (WP4-16, WP4-18; demo/pas_ardupilot/perception_stress.py,
tools/perception_stress_report.py)

  * the stressor table against the installed Project AirSim client: the
    WeatherParameter mirror, and the World methods the stressors call;
  * the rendering: each stressor's scene id, the exposure on every scene
    capture and no depth capture, the record, the unstressed configs
    byte-for-byte as before, and a stressed config the client's own loader
    accepts;
  * the weather: set after a load in the order the plugin needs (enabled
    first), read back, and refused when a call fails or a value does not
    stick; only read back by an attaching node, which refuses a scene loaded
    without its stressor (its scene id) or with another weather;
  * the node: --stress parsed, refused with a ROS 2 camera, the scene step's
    record, a flown run's metrics.perception_stress and its non-KPI reason;
  * the launchers: the desktop's -Stress and the Orin's STRESS;
  * the detector's frame luma, and the per-scenario metric over flight logs
    and detection logs built here.

Nothing here starts a simulator, an autopilot or a GPU.
"""
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "demo", PAS, ROOT / "tests", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import perception_stress as ps                                      # noqa: E402
import rail                                                          # noqa: E402
import perception_node as pn                                         # noqa: E402
import perception_stress_report as R                                 # noqa: E402
import test_pas_ardupilot as T                                       # noqa: E402

SKIP = "SKIP"
BASE_ID = "SceneCityLifeArduPilot"


def _tmp() -> Path:
    return Path(tempfile.mkdtemp())


def _raises(fn, *a, contains: str = "", exc=(ValueError, RuntimeError), **k) -> str:
    try:
        fn(*a, **k)
    except exc as e:
        assert contains.lower() in str(e).lower(), f"wrong reason: {e}"
        return str(e)
    raise AssertionError(f"{getattr(fn, '__name__', fn)} accepted {a} {k}")


def _f32(v: float) -> float:
    return struct.unpack("f", struct.pack("f", float(v)))[0]


class WeatherSim:
    """The plugin's weather as World/WeatherLib.cpp keeps it: a parameter set
    while the effects are disabled is written as 0 and answers false; the
    collection stores float32; a reset puts the defaults back."""

    def __init__(self, start=None, enable_ok=True, ignore_sets=False, get_fails=False):
        self.p = {i: 0.0 for i in range(9)}
        self.p.update(start or {})
        self.enable_ok, self.ignore_sets, self.get_fails = enable_ok, ignore_sets, get_fails
        self.calls = []

    def reset_weather_effects(self):
        self.calls.append("reset")
        self.p = {i: 0.0 for i in range(9)}
        return True

    def enable_weather_visual_effects(self):
        self.calls.append("enable")
        if not self.enable_ok:
            return False
        self.p[0] = 1.0
        return True

    def disable_weather_visual_effects(self):
        self.calls.append("disable")
        self.p[0] = 0.0
        return True

    def set_weather_visual_effects_param(self, param, value):
        self.calls.append(("set", param, value))
        if param != 0 and self.p[0] != 1.0:
            self.p[param] = 0.0
            return False
        if not self.ignore_sets:
            self.p[param] = _f32(value)
        return True

    def get_weather_visual_effects_param(self):
        self.calls.append("get")
        if self.get_fails:
            raise TimeoutError("no answer")
        return {i: self.p[i] for i in range(1, 9)}


# --------------------------------------------------------------------------- #
# The table against the installed client
# --------------------------------------------------------------------------- #

def test_the_weather_mirror_and_the_calls_are_the_installed_clients():
    """WEATHER_PARAMS mirrors projectairsim.types.WeatherParameter, and every
    World method a stressor calls exists with the arguments it is given."""
    try:
        from projectairsim.types import WeatherParameter
        from projectairsim import World
    except Exception:                                         # noqa: BLE001
        return SKIP
    assert ps.WEATHER_PARAMS == {m.name: int(m) for m in WeatherParameter}, \
        {m.name: int(m) for m in WeatherParameter}
    import inspect
    for name, params in (("reset_weather_effects", []),
                         ("enable_weather_visual_effects", []),
                         ("disable_weather_visual_effects", []),
                         ("set_weather_visual_effects_param", ["param", "value"]),
                         ("get_weather_visual_effects_param", [])):
        sig = inspect.signature(getattr(World, name))
        assert [p for p in sig.parameters if p != "self"] == params, (name, sig)
    # The sun is never touched (no SunSky in CityLife_Day): none of the
    # time-of-day calls appears in the module's code (its docstring names them).
    import ast
    tree = ast.parse((PAS / "perception_stress.py").read_text(encoding="utf-8"))
    called = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert not called & {"set_time_of_day", "set_sun_position_from_date_time",
                         "set_sunlight_intensity", "set_cloud_shadow_strength",
                         "set_light_object_intensity"}, called


def test_the_table_names_five_stressors_one_variable_each():
    assert ps.NAMES == ("day", "dusk", "night", "rain", "fog")
    for n, s in ps.STRESSORS.items():
        assert not (s["exposure_bias_ev"] is not None and s["weather"]), \
            f"{n} changes two things at once"
        assert ps.scenario_id(n) == f"pas-follow-redcar-{n}"
        for p in s["weather"]:
            assert p in ps.WEATHER_PARAMS and p != "ENABLED", p
    assert ps.STRESSORS["dusk"]["exposure_bias_ev"] == -2.0
    assert ps.STRESSORS["night"]["exposure_bias_ev"] == -4.0
    assert ps.STRESSORS["rain"]["weather"] == {"RAIN": 0.8}
    assert ps.STRESSORS["fog"]["weather"] == {"FOG": 0.6}
    assert ps.WEATHER_CHECKED == ("RAIN", "FOG")
    _raises(ps.stressor, "dawn", contains="day, dusk, night, rain, fog")
    assert "pas-follow-redcar-night" in ps.non_kpi_reason("night")


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #

def _captures(robot: dict) -> dict:
    return {(s["id"], c["image-type"]): c for s in robot["sensors"]
            if s.get("type") == "camera" for c in s.get("capture-settings", [])}


def _strip(cfg: dict) -> dict:
    return {k: v for k, v in cfg.items() if k not in ("dir", "stress", "scene_id",
                                                     "scene_sha256", "robot_sha256")}


def test_every_stressor_is_rendered_into_the_scene_and_recorded():
    plan = rail.plan_network("mirrored")
    plain_dir = _tmp()
    plain = rail.write_sim_configs(plain_dir, plan, vla_cameras=True)
    assert "stress" not in plain and plain["scene_id"] == BASE_ID
    robot0 = rail.load_jsonc(plain_dir / plain["robot"])
    for name in ps.NAMES:
        d = _tmp()
        cfg = rail.write_sim_configs(d, plan, vla_cameras=True, stress=name)
        st = cfg["stress"]
        want_id = BASE_ID + "Stress" + name.capitalize()
        assert cfg["scene_id"] == want_id == st["scene_id"], cfg["scene_id"]
        assert rail.load_jsonc(d / cfg["scene"])["id"] == want_id
        assert st["name"] == name and st["scenario_id"] == f"pas-follow-redcar-{name}"
        assert st["kpi"] == "non-KPI" and "SunSky" in st["sun"]
        # Everything else is the unstressed run's.
        assert _strip(cfg) == _strip(plain), (name, _strip(cfg))
        robot = rail.load_jsonc(d / cfg["robot"])
        caps, caps0 = _captures(robot), _captures(robot0)
        ev = ps.STRESSORS[name]["exposure_bias_ev"]
        for key, c in caps.items():
            rest = {k: v for k, v in c.items() if k != "auto-exposure-bias"}
            assert rest == caps0[key], (name, key)
            if ev is not None and key[1] == 0:
                assert c["auto-exposure-bias"] == ev, (name, key)
            else:
                assert "auto-exposure-bias" not in c, (name, key)
        if ev is not None:
            assert sorted(st["exposure"]["captures"]) == [
                "Chase/0", "FrontCamera/0", "VLADown/0", "VLAFront/0"], st["exposure"]
        else:
            assert st["exposure"]["captures"] == []
        # The robot outside its cameras is the unstressed one.
        strip_cams = lambda r: [s for s in r["sensors"] if s.get("type") != "camera"]
        assert strip_cams(robot) == strip_cams(robot0)
        assert {k: v for k, v in robot.items() if k != "sensors"} == \
            {k: v for k, v in robot0.items() if k != "sensors"}
    # A detector-pilot run's config (no VLA cameras) still exposes its own.
    cfg = rail.write_sim_configs(_tmp(), plan, stress="night")
    assert sorted(cfg["stress"]["exposure"]["captures"]) == ["Chase/0", "FrontCamera/0"]
    _raises(rail.write_sim_configs, _tmp(), plan, stress="dawn", contains="no perception")
    # A config with no scene capture cannot be exposed: refused, not ignored.
    _raises(ps.render_cameras, {"sensors": [{"id": "IMU", "type": "imu"}]}, "night",
            contains="no scene capture")


def test_the_attaching_node_records_the_same_scene_as_the_desktop_loads():
    plan = rail.plan_network("mirrored")
    for name in ps.NAMES:
        desk = rail.write_sim_configs(_tmp(), plan, vla_cameras=True, stress=name)
        orin = rail.write_attach_configs(_tmp(), vla_cameras=True, stress=name)
        assert orin["scene_id"] == desk["scene_id"], (name, orin["scene_id"])
        assert orin["stress"] == desk["stress"], name
        assert orin["network"]["mode"] == "attach"
    assert "stress" not in rail.write_attach_configs(_tmp())


def test_a_stressed_config_loads_through_the_clients_own_loader():
    """The client validates the robot config against its schema, which does
    not list auto-exposure-bias: it must still load (no additionalProperties)."""
    try:
        from projectairsim.utils import load_scene_config_as_dict
    except Exception:                                         # noqa: BLE001
        return SKIP
    cfg = rail.write_sim_configs(_tmp(), rail.plan_network("mirrored"), vla_cameras=True,
                                 stress="night")
    data, _ = load_scene_config_as_dict(cfg["scene"], cfg["dir"])
    assert data["id"] == BASE_ID + "StressNight"
    sensors = {s["id"]: s for s in data["actors"][0]["robot-config"]["sensors"]}
    assert sensors["FrontCamera"]["capture-settings"][0]["auto-exposure-bias"] == -4.0
    assert "auto-exposure-bias" not in sensors["FrontCamera"]["capture-settings"][1]


# --------------------------------------------------------------------------- #
# The weather
# --------------------------------------------------------------------------- #

def test_a_weather_stressor_enables_first_sets_and_reads_back():
    w = WeatherSim()
    rec = ps.apply_weather(w, "rain")
    assert rec["ok"] and rec["problems"] == [], rec
    assert w.calls == ["reset", "enable", ("set", 1, 0.8), "get"], w.calls
    assert abs(rec["read_back"]["RAIN"] - 0.8) < 1e-6 and rec["read_back"]["FOG"] == 0.0
    assert rec["asked"] == {"RAIN": 0.8} and rec["mode"] == "set"
    w = WeatherSim()
    rec = ps.apply_weather(w, "fog")
    assert rec["ok"] and w.calls == ["reset", "enable", ("set", 8, 0.6), "get"], w.calls
    json.dumps(rec)                                              # the record is JSON


def test_an_unweathered_stressor_resets_the_weather_and_checks_it_is_off():
    for name in ("day", "dusk", "night"):
        w = WeatherSim(start={0: 1.0, 1: 0.8})             # rain left on from before
        rec = ps.apply_weather(w, name)
        assert rec["ok"], rec
        assert w.calls == ["reset", "disable", "get"], w.calls
        assert rec["read_back"]["RAIN"] == 0.0


def test_a_weather_that_did_not_stick_is_refused_with_why():
    # The effects could not be enabled: the plugin then writes 0 and says false.
    rec = ps.apply_weather(WeatherSim(enable_ok=False), "rain")
    assert not rec["ok"]
    assert any("enable_weather_visual_effects() answered False" in p
               for p in rec["problems"]), rec["problems"]
    assert any("RAIN reads back 0" in p for p in rec["problems"]), rec["problems"]
    # The set answered true but nothing changed.
    rec = ps.apply_weather(WeatherSim(ignore_sets=True), "fog")
    assert not rec["ok"] and rec["problems"] == ["FOG reads back 0.0, not 0.6"], rec
    # No read-back at all.
    rec = ps.apply_weather(WeatherSim(get_fails=True), "rain")
    assert not rec["ok"] and rec["read_back"] is None
    assert rec["problems"] == ["get_weather_visual_effects_param() failed "
                               "(TimeoutError: no answer)"], rec["problems"]

    class Broken(WeatherSim):
        def reset_weather_effects(self):
            raise ConnectionError("gone")
    rec = ps.apply_weather(Broken(), "day")
    assert not rec["ok"] and "reset_weather_effects() failed" in rec["problems"][0]


def test_an_attached_node_only_reads_the_weather_back():
    w = WeatherSim(start={0: 1.0, 1: _f32(0.8)})
    rec = ps.check_weather(w, "rain")
    assert rec["ok"] and w.calls == ["get"], (rec, w.calls)
    assert "attached" in rec["mode"]
    rec = ps.check_weather(WeatherSim(), "rain")
    assert not rec["ok"] and rec["problems"] == ["RAIN reads back 0.0, not 0.8"]
    rec = ps.check_weather(WeatherSim(start={0: 1.0, 8: _f32(0.6)}), "night")
    assert not rec["ok"] and "FOG reads back 0.6" in rec["problems"][0], rec["problems"]
    rec = ps.check_weather(WeatherSim(get_fails=True), "day")
    assert not rec["ok"]
    assert ps.named_read_back({"1": 0.5, 8: 0.25}) == {"RAIN": 0.5, "FOG": 0.25}
    assert ps.named_read_back(None) is None
    assert ps.verify_weather(None, "day") == ["the simulator's weather parameters "
                                              "could not be read back"]


# --------------------------------------------------------------------------- #
# The node
# --------------------------------------------------------------------------- #

class _Client(T.FakeClient2):
    def __init__(self, address="127.0.0.1", topics=None, weather=None):
        super().__init__(address, topics)
        self.weather = weather or WeatherSim()


class _World:
    """A World whose weather calls reach the client's WeatherSim, as the real
    World's reach the simulator through client.request."""
    loads = []

    def __init__(self, client, scene_config_name="", delay_after_load_sec=0,
                 sim_config_path="sim_config/", sim_instance_idx=-1):
        _World.loads.append(scene_config_name)
        self.client = client

    def switch_streaming_view(self):
        pass

    def reset_weather_effects(self):
        return self.client.weather.reset_weather_effects()

    def enable_weather_visual_effects(self):
        return self.client.weather.enable_weather_visual_effects()

    def disable_weather_visual_effects(self):
        return self.client.weather.disable_weather_visual_effects()

    def set_weather_visual_effects_param(self, param, value):
        return self.client.weather.set_weather_visual_effects_param(param, value)

    def get_weather_visual_effects_param(self):
        return self.client.weather.get_weather_visual_effects_param()


def _pas(topics=None, weather=None):
    client = _Client(topics=topics, weather=weather)
    return SimpleNamespace(ProjectAirSimClient=lambda address: client, World=_World,
                           Drone=T.FakeDrone), client


def _live(scene_id: str) -> list[str]:
    return [f"/Sim/{scene_id}/robots/Drone1/sensors/FrontCamera/scene_camera"]


def test_the_parser_takes_the_stressors_and_refuses_the_rest():
    a = pn.build_parser().parse_args(["--stress", "night", "--network", "mirrored"])
    assert a.stress == "night"
    pn.check_args(a)
    assert pn.build_parser().parse_args([]).stress is None
    _raises(pn.build_parser().parse_args, ["--stress", "dawn"], exc=SystemExit)
    a = pn.build_parser().parse_args(["--stress", "rain", "--camera", "ros2", "--link",
                                      "mavros"])
    _raises(pn.check_args, a, contains="--camera ros2", exc=SystemExit)


def test_a_node_that_loads_the_scene_sets_its_weather_and_one_that_attaches_checks_it():
    plan = rail.plan_network("mirrored")
    # load: the stressed scene is loaded, then its weather set and read back.
    cfg = rail.write_sim_configs(_tmp(), plan, stress="fog")
    pas, client = _pas(topics=None)
    _World.loads.clear()
    cam = pn.ProjectAirSimCamera(T.FakeObs(), cfg, projectairsim=pas, stress="fog")
    cam.start()
    assert _World.loads == ["scene_citylife_ardupilot.jsonc"]
    assert client.weather.calls == ["reset", "enable", ("set", 8, 0.6), "get"]
    assert cam.scene_record["stress"]["weather"]["ok"]
    assert cam.scene_record["stress"]["scenario_id"] == "pas-follow-redcar-fog"
    # load, the weather does not stick: refused before ArduPilot starts.
    pas, _ = _pas(weather=WeatherSim(ignore_sets=True))
    cam = pn.ProjectAirSimCamera(T.FakeObs(), cfg, projectairsim=pas, stress="fog")
    _raises(cam.start, contains="FOG reads back 0.0")
    # attach to the night scene the desktop loaded with -Stress night: ok,
    # nothing set, the weather read back.
    orin = rail.write_attach_configs(_tmp(), stress="night")
    pas, client = _pas(topics=_live(BASE_ID + "StressNight"))
    _World.loads.clear()
    cam = pn.ProjectAirSimCamera(T.FakeObs(), orin, projectairsim=pas, attach=True,
                                 stress="night")
    cam.start()
    assert _World.loads == [] and client.weather.calls == ["get"], client.weather.calls
    assert cam.world.parent_topic == f"/Sim/{BASE_ID}StressNight"
    assert cam.scene_record["stress"]["weather"]["ok"]
    # attach with --stress night to a scene loaded WITHOUT it: its id is not
    # the night scene's, so the node refuses and says how to load it.
    pas, _ = _pas(topics=_live(BASE_ID))
    cam = pn.ProjectAirSimCamera(T.FakeObs(), orin, projectairsim=pas, attach=True,
                                 stress="night")
    _raises(cam.start, contains="-Step scene -Stress night")
    # ... and an unstressed node to a stressed scene, the other way round.
    pas, _ = _pas(topics=_live(BASE_ID + "StressNight"))
    cam = pn.ProjectAirSimCamera(T.FakeObs(), rail.write_attach_configs(_tmp()),
                                 projectairsim=pas, attach=True)
    _raises(cam.start, contains="serves no topic under")
    # attach with --stress rain to the rain scene whose rain is off.
    pas, _ = _pas(topics=_live(BASE_ID + "StressRain"))
    cam = pn.ProjectAirSimCamera(T.FakeObs(), rail.write_attach_configs(_tmp(), stress="rain"),
                                 projectairsim=pas, attach=True, stress="rain")
    _raises(cam.start, contains="-Stress rain")
    _raises(pn.ProjectAirSimCamera, T.FakeObs(), orin, stress="dawn", contains="no perception")


def test_the_scene_step_renders_sets_and_records_the_stressor():
    out = _tmp()
    args = pn.build_parser().parse_args(["--scene-only", "--network", "mirrored",
                                         "--out-root", str(out), "--tag", "s",
                                         "--vla-cameras", "--stress", "rain"])
    cfg, _ = pn.sim_configs(args, _tmp())
    pas, client = _pas()
    loader = pn.SceneLoader(cfg, projectairsim=pas, stress="rain")
    assert pn.load_scene_only(args, out / "s", loader) == 0
    rec = json.loads((out / "s" / "scene_load.json").read_text(encoding="utf-8"))
    assert rec["scene"]["scene_id"] == BASE_ID + "StressRain"
    assert rec["scene"]["stress"]["weather"] == {"RAIN": 0.8}
    assert rec["stress"]["name"] == "rain" and rec["stress"]["weather"]["ok"]
    assert rec["stress"]["attach_with"].startswith("--stress rain")
    assert client.weather.calls == ["reset", "enable", ("set", 1, 0.8), "get"]
    # A weather that cannot be set refuses the scene step (exit before SITL).
    pas, _ = _pas(weather=WeatherSim(enable_ok=False))
    loader = pn.SceneLoader(cfg, projectairsim=pas, stress="rain")
    _raises(pn.load_scene_only, args, _tmp(), loader, contains="the weather was not set")
    # Without --stress the scene step is as before: no weather call, no record.
    args = pn.build_parser().parse_args(["--scene-only", "--network", "mirrored",
                                         "--out-root", str(out), "--tag", "p"])
    cfg, _ = pn.sim_configs(args, _tmp())
    pas, client = _pas()
    assert pn.load_scene_only(args, out / "p", pn.SceneLoader(cfg, projectairsim=pas)) == 0
    rec = json.loads((out / "p" / "scene_load.json").read_text(encoding="utf-8"))
    assert "stress" not in rec and "stress" not in rec["scene"]
    assert client.weather.calls == []


def test_the_dry_run_shows_the_rendered_stressor():
    old = pn.module_available
    try:
        pn.module_available = lambda n: True
        args = pn.build_parser().parse_args(["--dry-run", "--network", "mirrored",
                                             "--dev-yaml", "--stress", "dusk"])
        rep = pn.dry_run(args, _tmp())
        assert rep["ok"] and rep["sim_config"]["scene_id"] == BASE_ID + "StressDusk"
        assert rep["sim_config"]["stress"]["exposure"]["auto_exposure_bias_ev"] == -2.0
    finally:
        pn.module_available = old


def test_a_stressed_flight_records_its_stressor_and_is_never_kpi_grade():
    rc, run, _ = T._fly("on", extra=("--stress", "night"))
    assert rc == 0
    m = T._json(run, "metrics.json")
    rec = m["perception_stress"]
    assert rec["name"] == "night" and rec["scenario_id"] == "pas-follow-redcar-night"
    assert rec["kpi"] == "non-KPI" and rec["rendered"]["scene_id"] == BASE_ID + "StressNight"
    assert rec["rendered"]["exposure"]["auto_exposure_bias_ev"] == -4.0
    assert m["sim_config"]["scene_id"] == BASE_ID + "StressNight"
    assert m["params"]["stress"] == "night"
    kpi = T._json(run, "kpi.json")
    assert kpi["kpi_grade"] is False
    assert ps.non_kpi_reason("night") in kpi["kpi_grade_reasons"], kpi["kpi_grade_reasons"]
    # Unstressed: no record, no such reason.
    rc, run, _ = T._fly("on")
    m = T._json(run, "metrics.json")
    assert m["perception_stress"] is None and m["params"]["stress"] is None
    assert not any("perception-stress" in r for r in T._json(run, "kpi.json")
                   ["kpi_grade_reasons"])
    # The report reads the flown run (no detections.jsonl from the stand-in).
    rm = R.run_metrics(run)
    assert rm["stress"] == R.UNSTRESSED and rm["detector"]["inferences"] is None


def test_perception_stress_record_names_who_applied_it():
    args = pn.build_parser().parse_args(["--stress", "rain", "--scene-mode", "attach",
                                         "--link", "mavros"])
    cfg = rail.write_attach_configs(_tmp(), stress="rain")
    cam = SimpleNamespace(scene_record={"stress": {"weather": {"ok": True}}})
    rec = pn.perception_stress_record(args, cfg, cam)
    assert rec["weather"] == {"ok": True} and "desktop's scene step" in rec["applied_by"]
    assert rec["rendered"]["weather"] == {"RAIN": 0.8}
    assert pn.perception_stress_record(pn.build_parser().parse_args([]), cfg, cam) is None
    # A camera with no record (a stand-in) leaves the weather None, never ok.
    assert pn.perception_stress_record(args, cfg, object())["weather"] is None


# --------------------------------------------------------------------------- #
# The launchers
# --------------------------------------------------------------------------- #

def _bash() -> str | None:
    if os.name == "nt":
        for cand in (r"C:\Program Files\Git\bin\bash.exe",
                     r"C:\Program Files\Git\usr\bin\bash.exe"):
            if Path(cand).is_file():
                return cand
        return None                      # never System32\bash.exe: that starts WSL
    return shutil.which("bash")


def _orin(env: dict) -> subprocess.CompletedProcess:
    g1 = _tmp() / "g1_verdict.json"
    g1.write_text("{}", encoding="utf-8")
    full = {k: v for k, v in os.environ.items()
            if k not in ("VLAGUARD_DRY_RUN", "G1_VERDICT", "TAG", "BUNDLE", "VLA_PORT",
                         "FRONT_CAPTURE_INTERVAL_S", "SECONDS_", "STRESS")}
    full.update({"VLAGUARD_DRY_RUN": "1", "G1_VERDICT": g1.as_posix(), **env})
    return subprocess.run([_bash(), (ROOT / "deploy" / "orin" / "run_pas_perception.sh")
                           .as_posix()], env=full, capture_output=True, text=True, timeout=60)


def test_the_orin_script_passes_stress_on_and_names_the_run_after_it():
    if _bash() is None:
        return SKIP
    import shlex
    r = _orin({"STRESS": "night"})
    assert r.returncode == 0, r.stderr
    line = next(l for l in r.stdout.splitlines() if l.startswith("EXEC:"))
    argv = shlex.split(line[len("EXEC:"):])
    i = argv.index("--stress")
    assert argv[i + 1] == "night", argv
    tag = argv[argv.index("--tag") + 1]
    assert tag.startswith("pas_hil_aiot_night_"), tag
    # The node's own parser takes it beside the hil flags.
    a = pn.build_parser().parse_args(argv[2:])
    pn.check_args(a)
    assert a.stress == "night" and a.scene_mode == "attach"
    r = _orin({"STRESS": "dawn"})
    assert r.returncode == 64 and "STRESS 'dawn'" in r.stderr and "EXEC:" not in r.stdout
    r = _orin({})
    assert r.returncode == 0 and "--stress" not in r.stdout, r.stdout


def test_the_desktop_launcher_passes_stress_to_the_scene_node_and_check_steps():
    src = (ROOT / "scripts" / "run_pas_ardupilot.ps1").read_text(encoding="utf-8")
    assert '[ValidateSet("", "day", "dusk", "night", "rain", "fog")]' in src
    assert '$stressArgs = @("--stress", $Stress)' in src
    scene = src[src.index('"scene" {'):src.index('"sitl" {')]
    node = src[src.index('"node" {'):src.index('"g1" {')]
    check = src[src.index('if ($Step -eq "check")'):src.index('if ($Step -eq "g1")')]
    assert "$sceneExtra += $stressArgs" in scene
    assert "$a += $stressArgs" in node and "@stressArgs" in check
    assert '$Step -notin @("scene", "node", "check")' in src
    # The table and the launcher name the same stressors.
    assert all(f'"{n}"' in src for n in ps.NAMES)
    pwsh = shutil.which("pwsh") or shutil.which("powershell")
    if pwsh is None:
        return SKIP
    ps1 = (ROOT / "scripts" / "run_pas_ardupilot.ps1").as_posix()
    cmd = ("$e = $null; $null = [System.Management.Automation.Language.Parser]::ParseFile("
           f"'{ps1}', [ref]$null, [ref]$e); $e.Count")
    r = subprocess.run([pwsh, "-NoProfile", "-NonInteractive", "-Command", cmd],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and r.stdout.strip().splitlines()[-1] == "0", (r.stdout, r.stderr)


# --------------------------------------------------------------------------- #
# The detector's luma and the per-scenario metric
# --------------------------------------------------------------------------- #

def test_the_detector_logs_its_frames_mean_grey_level():
    import follow_vlm as fv
    # The detector's log line carries it.
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    assert '"luma": frame_luma(img),' in src
    assert fv.frame_luma(None) is None and fv.frame_luma("not an image") is None
    try:
        from PIL import Image
    except Exception:                                         # noqa: BLE001
        print("      (Pillow is not installed here: the luma of a real image is not checked)")
        return SKIP
    assert fv.frame_luma(Image.new("RGB", (768, 432), (0, 0, 0))) == 0.0
    assert fv.frame_luma(Image.new("RGB", (768, 432), (255, 255, 255))) == 255.0
    mid = fv.frame_luma(Image.new("RGB", (768, 432), (128, 128, 128)))
    assert abs(mid - 128.0) < 0.6, mid
    # Half dark, half bright averages; a frame smaller than the step still reads.
    im = Image.new("RGB", (64, 32), (0, 0, 0))
    im.paste((200, 200, 200), (32, 0, 64, 32))
    assert abs(fv.frame_luma(im) - 100.0) < 1.0, fv.frame_luma(im)
    assert fv.frame_luma(Image.new("RGB", (5, 3), (90, 90, 90))) == 90.0


def _write_run(root: Path, name: str, *, stress=None, lock_pattern="", luma=110.0,
               score=0.3, hit_every=1, weather_ok=True, kpi_grade=False, truth=True):
    """A run folder as perception_node writes one, from a lock pattern: one
    row per character at 0.1 s - 'L' estimate on the subject, 'o' estimate
    off it, '.' no estimate."""
    run = root / name
    run.mkdir(parents=True)
    rows, dets = [], []
    for i, c in enumerate(lock_pattern):
        tx, ty = 100.0 + i, 50.0
        r = {"t": round(i * 0.1, 3), "tick": i, "det_seq": 10 + i,
             "subject": (None if c == "." else [tx + (1.0 if c == "L" else 40.0), ty, "s"])}
        if truth:
            r.update(subject_truth=[tx, ty], subject_range_m=20.0 + (i % 3) * 10.0,
                     subject_gap_m=(None if c == "." else (1.0 if c == "L" else 40.0)))
        else:
            r.update(subject_truth=None, subject_truth_why="unread")
        rows.append(r)
    for k in range(5, 10 + len(lock_pattern) + 5):       # before and after the mission too
        hit = (k - 10) % hit_every == 0
        dets.append({"seq": k, "t": 1.7e9 + k, "luma": luma, "switched": k == 12,
                     "det": ({"cx": 1, "cy": 1, "w": 5, "h": 5, "score": score}
                             if hit else None)})
    ps_rec = None if stress is None else {
        "name": stress, "scenario_id": ps.scenario_id(stress), "kpi": "non-KPI",
        "weather": {"ok": weather_ok, "read_back": {"RAIN": 0.0, "FOG": 0.0}}}
    (run / "metrics.json").write_text(json.dumps(
        {"perception_stress": ps_rec, "det_hz": 3.0, "end_reason": "max_s",
         "mission_s": len(rows) * 0.1,
         "sim_config": {"scene_id": BASE_ID if stress is None else
                        ps.scene_id_for(BASE_ID, stress)}}), encoding="utf-8")
    (run / "kpi.json").write_text(json.dumps({"kpi_grade": kpi_grade}), encoding="utf-8")
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    (run / "detections.jsonl").write_text("".join(json.dumps(d) + "\n" for d in dets),
                                          encoding="utf-8")
    return run


def test_the_per_run_metric_reads_what_the_run_logged():
    root = _tmp()
    # 10 locked, 12 off (1.2 s: one loss), 5 locked, 3 without an estimate
    # (0.3 s: no loss), 10 locked, then 15 off to the end (a loss still open).
    pat = "L" * 10 + "o" * 12 + "L" * 5 + "." * 3 + "L" * 10 + "o" * 15
    run = _write_run(root, "r1", stress="night", lock_pattern=pat, luma=30.0,
                     score=0.25, hit_every=2)
    m = R.run_metrics(run)
    t = m["track"]
    assert m["stress"] == "night" and m["scenario_id"] == "pas-follow-redcar-night"
    assert t["ticks"] == 55 and t["truth_ticks"] == 55
    assert t["estimate_frac"] == round(52 / 55, 4)
    assert t["lock_frac"] == round(25 / 55, 4)
    assert t["estimate_on_subject_frac"] == round(25 / 52, 4)
    assert t["track_losses"] == 2, t
    assert t["longest_loss_s"] == 1.4, t                # 40 -> 54 is 1.4 s; 10 -> 22 is 1.2
    assert t["estimate_dropouts"] == 0                    # 0.3 s without one is no dropout
    assert t["within_30m_frac"] == round(sum(1 for i in range(55) if i % 3 != 2) / 55, 4)
    assert t["range_median_m"] == 30.0
    d = m["detector"]
    # The mission's inferences only: seq 10..64, not the 5 before or after.
    assert d["inferences"] == 55 and d["window"].startswith("seq 10-64")
    assert d["hits"] == 28 and d["det_hit_rate"] == round(28 / 55, 4)
    assert d["det_conf_mean"] == 0.25 and d["det_switches"] == 1
    assert d["luma_mean"] == 30.0 and d["det_hz"] == 3.0
    assert m["weather_ok"] is True and m["kpi_grade"] is False
    assert m["scene_id"] == BASE_ID + "StressNight"
    # Never locked: no loss count (nothing was lost), the dropouts still counted.
    m = R.run_metrics(_write_run(root, "r2", lock_pattern="o" * 20 + "." * 15 + "o" * 5))
    assert m["track"]["lock_frac"] == 0.0 and m["track"]["track_losses"] is None
    assert m["track"]["estimate_dropouts"] == 1 and m["stress"] == R.UNSTRESSED
    # No truth read: the lock is not measured (None), never 0.
    m = R.run_metrics(_write_run(root, "r3", lock_pattern="L" * 20, truth=False))
    assert m["track"]["lock_frac"] is None and m["track"]["truth_ticks"] == 0
    assert m["track"]["within_30m_frac"] is None and m["track"]["estimate_frac"] == 1.0
    _raises(R.run_metrics, root / "nothing", contains="metrics.json")


def test_the_scenario_table_compares_each_stressor_with_the_day():
    root = _tmp()
    runs = [R.run_metrics(_write_run(root, "d1", stress="day", lock_pattern="L" * 30,
                                     luma=120.0, score=0.4)),
            R.run_metrics(_write_run(root, "d2", stress="day", lock_pattern="L" * 20 + "o" * 10,
                                     luma=110.0, score=0.4)),
            R.run_metrics(_write_run(root, "n1", stress="night", lock_pattern="L" * 10 + "o" * 20,
                                     luma=30.0, score=0.1, hit_every=3)),
            # A "night" run whose frames were as bright as the day's.
            R.run_metrics(_write_run(root, "k1", stress="dusk", lock_pattern="L" * 30,
                                     luma=112.0)),
            R.run_metrics(_write_run(root, "f1", stress="fog", lock_pattern="L" * 30,
                                     weather_ok=False)),
            R.run_metrics(_write_run(root, "x1", stress="rain", lock_pattern="L" * 30,
                                     kpi_grade=True))]
    tab = R.scenario_table(runs)
    assert tab["baseline"] == "day" and "non-KPI" in tab["label"]
    by = {s["stress"]: s for s in tab["scenarios"]}
    assert [s["stress"] for s in tab["scenarios"]] == ["day", "dusk", "night", "rain", "fog"]
    day, night = by["day"], by["night"]
    assert day["runs"] == 2 and day["scenario_id"] == "pas-follow-redcar-day"
    # The mean of the runs' own (4-decimal) figures: 1.0 and 0.6667.
    assert abs(day["mean"]["lock_frac"] - (1.0 + 20 / 30) / 2) < 1e-4, day["mean"]
    assert day["mean"]["luma_mean"] == 115.0 and day["delta_vs_baseline"] is None
    assert night["mean"]["lock_frac"] == round(10 / 30, 4)
    assert night["delta_vs_baseline"]["lock_frac"] == \
        round(night["mean"]["lock_frac"] - day["mean"]["lock_frac"], 4)
    assert night["delta_vs_baseline"]["lock_frac"] < -0.4
    assert night["delta_vs_baseline"]["luma_mean"] == -85.0
    assert night["checks"] == [], night["checks"]
    assert any("not darker than the baseline" in c for c in by["dusk"]["checks"])
    assert any("weather not verified" in c and "f1" in c for c in by["fog"]["checks"])
    assert any("KPI-graded although stressed" in c for c in by["rain"]["checks"])
    md = R.to_markdown(tab)
    assert "| pas-follow-redcar-night | 1 |" in md and "Checks:" in md
    assert md.count("\n| pas-follow-redcar-") == 5
    # Without a day scenario the unstressed runs are the baseline.
    tab = R.scenario_table([runs[2], R.run_metrics(_write_run(root, "u1", lock_pattern="L" * 30))])
    assert tab["baseline"] == R.UNSTRESSED
    assert {s["stress"]: s["baseline"] for s in tab["scenarios"]} == \
        {"night": R.UNSTRESSED, R.UNSTRESSED: None}


def test_the_report_cli_writes_json_and_markdown_and_skips_a_folder_without_metrics():
    root = _tmp()
    a = _write_run(root, "a", stress="day", lock_pattern="L" * 10)
    b = _write_run(root, "b", stress="rain", lock_pattern="L" * 5 + "o" * 5)
    (root / "empty").mkdir()
    js, md = root / "t.json", root / "t.md"
    assert R.main([str(a), str(b), str(root / "empty"), "--json", str(js), "--md", str(md)]) == 0
    tab = json.loads(js.read_text(encoding="utf-8"))
    assert [s["stress"] for s in tab["scenarios"]] == ["day", "rain"]
    assert "pas-follow-redcar-rain" in md.read_text(encoding="utf-8")
    assert b"\r\n" not in md.read_bytes()
    assert R.main([str(root / "empty")]) == 2
    # The tool's constants are the node's.
    assert (R.ESTIMATE_NEAR_M, R.FOLLOW_RANGE_M, R.LOSS_MIN_S) == \
        (pn.ESTIMATE_NEAR_M, pn.FOLLOW_RANGE_M, pn.DETECTION_STALE_S)


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
