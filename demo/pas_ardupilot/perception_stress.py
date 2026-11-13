"""Perception stressors for the Project AirSim + ArduPilot rail (WP4-16, WP4-18).

The grant's Stress Testing page names Project AirSim as the rail for
perception stress ("texture / lighting / shadow stressors", lighting,
weather, novel textures, dynamic actors). CityLife already brings the dynamic
actors (40 pedestrians, 24 cars, signals). This module adds a lighting and a
weather axis to the follow mission flown on this rail, one stressor per run:

    day     the baseline of the stress set: the weather explicitly reset
    dusk    every scene capture exposed 2 EV down (a quarter of the light)
    night   every scene capture exposed 4 EV down (a sixteenth)
    rain    the plugin's weather effects on, RAIN 0.8
    fog     the plugin's weather effects on, FOG 0.6

A stressed run is perception-stress evidence, never a KPI number: the node
adds NON_KPI_REASON to kpi.json's kpi_grade_reasons whenever --stress is
given ("day" included - it is the stress set's own baseline).

HOW EACH ONE IS APPLIED, READ FROM THE SOURCE ON THIS DESKTOP (2026-11-13)

Client: projectairsim 0.2.0, editable install at
D:/ProjectAirSim/repo/client/python/projectairsim/src/projectairsim.
Server: PASBlocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private.

  * THE SUN CANNOT BE MOVED IN CityLife_Day. World.set_time_of_day
    (world.py:933), set_sun_position_from_date_time (world.py:984),
    set_sunlight_intensity (world.py:128) and set_cloud_shadow_strength
    (world.py:159) all act on an actor named "SunSky"
    (World/TimeofDay.cpp TimeOfDay::initialize, FindActor(world, "SunSky")).
    CityLife_Day has none: the simulator logged "No SunSky objects found.
    TimeOfDay settings would ignored/not applied" when it loaded
    SceneCityLifeArduPilot on 2026-11-13 (PASBlocks/projectairsim_server.log).
    Then set_time_of_day answers TRUE and changes nothing (TimeOfDay::set
    returns tod_enabled_ == is_enabled), set_sunlight_intensity answers
    false, and set_sun_position_from_date_time would dereference the null
    SunSky (TimeOfDay::SetDateAndTime -> SetIntPropertyValue on sun_sky_).
    None of them is called here. set_light_object_intensity (world.py:872)
    finds only Project AirSim's own ALightActorBase actors
    (WorldSimApi.cpp:1260), never the level's DirectionalLight.
  * SO DUSK AND NIGHT ARE THE CAMERAS' EXPOSURE. The robot config's
    capture-settings key "auto-exposure-bias" is read by the simulator
    (core_sim CaptureSettings.auto_exposure_bias, core_sim/sensors/camera.hpp)
    and set as the scene capture's PostProcessSettings.AutoExposureBias
    (Sensors/UnrealCamera.cpp:467-470, UpdateCameraPostProcessingSetting):
    Unreal's exposure compensation in EV, each -1 halving the light. It is
    rendered into every image-type 0 (scene) capture of every camera -
    FrontCamera (the detector's, and the VLA's front frame with
    --vla-front-source detector), VLAFront, VLADown and Chase (the video's
    third-person view) - so the world looks the same to all of them. Depth
    captures are untouched. This is darkness AT THE SENSOR: the level's
    street lamps and car lights do not come on, the shadows stay the day's.
    The client's JSON schema does not list the key (robot_config_schema.jsonc
    capture-settings) but sets no additionalProperties, so the loader
    accepts it (tests/test_pas_perception_stress.py loads one).
  * RAIN AND FOG ARE THE PLUGIN'S WEATHER EFFECTS. At every scene load the
    simulator spawns its WeatherActor 5 m above each robot and resets the
    weather parameters (World/WeatherLib.cpp UWeatherLib::initWeather, from
    UnrealScene.cpp:369); the parameters are a material parameter collection
    (/ProjectAirSim/Weather/WeatherFX/WeatherGlobalParams) the actor's
    particles read. The calls, in this order:
        World.reset_weather_effects()             world.py:711
        World.enable_weather_visual_effects()     world.py:683
        World.set_weather_visual_effects_param(WeatherParameter.RAIN, 0.8)
                                                  world.py:725, types.py:35-56
        World.get_weather_visual_effects_param()  world.py:743 (read back)
    A parameter set while the effects are disabled is written as 0 and
    answers false (WeatherLib.cpp setWeatherParamScalar), so the effects are
    enabled first and every value is read back. Day, dusk and night reset
    the weather and disable the effects (World.disable_weather_visual_
    effects, world.py:697), and read back that no parameter a weather
    stressor sets is above WEATHER_TOL.
  * WHICH SCENE A NODE ATTACHES TO. A stressed scene is rendered under its
    own scene id (scene_id_for: SceneCityLifeArduPilotStressNight, ...). The
    attaching node on the AGX Orin builds its World on that id and refuses
    when the simulator serves no topic under it (perception_node
    ProjectAirSimCamera._attach_world), so a node told --stress night cannot
    fly a scene the desktop loaded without -Stress night, nor the other way
    round - the exposure, which no RPC reads back, is bound by the scene id.
    The weather it reads back as well (check_weather).

WHAT IS NOT HERE
  * novel textures (World.set_object_material, swap_object_texture,
    world.py:765-871): the level's actor names and the packaged materials
    are not enumerated yet; WP4-18 keeps that part open;
  * sensor noise (the robot config's noise-settings): not rendered;
  * combinations (night + rain): one stressor per run, so a metric's change
    has one cause.

Stdlib only, like rail.py: WSL's and the Orin's python3 import it without
the project's environments.
"""
from __future__ import annotations

import copy
import math

# The base scene id of the template (scene_citylife_ardupilot.jsonc). A
# stressed scene is rendered as BASE + "Stress" + Name.
STRESS_ID_INFIX = "Stress"

# projectairsim.types.WeatherParameter (types.py:35-56), mirrored so this
# module needs no projectairsim (tests/test_pas_perception_stress.py checks
# the mirror against the installed client).
WEATHER_PARAMS = {"ENABLED": 0, "RAIN": 1, "ROAD_WETNESS": 2, "SNOW": 3,
                  "ROAD_SNOW": 4, "MAPLE_LEAF": 5, "ROAD_LEAF": 6, "DUST": 7,
                  "FOG": 8}
WEATHER_NAMES = {v: k for k, v in WEATHER_PARAMS.items()}
# A read-back within this of the value set is the value set (the collection
# stores float32: 0.8 reads back 0.800000011920929).
WEATHER_TOL = 1e-3
# The scene capture's image type (robot_config_schema.jsonc capture-settings
# image-type: 0 scene, 2 depth planar, ...).
SCENE_IMAGE_TYPE = 0

# One table: what each stressor does. exposure_bias_ev None leaves the
# cameras as the template has them; weather {} resets and disables the
# weather effects.
STRESSORS: dict[str, dict] = {
    "day": {
        "exposure_bias_ev": None, "weather": {},
        "what": "the stress set's baseline: the template's cameras, the weather reset"},
    "dusk": {
        "exposure_bias_ev": -2.0, "weather": {},
        "what": "every scene capture exposed 2 EV down (1/4 of the light)"},
    "night": {
        "exposure_bias_ev": -4.0, "weather": {},
        "what": "every scene capture exposed 4 EV down (1/16 of the light)"},
    "rain": {
        "exposure_bias_ev": None, "weather": {"RAIN": 0.8},
        "what": "the plugin's weather effects on, RAIN 0.8"},
    "fog": {
        "exposure_bias_ev": None, "weather": {"FOG": 0.6},
        "what": "the plugin's weather effects on, FOG 0.6"},
}
NAMES = tuple(STRESSORS)

# Every weather parameter some stressor sets: what an unweathered run reads
# back as off. Only these, never all eight: the collection's defaults for the
# others are the asset's, not known to be 0.
WEATHER_CHECKED = tuple(sorted({p for s in STRESSORS.values() for p in s["weather"]},
                               key=lambda p: WEATHER_PARAMS[p]))

# The suite's names for the stress scenarios of the PAS follow mission (the
# red car in CityLife, the AIOT-VLA pilot on the AGX Orin; WP4-16). Not a
# ScenarioSpec family: guardrail/scenario_spec.py's Stress model is the
# headless harness's and its library is flown by experiments/
# sweep_scenarios.py on the headless and hil rails, which have no camera.
SCENARIO_PREFIX = "pas-follow-redcar-"

NON_KPI_REASON = ("a perception-stress run (--stress {name}, scenario {scenario}): "
                  "Project AirSim is the grant's perception-stress rail, and its runs "
                  "are reported per scenario as non-KPI evidence (WP4-16, WP4-18)")

SUN_NOTE = ("the sun is not moved: CityLife_Day has no SunSky actor, so Project "
            "AirSim's time-of-day and sun calls are no-ops there ('No SunSky objects "
            "found', projectairsim_server.log); dusk and night are the cameras' "
            "exposure (auto-exposure-bias)")


def stressor(name: str) -> dict:
    """The table's entry for `name`; ValueError naming the choices."""
    if name not in STRESSORS:
        raise ValueError(f"no perception stressor {name!r}; the choices are "
                         f"{', '.join(NAMES)}")
    return STRESSORS[name]


def scenario_id(name: str) -> str:
    stressor(name)
    return SCENARIO_PREFIX + name


def scene_id_for(base_id: str, name: str) -> str:
    """The scene id a stressed scene is rendered under (module docstring,
    WHICH SCENE A NODE ATTACHES TO)."""
    stressor(name)
    return f"{base_id}{STRESS_ID_INFIX}{name.capitalize()}"


def render_cameras(robot: dict, name: str) -> list[str]:
    """Write the stressor's exposure into every scene capture of every camera
    of `robot`, in place. Returns the "<camera>/<image-type>" captures it set
    (empty for a stressor that leaves the cameras alone). ValueError when the
    stressor has an exposure and the config has no scene capture to take it."""
    ev = stressor(name)["exposure_bias_ev"]
    if ev is None:
        return []
    touched = []
    for s in robot.get("sensors", []):
        if s.get("type") != "camera":
            continue
        for cs in s.get("capture-settings", []):
            if cs.get("image-type") == SCENE_IMAGE_TYPE:
                cs["auto-exposure-bias"] = float(ev)
                touched.append(f"{s.get('id')}/{SCENE_IMAGE_TYPE}")
    if not touched:
        raise ValueError(f"--stress {name}: the robot config has no scene capture "
                         f"to expose {ev:+g} EV")
    return touched


def render(robot: dict, scene: dict, name: str) -> tuple[dict, dict, dict]:
    """(robot, scene, record): copies of the rendered configs with the
    stressor written in (the cameras' exposure, the scene's id), and what was
    written, for the run's sim_config record."""
    st = stressor(name)
    robot, scene = copy.deepcopy(robot), copy.deepcopy(scene)
    touched = render_cameras(robot, name)
    base = scene.get("id")
    if not base:
        raise ValueError("the scene template has no id to render a stressed scene under")
    scene["id"] = scene_id_for(base, name)
    rec = {"name": name, "scenario_id": scenario_id(name), "what": st["what"],
           "scene_id": scene["id"], "base_scene_id": base,
           "exposure": {"auto_exposure_bias_ev": st["exposure_bias_ev"],
                        "captures": touched},
           "weather": dict(st["weather"]), "sun": SUN_NOTE, "kpi": "non-KPI"}
    return robot, scene, rec


def _status(fn, *a):
    """(answer, None) of a simulator call, or (None, "<Error>: why")."""
    try:
        return fn(*a), None
    except Exception as exc:                                  # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}"


def named_read_back(raw) -> dict | None:
    """World.get_weather_visual_effects_param()'s {int: float} as
    {"RAIN": 0.8, ...} (JSON-safe); None for an answer that is no mapping."""
    if not isinstance(raw, dict):
        return None
    out = {}
    for k, v in raw.items():
        try:
            key = WEATHER_NAMES.get(int(k), str(k))
        except (TypeError, ValueError):
            key = str(k)
        try:
            out[key] = round(float(v), 6)
        except (TypeError, ValueError):
            out[key] = v
    return out


def verify_weather(read_back: dict | None, name: str) -> list[str]:
    """What is wrong with a named read-back (named_read_back) for stressor
    `name`: each parameter it sets at its value, and, for a stressor with no
    weather, no WEATHER_CHECKED parameter above WEATHER_TOL. Empty = as asked."""
    want = stressor(name)["weather"]
    if read_back is None:
        return ["the simulator's weather parameters could not be read back"]
    problems = []
    for p, v in want.items():
        got = read_back.get(p)
        if not isinstance(got, (int, float)) or not math.isfinite(got) \
                or abs(got - v) > WEATHER_TOL:
            problems.append(f"{p} reads back {got!r}, not {v:g}")
    if not want:
        for p in WEATHER_CHECKED:
            got = read_back.get(p)
            if isinstance(got, (int, float)) and abs(got) > WEATHER_TOL:
                problems.append(f"{p} reads back {got:g}: the weather of another "
                                f"stressor is still on")
    return problems


def apply_weather(world, name: str) -> dict:
    """Set the stressor's weather on a loaded scene's World and read it back
    (module docstring, RAIN AND FOG). Never raises: the record's `problems`
    says what failed, and the caller refuses the run on any."""
    want = stressor(name)["weather"]
    calls, problems = [], []

    def call(label, fn, *a, must_be_true=True):
        ans, err = _status(fn, *a)
        calls.append({"call": label, "answer": ans, "error": err})
        if err is not None:
            problems.append(f"{label} failed ({err})")
        elif must_be_true and ans is not True:
            problems.append(f"{label} answered {ans!r}")

    # The reset answers true whatever it did (WorldSimApi::resetWeatherEffect).
    call("reset_weather_effects()", world.reset_weather_effects, must_be_true=False)
    if want:
        call("enable_weather_visual_effects()", world.enable_weather_visual_effects)
        for p, v in want.items():
            call(f"set_weather_visual_effects_param({p}={WEATHER_PARAMS[p]}, {v:g})",
                 world.set_weather_visual_effects_param, WEATHER_PARAMS[p], float(v))
    else:
        call("disable_weather_visual_effects()", world.disable_weather_visual_effects)
    raw, err = _status(world.get_weather_visual_effects_param)
    read = named_read_back(raw)
    if err is not None:
        problems.append(f"get_weather_visual_effects_param() failed ({err})")
    else:
        problems += verify_weather(read, name)
    return {"mode": "set", "asked": dict(want), "calls": calls, "read_back": read,
            "ok": not problems, "problems": problems}


def check_weather(world, name: str) -> dict:
    """Read the weather of a scene this node only attached to (the desktop's
    scene step set it) and say whether it is the stressor's."""
    raw, err = _status(world.get_weather_visual_effects_param)
    read = named_read_back(raw)
    problems = ([f"get_weather_visual_effects_param() failed ({err})"] if err is not None
                else verify_weather(read, name))
    return {"mode": "checked (attached; set by the desktop's scene step)",
            "asked": dict(stressor(name)["weather"]), "calls": [], "read_back": read,
            "ok": not problems, "problems": problems}


def non_kpi_reason(name: str) -> str:
    return NON_KPI_REASON.format(name=name, scenario=scenario_id(name))
