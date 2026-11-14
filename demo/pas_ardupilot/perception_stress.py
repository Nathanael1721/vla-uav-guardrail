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

  * THE SUN IS THE LEVEL'S OWN DirectionalLight_0, TURNED (shadow, since
    2026-11-15). World.set_object_pose (world.py:426; WorldSimApi::
    setObjectPose: SetActorLocationAndRotation on any actor found by name)
    turns it, and World.get_object_pose reads it back, so this stressor -
    unlike the texture - is checked by an attaching node too. Measured on
    15 Nov (the Chase camera, CityLife_Day, ArduPilot ticking): turned from
    the level's 49.5 deg below the horizontal to 8 deg, and 60 deg round,
    the frame's mean grey level fell from 98.5 to 60.8 and came back to
    100.8 when the light was put back. Every stressor reads the light back
    and refuses a run whose sun is not its own (a shadow scene's light left
    turned in a simulator that was not restarted would otherwise darken the
    next "day").
  * TIME OF DAY CANNOT BE SET IN CityLife_Day. World.set_time_of_day
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

  * NOVEL TEXTURES ARE THE LEVEL'S OWN ACTORS RE-SKINNED (texture, since
    2026-11-15). World.list_objects(regex) (world.py:351; std::regex_match
    on each actor's GetName, UnrealHelpers::ListMatchingActors) picks the
    actors, and World.set_object_texture_from_file (world.py:803) gives
    every static mesh component of each one a dynamic instance of the
    plugin's RuntimeMeshMaterial with the PNG as its base colour
    (WorldSimApi::setObjectTexture). The PNG is written here (texture_png:
    yellow / black diagonal stripes, stdlib only), so the stressor is the
    same bytes on every run (png_sha256 in the record). Nothing reads a
    texture back: the record holds each call's answer, and an attaching
    node trusts the desktop's scene step for it, bound by the scene id as
    the exposure is.

WHAT IS NOT HERE
  * sensor noise (the robot config's noise-settings): not rendered;
  * combinations (night + rain): one stressor per run, so a metric's change
    has one cause.

Stdlib only, like rail.py: WSL's and the Orin's python3 import it without
the project's environments.
"""
from __future__ import annotations

import copy
import hashlib
import math
import os
import struct
import tempfile
import zlib

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

# The texture stressor (module docstring, NOVEL TEXTURES): which actors
# (an ECMAScript regex matched whole against each actor's name, so no inline
# flags), at most how many, and the PNG.
TEXTURE_OBJECTS = r".*[Rr]oad.*"
TEXTURE_MAX_OBJECTS = 400
TEXTURE_SIZE_PX = 64
TEXTURE_STRIPE_PX = 8
TEXTURE_COLOURS = ((255, 210, 0), (16, 16, 16))
TEXTURE_FILE = "pas_stress_texture.png"

# The sun (module docstring, THE SUN IS THE LEVEL'S OWN DirectionalLight_0):
# the actor, its (roll, pitch, yaw) in the level as read back on 15 Nov
# (World.get_object_pose, NED, ZYX, degrees), and how near a read-back must
# be. Only pitch and yaw aim a directional light; its roll is never changed.
SUN_ACTOR = "DirectionalLight_0"
LEVEL_SUN_RPY_DEG = (112.361, -49.517, -10.309)
SUN_TOL_DEG = 0.5

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
    "shadow": {
        "exposure_bias_ev": None, "weather": {},
        "sun": {"pitch_deg": -8.0, "yaw_deg": 49.691},
        "what": "the level's sun turned to 8 deg above the horizon and 60 deg "
                "round from the day's: low light, long shadows from another side"},
    "texture": {
        "exposure_bias_ev": None, "weather": {},
        "texture": {"objects": TEXTURE_OBJECTS, "pattern": "yellow/black diagonal stripes"},
        "what": "the level's road actors re-skinned with yellow / black diagonal "
                "stripes (novel textures)"},
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

SUN_NOTE = ("no time of day: CityLife_Day has no SunSky actor, so Project "
            "AirSim's time-of-day and sun calls are no-ops there ('No SunSky objects "
            "found', projectairsim_server.log); dusk and night are the cameras' "
            "exposure (auto-exposure-bias); the shadow stressor turns the level's "
            "DirectionalLight_0 itself (set_object_pose), and every stressor reads "
            "it back")


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


# --------------------------------------------------------------------------- #
# Novel textures (module docstring, NOVEL TEXTURES)
# --------------------------------------------------------------------------- #

def texture_png(size: int = TEXTURE_SIZE_PX, stripe: int = TEXTURE_STRIPE_PX,
                colours=TEXTURE_COLOURS) -> bytes:
    """The stressor's texture: an 8-bit RGB PNG of diagonal stripes, `stripe`
    pixels wide, in `colours` (stdlib only: zlib and struct)."""
    if size < 1 or stripe < 1:
        raise ValueError("a texture needs a positive size and stripe width")
    on, off = bytes(colours[0]), bytes(colours[1])
    raw = bytearray()
    for y in range(size):
        raw.append(0)                                         # filter: none
        for x in range(size):
            raw += on if ((x + y) // stripe) % 2 == 0 else off

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))


def apply_texture(world, name: str, png_dir: str | None = None) -> dict:
    """Re-skin the stressor's actors on a loaded scene's World (module
    docstring, NOVEL TEXTURES). Never raises: the record's `problems` says
    what failed (no actor matched, none took the texture), and the caller
    refuses the run on any. A stressor without a texture answers ok with
    mode "none"."""
    spec = stressor(name).get("texture")
    if not spec:
        return {"mode": "none", "ok": True, "problems": []}
    problems: list[str] = []
    png = texture_png()
    rec = {"mode": "set", "objects_regex": spec["objects"], "pattern": spec["pattern"],
           "png_sha256": "sha256:" + hashlib.sha256(png).hexdigest(),
           "png_size_px": TEXTURE_SIZE_PX, "stripe_px": TEXTURE_STRIPE_PX,
           "read_back": "none: Project AirSim has no call that reads a texture back"}
    found, err = _status(world.list_objects, spec["objects"])
    if err is not None:
        problems.append(f"list_objects({spec['objects']!r}) failed ({err})")
        found = []
    names = sorted({str(n) for n in (found or [])})
    chosen = names[:TEXTURE_MAX_OBJECTS]
    rec.update(matched=len(names), asked=len(chosen),
               dropped_over_cap=len(names) - len(chosen), sample=chosen[:10])
    path = os.path.join(png_dir or tempfile.gettempdir(), TEXTURE_FILE)
    n_set, failed = 0, []
    if chosen:
        with open(path, "wb") as fh:
            fh.write(png)
        for obj in chosen:
            ans, err = _status(world.set_object_texture_from_file, obj, path)
            if ans is True:
                n_set += 1
            else:
                failed.append({"object": obj, "answer": ans, "error": err})
    rec.update(set=n_set, n_failed=len(failed), failed=failed[:20])
    if err is None and not names:
        problems.append(f"no actor's name matches {spec['objects']!r}")
    elif chosen and not n_set:
        problems.append(f"none of the {len(chosen)} matched actor(s) took the texture")
    rec.update(ok=not problems, problems=problems)
    return rec


def texture_attached(name: str) -> dict:
    """The record of an attaching node: the texture was set by the desktop's
    scene step, which this node cannot read back (the scene id binds it)."""
    spec = stressor(name).get("texture")
    if not spec:
        return {"mode": "none", "ok": True, "problems": []}
    return {"mode": "attached (set by the desktop's scene step; no read-back call)",
            "objects_regex": spec["objects"], "pattern": spec["pattern"],
            "png_sha256": "sha256:" + hashlib.sha256(texture_png()).hexdigest(),
            "ok": True, "problems": []}


# --------------------------------------------------------------------------- #
# The sun (module docstring, THE SUN IS THE LEVEL'S OWN DirectionalLight_0)
# --------------------------------------------------------------------------- #

def quaternion_to_rpy_deg(w: float, x: float, y: float, z: float) -> tuple:
    """(roll, pitch, yaw) in degrees, ZYX, as projectairsim.utils.
    quaternion_to_rpy (utils.py:260) computes them in radians."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def rpy_deg_to_quaternion(roll: float, pitch: float, yaw: float) -> dict:
    """{"w", "x", "y", "z"} for (roll, pitch, yaw) in degrees, ZYX, as
    projectairsim.utils.rpy_to_quaternion (utils.py:296)."""
    r, p, y = (math.radians(a) * 0.5 for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return {"w": cy * cr * cp + sy * sr * sp, "x": cy * sr * cp - sy * cr * sp,
            "y": cy * cr * sp + sy * sr * cp, "z": sy * cr * cp - cy * sr * sp}


def _field(o, k):
    return o[k] if isinstance(o, dict) else getattr(o, k)


def _angle_off(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def sun_wanted(name: str) -> tuple:
    """(pitch, yaw) in degrees the stressor's sun must read back at: the
    shadow stressor's, or the level's own for every other stressor."""
    sun = stressor(name).get("sun")
    if sun:
        return float(sun["pitch_deg"]), float(sun["yaw_deg"])
    return LEVEL_SUN_RPY_DEG[1], LEVEL_SUN_RPY_DEG[2]


def _read_sun(world, problems: list) -> dict | None:
    pose, err = _status(world.get_object_pose, SUN_ACTOR)
    if err is not None:
        problems.append(f"get_object_pose({SUN_ACTOR!r}) failed ({err})")
        return None
    try:
        q = _field(pose, "rotation")
        t = _field(pose, "translation")
        rpy = quaternion_to_rpy_deg(*(float(_field(q, k)) for k in "wxyz"))
        return {"rpy_deg": [round(a, 3) for a in rpy],
                "translation": {k: float(_field(t, k)) for k in "xyz"}}
    except (KeyError, AttributeError, TypeError, ValueError) as exc:
        problems.append(f"{SUN_ACTOR}'s pose could not be read ({type(exc).__name__}: {exc})")
        return None


def _verify_sun(read: dict | None, name: str, problems: list) -> None:
    if read is None:
        return
    pitch, yaw = sun_wanted(name)
    got_p, got_y = read["rpy_deg"][1], read["rpy_deg"][2]
    if _angle_off(got_p, pitch) > SUN_TOL_DEG or _angle_off(got_y, yaw) > SUN_TOL_DEG:
        whose = "the shadow stressor's" if stressor(name).get("sun") else "the level's own"
        problems.append(f"{SUN_ACTOR} reads back pitch {got_p:.1f} yaw {got_y:.1f} deg, not "
                        f"{whose} {pitch:.1f} / {yaw:.1f} (a simulator left turned by an "
                        f"earlier shadow scene is restarted, not reused)")


def apply_sun(world, name: str) -> dict:
    """Turn the level's sun for the shadow stressor (its roll and position
    kept) and read it back; for every other stressor only read it back. Never
    raises: the record's `problems` says what failed."""
    problems: list[str] = []
    sun = stressor(name).get("sun")
    before = _read_sun(world, problems)
    calls = []
    if sun and before is not None:
        roll = before["rpy_deg"][0]
        pose = {"frame_id": "DEFAULT_ID", "translation": before["translation"],
                "rotation": rpy_deg_to_quaternion(roll, float(sun["pitch_deg"]),
                                                  float(sun["yaw_deg"]))}
        ans, err = _status(world.set_object_pose, SUN_ACTOR, pose, True)
        calls.append({"call": f"set_object_pose({SUN_ACTOR!r})", "answer": ans, "error": err})
        if err is not None:
            problems.append(f"set_object_pose({SUN_ACTOR!r}) failed ({err})")
        elif ans is not True:
            problems.append(f"set_object_pose({SUN_ACTOR!r}) answered {ans!r}")
    read = _read_sun(world, problems) if sun else before
    _verify_sun(read, name, problems)
    return {"mode": "set" if sun else "checked", "actor": SUN_ACTOR,
            "wanted_pitch_yaw_deg": list(sun_wanted(name)), "before": before,
            "calls": calls, "read_back": read, "ok": not problems, "problems": problems}


def check_sun(world, name: str) -> dict:
    """Read the sun of a scene this node only attached to and say whether
    it is the stressor's."""
    problems: list[str] = []
    read = _read_sun(world, problems)
    _verify_sun(read, name, problems)
    return {"mode": "checked (attached; set by the desktop's scene step)",
            "actor": SUN_ACTOR, "wanted_pitch_yaw_deg": list(sun_wanted(name)),
            "read_back": read, "ok": not problems, "problems": problems}
