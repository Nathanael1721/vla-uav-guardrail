"""Light the CityLife signal heads by the fixed-time plan, instead of all at once.

    python -m tools.citylife_mcp.signals            # build/refresh the controller
    python -m tools.citylife_mcp.signals --dry-run  # print the per-head plan only

WHY. The 292 signal heads in CityLife_Day are StaticMeshActors whose lamps are
material slots (docs/data/citylife_signals.json, surveyed read-only by
inspect_signals.py). Nothing switches them, so every head shows red AND green
at once - on camera, a junction that has no state at all. The cars now obey a
fixed-time plan (tools/citylife_signals.py; UpdateEffSpeed in drive_tick.py),
and a lamp that disagrees with the traffic would be worse than none.

WHAT IT BUILDS, through the editor's MCP endpoint:

  /Game/CityLife/Blueprints/BP_SignalController   an Actor with, per head,
      Heads (StaticMeshActor), HOff (s: start of the green of the axis the head
      stands on), HRole (0 vehicle, 1 pedestrian), HG / HR (the slot that is the
      green / red lamp; equal for a single-lamp pedestrian head)
    and a 0.25 s looping timer, UpdateLamps, that computes each head's state
    from GetGameTimeInSeconds - the same arithmetic as citylife_signals.py - and
    calls SetMaterial only when a head's state changes:
      vehicle   green (G lamp Green, R lamp dark), yellow (G lamp Yellow: the
                heads have two lamp slots, so yellow shows in the green
                position), red (R lamp Red, G lamp dark)
      pedestrian walk (Green_b), flash (Green_b blinking at 1 Hz: no new
                starts), don't walk (Red_b); the other lamp dark
    "Dark" is the housing material MI_jctTrafficLightBase.
  one placed CityLife_SignalController, its arrays written from the survey.

WHICH AXIS A HEAD SHOWS. By where it stands, not which way it faces (the
mesh's forward axis is not known): heads stand ~1530 cm out along one leg of a
junction and ~830 cm to its side, so the leg is the larger offset. A vehicle
head on a leg shows the state of the traffic travelling along that leg's
road; a pedestrian head on a leg governs the zebra across that road, whose
walk window is the start of the OTHER axis' green (citylife_signals.walk).
Consistent either way round: on one leg the vehicle heads are green exactly
when the pedestrian heads say don't walk.

NOT TOUCHED. F heads (34, single lamp at the kerb corner, where the leg is
ambiguous) keep their materials. The editor-time level is unchanged: the
materials change only while the game runs.

The in-editor part is idempotent: it creates what is missing and rewrites the
graphs and arrays. Nothing is saved here; save after checking in Simulate.
"""
import json
import math
import sys
from pathlib import Path

from tools import citylife_signals as S
from tools.citylife_mcp import ue_rpc

ROOT = Path(__file__).resolve().parents[2]
SURVEY = ROOT / "docs" / "data" / "citylife_signals.json"
MATS = "/Game/JapaneseCity/Materials/Props/"
MESH_DEFAULT = {"A": ["MI_jctTrafficLightBase", "MI_jctTrafficLight_Red", "MI_jctTrafficLight_Green"],
                "B": ["MI_jctTrafficLightBase", "MI_jctTrafficLight_Green", "MI_jctTrafficLight_Red"],
                "C": ["MI_jctTrafficLightBase", "MI_jctTrafficLight_Green", "MI_jctTrafficLight_Red"],
                "D": ["MI_jctTrafficLightBase", "MI_jctTrafficLight_Green"],
                "E": ["MI_jctTrafficLightBase", "MI_jctTrafficLight_Green_b"],
                "F": ["MI_jctTrafficLightBase", "MI_jctTrafficLight_Red"]}
SKIP_KINDS = ("F",)
TIMER_S = 0.25


def slots_of(head: dict) -> list:
    """The material each slot actually shows: the level override, else the mesh's."""
    base = list(MESH_DEFAULT[head["kind"]])
    for k, m in enumerate(head.get("override_slots") or []):
        if m not in (None, "None") and k < len(base):
            base[k] = m
    return base


def leg_axis(head: dict) -> str:
    rx, ry = head["rel_cm"]
    return S.NS if abs(rx) >= abs(ry) else S.EW


def head_plan(head: dict):
    """{label, off, role, g, r} or None for a head this controller leaves alone."""
    if head["kind"] in SKIP_KINDS:
        return None
    mats = slots_of(head)
    lamps = [(k, m) for k, m in enumerate(mats) if k > 0]
    ped = any(m.endswith("_b") for _, m in lamps)
    g = [k for k, m in lamps if "Green" in m]
    r = [k for k, m in lamps if "Red" in m]
    if ped:
        if len(lamps) == 1:
            g = r = [lamps[0][0]]                   # one lamp: its material says which
        if not g or not r:
            return None
        role = 1
    else:
        if not g or not r:
            return None
        role = 0
    jx, jy = head["junction_cm"]
    axis = leg_axis(head)
    off = S.offset(jx, jy) + (S.HALF_CYCLE_S if axis == S.EW else 0.0)
    return {"label": head["label"], "off": off, "role": role, "g": g[0], "r": r[0],
            "axis": axis, "junction": [jx, jy]}


def plan(path=SURVEY) -> list:
    heads = json.loads(Path(path).read_text(encoding="utf-8"))["heads"]
    return [p for p in (head_plan(h) for h in heads) if p is not None]


def lamp_code(t: float, off: float, role: int, blink_s: float = 1.0) -> int:
    """What UpdateLamps shows, as the Blueprint computes it (C-style fmod):
    0 green, 1 yellow, 2 red; 3 walk, 4 don't walk, 5 flash-dark."""
    ph = math.fmod(t - off + 100.0, S.CYCLE_S)
    if role == 0:
        return 0 if ph < S.GREEN_S else (1 if ph < S.GREEN_S + S.YELLOW_S else 2)
    ph = math.fmod(ph + S.HALF_CYCLE_S, S.CYCLE_S)
    if ph < S.WALK_S:
        return 3
    if ph < S.GREEN_S + S.YELLOW_S:
        return 3 if math.fmod(t, blink_s) < 0.5 * blink_s else 5
    return 4


V = "Variables|Default|"


def _g(name):
    return "(%sGet%s)" % (V, name)


class _G(dict):
    def __missing__(self, k):
        return _g(k)


def _m(name):
    return '"%s%s.%s"' % (MATS, name, name)


LAMPS = """(fn UpdateLamps ()
  (Variables|Default|SetNowT (Utilities|Time|GetGameTimeInSeconds))
  (Variables|Default|SetBlink (< (Math|Float|%(Float) {NowT} 1.0) 0.5))
  (for _i (range (Utilities|Array|Length {Heads}))
    (Variables|Default|SetPh (Math|Float|%(Float) (+ (- {NowT} (Utilities|Array|Get(acopy) {HOff} _i)) 100.0) __CYCLE__))
    (if (== (Utilities|Array|Get(acopy) {HRole} _i) 0)
      (Variables|Default|SetCode (select (< {Ph} __GREEN__) 0 (select (< {Ph} __AMBER_END__) 1 2)))
      (else
        (Variables|Default|SetPh (Math|Float|%(Float) (+ {Ph} __HALF__) __CYCLE__))
        (Variables|Default|SetCode (select (< {Ph} __WALK__) 3 (select (< {Ph} __AMBER_END__) (select {Blink} 3 5) 4)))))
    (if (!= {Code} (Utilities|Array|Get(acopy) {HShown} _i))
      (Utilities|Array|SetArrayElem {HShown} _i {Code})
      (Variables|Default|SetG (Utilities|Array|Get(acopy) {HG} _i))
      (Variables|Default|SetR (Utilities|Array|Get(acopy) {HR} _i))
      (Variables|Default|SetComp (Class|StaticMeshActor|GetStaticMeshComponent :self (Utilities|Array|Get(acopy) {Heads} _i)))
      (switch int {Code}
        (:0
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {R} :Material __OFF__)
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {G} :Material __GREEN_M__))
        (:1
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {R} :Material __OFF__)
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {G} :Material __YELLOW_M__))
        (:2
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {G} :Material __OFF__)
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {R} :Material __RED_M__))
        (:3
          (if (!= {G} {R})
            (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {R} :Material __OFF__))
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {G} :Material __GREENB_M__))
        (:4
          (if (!= {G} {R})
            (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {G} :Material __OFF__))
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {R} :Material __REDB_M__))
        (:Default
          (if (!= {G} {R})
            (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {R} :Material __OFF__))
          (Rendering|Material|SetMaterial :self {Comp} :ElementIndex {G} :Material __OFF__))))))
"""

EVENTS = """(event EventBeginPlay
  (Utilities|Array|Clear {HShown})
  (for _i (range (Utilities|Array|Length {Heads}))
    (Utilities|Array|Add {HShown} -1))
  (Utilities|Time|SetTimerbyFunctionName :Object self :FunctionName "UpdateLamps" :Time __TIMER__ :bLooping true))
"""


def render(src: str) -> str:
    rep = {"__CYCLE__": "%.1f" % S.CYCLE_S, "__GREEN__": "%.1f" % S.GREEN_S,
           "__AMBER_END__": "%.1f" % (S.GREEN_S + S.YELLOW_S), "__HALF__": "%.1f" % S.HALF_CYCLE_S,
           "__WALK__": "%.1f" % S.WALK_S, "__TIMER__": "%.2f" % TIMER_S,
           "__OFF__": _m("MI_jctTrafficLightBase"), "__GREEN_M__": _m("MI_jctTrafficLight_Green"),
           "__YELLOW_M__": _m("MI_jctTrafficLight_Yellow"), "__RED_M__": _m("MI_jctTrafficLight_Red"),
           "__GREENB_M__": _m("MI_jctTrafficLight_Green_b"), "__REDB_M__": _m("MI_jctTrafficLight_Red_b")}
    out = src.format_map(_G())
    for k, v in rep.items():
        out = out.replace(k, v)
    return out


EDITOR = r'''
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
BT = "editor_toolset.toolsets.blueprint.BlueprintTools."
AT = "editor_toolset.toolsets.asset.AssetTools."
FOLDER = "/Game/CityLife/Blueprints"
NAME = "BP_SignalController"
BPP = FOLDER + "/" + NAME + "." + NAME
LABEL = "CityLife_SignalController"
PLAN = json.loads(__PLAN__)
LAMPS = json.loads(__LAMPS__)
EVENTS = json.loads(__EVENTS__)
ARRAYS = [("HOff", "float"), ("HRole", "int"), ("HG", "int"), ("HR", "int"), ("HShown", "int")]
SCALARS = [("NowT", "float"), ("Ph", "float"), ("Blink", "bool"), ("Code", "int"),
           ("G", "int"), ("R", "int")]


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    out = {}
    bp = {"refPath": BPP}
    if not T(AT + "exists", path=FOLDER + "/" + NAME):
        T(BT + "create", folder_path=FOLDER, asset_name=NAME,
          asset_type={"refPath": "/Script/Engine.Actor"})
        out["created"] = True
    have = [v if isinstance(v, str) else v["name"] for v in T(BT + "list_variables", blueprint=bp)]
    added = []
    if "Heads" not in have:
        T(BT + "add_object_variable", blueprint=bp, name="Heads",
          object_class={"refPath": "/Script/Engine.StaticMeshActor"}, container_type="ARRAY")
        added.append("Heads")
    if "Comp" not in have:
        T(BT + "add_object_variable", blueprint=bp, name="Comp",
          object_class={"refPath": "/Script/Engine.StaticMeshComponent"})
        added.append("Comp")
    for name, typ in ARRAYS:
        if name not in have:
            T(BT + "add_variable", blueprint=bp, name=name, type_name=typ, container_type="ARRAY")
            added.append(name)
    for name, typ in SCALARS:
        if name not in have:
            T(BT + "add_variable", blueprint=bp, name=name, type_name=typ)
            added.append(name)
    for name in ["Heads"] + [a for a, _ in ARRAYS]:
        T(BT + "set_variable_instance_editable", blueprint=bp, variable_name=name,
          instance_editable=True)
    out["added"] = added
    graphs = [x["refPath"].split(":")[-1] for x in T(BT + "list_graphs", blueprint=bp)]
    if "UpdateLamps" in graphs:
        gl = {"refPath": BPP + ":UpdateLamps"}
    else:
        gl = T(BT + "add_function_graph", blueprint=bp, graph_name="UpdateLamps")
    T(BT + "write_graph_dsl", graph=gl, code=LAMPS)
    ge = {"refPath": BPP + ":EventGraph"}
    T(BT + "write_graph_dsl", graph=ge, code=EVENTS)
    out["compile"] = T(BT + "compile_blueprint", blueprint=bp)

    by_label = {}
    for a in T(ST + "find_actors", name="TrafficLight", tag="", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            continue
        by_label[T(AC + "get_label", actor=a)] = a
    ctrl = None
    for a in T(ST + "find_actors", name="SignalController", tag="", collision_channels=[]):
        if "UEDPIE" not in a["refPath"] and T(AC + "get_label", actor=a) == LABEL:
            ctrl = a
    if ctrl is None:
        ctrl = T(ST + "add_to_scene_from_class",
                 actor_type={"refPath": FOLDER + "/" + NAME + "." + NAME + "_C"},
                 name=LABEL, xform={"location": {"x": 4100.0, "y": 0.0, "z": 2000.0}},
                 snap_to_ground=False)
        T(AC + "set_label", actor=ctrl, label=LABEL)
        T(ST + "set_actor_folder", actor=ctrl, folder_path="CityLife/Signals")
        out["placed"] = True
    rows = [p for p in PLAN if p["label"] in by_label]
    out["heads_planned"] = len(PLAN)
    out["heads_found"] = len(rows)
    vals = {"Heads": [by_label[p["label"]] for p in rows],
            "HOff": [p["off"] for p in rows], "HRole": [p["role"] for p in rows],
            "HG": [p["g"] for p in rows], "HR": [p["r"] for p in rows]}
    for k in vals:
        T(OT + "set_properties", instance=ctrl, values=json.dumps({k: []}))
        T(OT + "set_properties", instance=ctrl, values=json.dumps({k: vals[k]}))
    back = json.loads(T(OT + "get_properties", instance=ctrl, properties=["HOff", "HRole", "Heads"]))
    out["written"] = {k: len(back[k]) for k in back}
    return out
'''


def main():
    rows = plan()
    roles = {0: 0, 1: 0}
    for r in rows:
        roles[r["role"]] += 1
    print(f"{len(rows)} heads switched: {roles[0]} vehicle, {roles[1]} pedestrian "
          f"(F heads and unreadable ones left as they are)")
    if "--dry-run" in sys.argv:
        for r in rows[:12]:
            print(r)
        return
    code = (EDITOR.replace("__PLAN__", json.dumps(json.dumps(rows)))
            .replace("__LAMPS__", json.dumps(json.dumps(render(LAMPS))))
            .replace("__EVENTS__", json.dumps(json.dumps(render(EVENTS)))))
    print(json.dumps(ue_rpc.run_script(code), indent=1))


if __name__ == "__main__":
    main()
