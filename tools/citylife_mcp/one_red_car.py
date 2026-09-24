"""Leave exactly one red car in CityLife: Car_10, the drone's subject.

    python -m tools.citylife_mcp.ue_rpc script tools/citylife_mcp/one_red_car.py

Three cars on loop A were red: Car_10 (the TP body with MIC_Paint_Red) and
Car_04 and Car_22 (the Vehicle Variety Pack sports car, red by default). A
mission that says "follow the red car" is ambiguous with three, and the colour
gate cannot tell them apart. Car_04 becomes a blue TP car and Car_22 a white
one; both keep their route and speed.

Paint colours, as OpenCV hue of the sRGB-converted PaintColor (0-179): Red 0,
Orange 18, Yellow 26, Green 71, Cyan 92, Blue 112, Purple 152. The colour gate's
red band is 0-10 and 170-179, so Red is the only paint in it; Black and White
have no chroma.
"""
import json

OT = "editor_toolset.toolsets.object.ObjectTools."
ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
TP = "/Game/Car/SM_AutomotiveTP_Car.SM_AutomotiveTP_Car"
PAINT = {"Car_04": "/Game/Car/Assets/MIC_Paint_Blue.MIC_Paint_Blue",
         "Car_22": "/Game/Car/Assets/MIC_Paint_White.MIC_Paint_White"}


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    by_tag = {}
    for a in T(ST + "find_actors", name="", tag="citylife.car", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            continue
        for t in T(AC + "get_tags", actor=a):
            if t.startswith("Car_"):
                by_tag[t] = a
    out = {}
    for tag, mic in PAINT.items():
        a = by_tag[tag]
        body = {"refPath": a["refPath"] + ".Body"}
        T(OT + "set_properties", instance=body, values=json.dumps({"staticMesh": TP}))
        # slot 1 is the TP body's paint; write [] first (array-resize trap)
        for arr in ([], ["None", "None"], ["None", mic]):
            T(OT + "set_properties", instance=body, values=json.dumps({"overrideMaterials": arr}))
        # nudge so the render state re-registers with the new mesh
        l = T(AC + "get_actor_transform", actor=a)["location"]
        T(AC + "set_actor_transform", actor=a,
          xform={"location": {"x": l["x"] + 1, "y": l["y"], "z": l["z"]}})
        T(AC + "set_actor_transform", actor=a, xform={"location": l})
        g = json.loads(T(OT + "get_properties", instance=body,
                         properties=["staticMesh", "overrideMaterials"]))
        out[tag] = {"mesh": g["staticMesh"]["refPath"].split(".")[-1],
                    "overrides": [x if isinstance(x, str) else x["refPath"].split(".")[-1]
                                  for x in g["overrideMaterials"]]}
    # every car's body, so the one-red claim is checked over the whole fleet
    fleet = {}
    for tag, a in sorted(by_tag.items()):
        g = json.loads(T(OT + "get_properties", instance={"refPath": a["refPath"] + ".Body"},
                         properties=["staticMesh", "overrideMaterials"]))
        fleet[tag] = [g["staticMesh"]["refPath"].split(".")[-1]] + [
            x if isinstance(x, str) else x["refPath"].split(".")[-1] for x in g["overrideMaterials"]]
    out["fleet"] = fleet
    return out
