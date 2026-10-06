# Runs inside the editor (ProgrammaticToolset), which supplies execute_tool.
#     python -m tools.citylife_mcp.ue_rpc script tools/citylife_mcp/inspect_signals.py
"""READ-ONLY survey of the CityLife_Day traffic signals and the crowd's Roam.

Before the level can run signals, three things have to be known:

  * what the signal heads ARE - separate StaticMeshActors or instances of an
    instanced component, which mesh (SM_jctTrafficLightA-F), which material
    slots, and which way each faces - because that decides whether a lamp
    can be switched with SetMaterial at all;
  * where they stand relative to the junction centres (4100 cm + k * 8200)
    and so which approach each one governs;
  * what BP_CityPed's Roam does today, so the walk logic that replaces it
    starts from the real graph (the script that wrote it is not in the repo).

Found 2026-09-29: 292 heads, every one its own StaticMeshActor whose lamps
are OverrideMaterials slots (B, the pedestrian head: [base, green_b, red_b]
or the reverse). Nothing is created, moved or saved. The caller writes the
result to docs/data/citylife_signals.json.
"""
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
BT = "editor_toolset.toolsets.blueprint.BlueprintTools."
PEDP = "/Game/CityLife/Blueprints/BP_CityPed.BP_CityPed"
GRID = 8200.0


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def prop(obj, n):
    try:
        d = json.loads(T(OT + "get_properties", instance=obj, properties=[n]))
        return d[n] if n in d else None
    except Exception as e:               # noqa: BLE001 - survey: record, go on
        return "ERR " + str(e)[:80]


def short(v):
    if isinstance(v, dict) and "refPath" in v:
        return v["refPath"].split(".")[-1]
    if isinstance(v, list):
        return [short(x) for x in v]
    return v


def near_junction(x, y):
    jx = round((x - 4100.0) / GRID) * GRID + 4100.0
    jy = round((y - 4100.0) / GRID) * GRID + 4100.0
    return jx, jy


def run():
    heads = []
    comp_seen = {}
    for a in T(ST + "find_actors", name="TrafficLight", tag="", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            continue
        label = T(AC + "get_label", actor=a)
        kind = label.split("_")[1] if label.startswith("SM_") else label
        xf = T(AC + "get_actor_transform", actor=a)
        loc, rot = xf["location"], xf["rotation"]
        jx, jy = near_junction(loc["x"], loc["y"])
        row = {"label": label, "kind": kind,
               "loc": [round(loc["x"]), round(loc["y"]), round(loc["z"])],
               "yaw": round(rot["yaw"], 1), "junction": [jx, jy],
               "rel": [round(loc["x"] - jx), round(loc["y"] - jy)]}
        n_kind = comp_seen.get(kind, 0)
        if n_kind < 4:
            comp_seen[kind] = n_kind + 1
            comps = []
            for c in T(AC + "get_components", actor=a)[:6]:
                comps.append({"ref": c["refPath"].split(".")[-1],
                              "mesh": short(prop(c, "StaticMesh")),
                              "slots": short(prop(c, "OverrideMaterials"))})
            row["components"] = comps
        else:
            c = T(AC + "get_components", actor=a)[0]
            row["slots"] = short(prop(c, "OverrideMaterials"))
        heads.append(row)
    graphs = [x["refPath"].split(":")[-1]
              for x in T(BT + "list_graphs", blueprint={"refPath": PEDP})]
    ped = {"graphs": graphs, "variables": T(BT + "list_variables", blueprint={"refPath": PEDP})}
    for g in graphs:
        if g in ("Roam", "EventGraph"):
            try:
                ped["dsl_" + g] = T(BT + "read_graph_dsl", graph={"refPath": PEDP + ":" + g})
            except Exception as e:       # noqa: BLE001
                ped["dsl_" + g] = "ERR " + str(e)[:200]
    return {"heads": heads, "ped": ped}
