"""BP_CityCar EventGraph: Tick -> UpdateEffSpeed -> DriveTick -> wheel rotations.

Run after drive_tick.py:

    python -m tools.citylife_mcp.ue_rpc script tools/citylife_mcp/rewire_tick.py

The EventGraph is edited node by node rather than rewritten from DSL, because
its BeginPlay reaches the wheel components through SCS component getters, which
write_graph_dsl cannot recreate. So: splice the DriveTick call in after
UpdateEffSpeed, hand its exec on to the first wheel SetRelativeRotation (the
four wheel nodes and the MakeRotator/Get Spin/Get Steer nodes feeding them are
kept), then delete everything the old waypoint chain left behind, including
the orphans of earlier edits. Idempotent: a second run finds DriveTick already
wired and deletes nothing that is still reachable.
"""
import json

# The sandbox's dicts refuse .get(key, default): tool results are _StrictDict.

BT = "editor_toolset.toolsets.blueprint.BlueprintTools."
BPP = "/Game/CityLife/Blueprints/BP_CityCar.BP_CityCar"
G = {"refPath": BPP + ":EventGraph"}
EXEC_OUT = ("then", "True", "False", "Completed", "Loop Body")


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def nm(ref):
    return ref["refPath"].split(".")[-1]


def pin_of(info, direction, name):
    pins = info["input_pins"] if direction == "in" else info["output_pins"]
    for p in pins:
        if p["name"] == name:
            return p
    raise RuntimeError("%s has no %s pin %s: %s" % (nm(info["node"]), direction, name,
                                                     [q["name"] for q in pins]))


def run():
    infos = {nm(i["node"]): i for i in T(BT + "get_node_infos",
                                         nodes=T(BT + "find_nodes", graph=G, title=""))}
    by_type = {}
    for k, i in infos.items():
        if i["type_id"] not in by_type:
            by_type[i["type_id"]] = []
        by_type[i["type_id"]].append(k)
    report = {"n_before": len(infos),
              "types": sorted(set(t for t in by_type if "Event" in t or "CallFunction" in t))}

    tick = [k for k, i in infos.items() if "EventTick" in i["type_id"]]
    begin = [k for k, i in infos.items() if "BeginPlay" in i["type_id"]]
    overlap = [k for k, i in infos.items() if "ActorBeginOverlap" in i["type_id"]]
    # a function call node's type_id is "|Name"; create_node takes "CallFunction|Name"
    eff = (by_type["|UpdateEffSpeed"] if "|UpdateEffSpeed" in by_type else [])
    drive = (by_type["|DriveTick"] if "|DriveTick" in by_type else [])
    # the wheel nodes that RUN: reachable from Tick along exec links. Earlier
    # edits left four more SetRelativeRotation nodes orphaned in the graph.
    live, todo = set(), list(tick)
    while todo:
        k = todo.pop()
        if k in live:
            continue
        live.add(k)
        for p in infos[k]["output_pins"]:
            for c in p["connected_pins"]:
                ck = nm(c["node"])
                if ck in infos and any(q["name"] == "execute" and
                                       any(nm(z["node"]) == k for z in q["connected_pins"])
                                       for q in infos[ck]["input_pins"]):
                    todo.append(ck)
    srr = [k for k in (by_type["Transformation|SetRelativeRotation"]
                       if "Transformation|SetRelativeRotation" in by_type else []) if k in live]
    report.update({"tick": tick, "begin": begin, "overlap": overlap, "eff": eff,
                   "drive": drive, "srr": srr})
    if len(tick) != 1 or len(eff) != 1 or len(srr) != 4 or len(begin) != 1:
        report["abort"] = "unexpected graph shape"
        return report
    tick, eff, begin = tick[0], eff[0], begin[0]

    # the first wheel node: the one whose exec input does NOT come from another wheel node
    first = None
    for k in srr:
        src = [nm(c["node"]) for c in pin_of(infos[k], "in", "execute")["connected_pins"]]
        if not any(s in srr for s in src):
            first = k
    report["first_wheel"] = first

    if not drive:
        d = T(BT + "create_node", graph=G, type_id="CallFunction|DriveTick",
              pos={"x": 600, "y": -300})
        drive = nm(d)
        infos[drive] = T(BT + "get_node_infos", nodes=[d])[0]
    else:
        drive = drive[0]
    report["drive_node"] = drive

    def ref(k):
        return infos[k]["node"]

    def pid(k, direction, name):
        p = pin_of(infos[k], direction, name)
        return {"direction": "EGPD_Input" if direction == "in" else "EGPD_Output",
                "index_id": p["pin_id"]["index_id"], "node": ref(k)}

    # UpdateEffSpeed.then -> DriveTick
    eff_then = pin_of(infos[eff], "out", "then")
    for c in eff_then["connected_pins"]:
        if nm(c["node"]) != drive:
            T(BT + "break_pins", output_pin=pid(eff, "out", "then"),
              input_pin={"direction": "EGPD_Input", "index_id": c["index_id"], "node": c["node"]})
    T(BT + "connect_pins", output_pin=pid(eff, "out", "then"), input_pin=pid(drive, "in", "execute"))
    # DriveTick.then -> first wheel; cut whatever else fed that wheel's exec
    for c in pin_of(infos[first], "in", "execute")["connected_pins"]:
        if nm(c["node"]) != drive:
            T(BT + "break_pins",
              output_pin={"direction": "EGPD_Output", "index_id": c["index_id"], "node": c["node"]},
              input_pin=pid(first, "in", "execute"))
    T(BT + "connect_pins", output_pin=pid(drive, "out", "then"), input_pin=pid(first, "in", "execute"))

    # re-read after rewiring, then keep: BeginPlay's exec chain, the three
    # entry points, the two calls, the wheel nodes, and all their data inputs
    infos = {nm(i["node"]): i for i in T(BT + "get_node_infos",
                                         nodes=T(BT + "find_nodes", graph=G, title=""))}
    keep = set()

    def add(k, follow_exec):
        if k in keep or k not in infos:
            return
        keep.add(k)
        i = infos[k]
        for p in i["input_pins"]:
            if p["name"] in ("execute", "exec"):
                continue
            for c in p["connected_pins"]:
                add(nm(c["node"]), False)
        if follow_exec:
            for p in i["output_pins"]:
                if p["name"] in EXEC_OUT:
                    for c in p["connected_pins"]:
                        add(nm(c["node"]), True)

    add(begin, True)
    for k in overlap + [tick, eff, drive] + srr:
        add(k, False)
    doomed = [k for k in infos if k not in keep]
    report["kept"] = sorted(keep)
    for k in doomed:
        T(BT + "delete_node", node=infos[k]["node"])
    report["deleted"] = len(doomed)
    T(BT + "compile_blueprint", blueprint={"refPath": BPP})
    try:
        T(BT + "remove_function_graph", blueprint={"refPath": BPP}, graph_name="UpdateAim")
        report["removed_UpdateAim"] = True
    except Exception as e:                       # noqa: BLE001
        report["removed_UpdateAim"] = str(e)[:300]
    T(BT + "compile_blueprint", blueprint={"refPath": BPP})
    report["event_dsl"] = T(BT + "read_graph_dsl", graph=G)
    return report
