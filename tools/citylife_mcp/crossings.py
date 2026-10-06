# Copied verbatim on 2026-09-29 from the session scratchpad script that cut the
# seven crossing gaps into the CityLife_Day no-walk bands on 2026-09-22. It
# existed only in a temp directory; tools/citylife_routes.py CROSSINGS and
# tools/citylife_peds.py depend on the geometry it wrote into the level.
# Runs inside the editor (ProgrammaticToolset), which supplies execute_tool.
"""Open pedestrian crossings by cutting gaps in the no-walk bands.

Run through ProgrammaticToolset.execute_tool_script once the editor is back.

A NavLinkProxy would be the textbook answer, but its PointLinks is a struct
array and struct-array writes through this toolset are documented as unreliable.
A gap in the NavArea_Null band needs no struct write at all: the navmesh simply
stays walkable across the carriageway there, which is what a crossing IS.

Gaps are 600 cm wide (a zebra is about that) and sit 1100 cm either side of a
junction centre, where the crosswalk markings are painted.
"""
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."

HALF_ROAD = 800.0          # carriageway half width
GAP = 300.0                # half width of a crossing corridor
Y_LO, Y_HI = -5000.0, 13500.0
X_LO, X_HI = 1500.0, 21500.0
# Crossings only on the two junctions the demo corridor runs through.
NS_BANDS = {"CityLife_NoWalk_NS_0": (4100.0, [-3000.0, 3000.0, 5200.0])}
EW_BANDS = {"CityLife_NoWalk_EW_0": (-4100.0, [3000.0, 5200.0]),
            "CityLife_NoWalk_EW_1": (4100.0, [3000.0, 5200.0])}
DROP = ("CityLife_NoWalk_NS", "CityLife_NoWalk_EW")   # the hand-fitted originals


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def segments(lo, hi, gaps):
    """Spans of [lo, hi] left solid once each gap is cut out."""
    out, cur = [], lo
    for g in sorted(gaps):
        a, b = g - GAP, g + GAP
        if a > cur:
            out.append((cur, a))
        cur = max(cur, b)
    if cur < hi:
        out.append((cur, hi))
    return [(a, b) for a, b in out if b - a > 200.0]


def place(label, cx, cy, hx, hy):
    a = T(ST + "add_to_scene_from_class",
          actor_type={"refPath": "/Script/NavigationSystem.NavModifierVolume"},
          name=label,
          xform={"location": {"x": cx, "y": cy, "z": 100.0},
                 "scale": {"x": hx / 100.0, "y": hy / 100.0, "z": 2.0}},
          snap_to_ground=False)
    T(AC + "set_label", actor=a, label=label)
    T(ST + "set_actor_folder", actor=a, folder_path="CityLife/Nav")
    T(OT + "set_properties", instance=a, values=json.dumps(
        {"areaClass": "/Script/NavigationSystem.NavArea_Null"}))
    return a


def run():
    by_label = {}
    for a in T(ST + "find_actors", name="NoWalk", tag="", collision_channels=[]):
        by_label[T(AC + "get_label", actor=a)] = a
    removed = []
    for label in DROP:
        if label in by_label:
            T(ST + "remove_from_scene", actor=by_label[label])
            removed.append(label)
    made = []
    for label, (cx, gaps) in NS_BANDS.items():
        if label in by_label:
            T(ST + "remove_from_scene", actor=by_label[label])
            removed.append(label)
        for i, (a, b) in enumerate(segments(Y_LO, Y_HI, gaps)):
            name = "%s_s%d" % (label, i)
            place(name, cx, (a + b) / 2.0, HALF_ROAD, (b - a) / 2.0)
            made.append([name, round(a), round(b)])
    for label, (cy, gaps) in EW_BANDS.items():
        if label in by_label:
            T(ST + "remove_from_scene", actor=by_label[label])
            removed.append(label)
        for i, (a, b) in enumerate(segments(X_LO, X_HI, gaps)):
            name = "%s_s%d" % (label, i)
            place(name, (a + b) / 2.0, cy, (b - a) / 2.0, HALF_ROAD)
            made.append([name, round(a), round(b)])
    now = sorted(T(AC + "get_label", actor=a)
                 for a in T(ST + "find_actors", name="NoWalk", tag="",
                            collision_channels=[]))
    return {"removed": removed, "created": made, "bands_now": now}
