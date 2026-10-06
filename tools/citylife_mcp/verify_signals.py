"""Check the signal lamps in a running Simulate against the plan.

    python -m tools.citylife_mcp.verify_signals          # sample now
    python -m tools.citylife_mcp.verify_signals --n 40   # how many heads

Reads, from the PIE world, the materials in each sampled head's lamp slots,
between two reads of the controller's NowT, and compares them with what
tools/citylife_mcp/signals.py `lamp_code` says the head shows at some time
in [NowT before - 0.25, NowT after + 0.25]: NowT is the time of the last
UpdateLamps (a 0.25 s timer), and every editor call lets the game advance a
frame, so a whole-sample time would be wrong by the length of the sample.
Needs Simulate running (verify_drive start / StartPIE). Exits 1 on anything
but a clean pass.

BOTH SLOTS. A head nothing drives keeps its mesh materials: red AND green
lit (signals.MESH_DEFAULT). Checking only the slot the expected code lights
passed such a head whenever the plan said red or green - the one fault this
check exists for. So per candidate code the lit slot must show its material
AND the other slot MI_jctTrafficLightBase when G != R (code 5, the dark half
of a flash: both dark), as UpdateLamps writes them (`expected`). The blanket
"a flash may be 3 or 5" is gone: the time bracket holds both codes only when
the read really straddled a blink edge.

A SINGLE-LAMP pedestrian head (kind E, g == r: 30 of the 258 planned, 4 of
the default 33 sampled) has no other slot. Left alone it shows its one
lamp, Green_b or Red_b, which IS the walk or don't-walk code whenever the
plan gives it; an undriven E head passed the lamp comparison in an offline
run of main() (2026-09-30). Only the driven check below catches one, which
is why a run that could not read Heads/HShown exits 1.

NOT DRIVEN is reported apart from a mismatch: a sampled head missing from
the PIE world, one not in the controller's Heads array, or one whose HShown
is still -1 (UpdateLamps never set it).
"""
import json
import math
import sys

from tools.citylife_mcp import signals as L
from tools.citylife_mcp import ue_rpc

EDITOR = r'''
import json
ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
WANT = json.loads(__WANT__)


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def tail(ref):
    s = ref["refPath"] if isinstance(ref, dict) else str(ref)
    return s.split(".")[-1].split(":")[-1]


def run():
    ctrl = None
    for a in T(ST + "find_actors", name="SignalController", tag="", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            ctrl = a
    if ctrl is None:
        return {"error": "no SignalController in the PIE world - is Simulate running?"}

    def now():
        return json.loads(T(OT + "get_properties", instance=ctrl, properties=["NowT"]))["NowT"]

    try:
        c = json.loads(T(OT + "get_properties", instance=ctrl, properties=["Heads", "HShown"]))
        heads = [tail(h) if h else None for h in c["Heads"]]
        shown = list(c["HShown"])
    except (RuntimeError, KeyError, TypeError):
        heads, shown = None, None
    out = {}
    for a in T(ST + "find_actors", name="TrafficLight", tag="", collision_channels=[]):
        if "UEDPIE" not in a["refPath"]:
            continue
        lab = T(AC + "get_label", actor=a)
        if lab not in WANT:
            continue
        comp = T(AC + "get_components", actor=a)[0]
        t0 = now()
        d = json.loads(T(OT + "get_properties", instance=comp, properties=["OverrideMaterials"]))
        t1 = now()
        mats = [m["refPath"].split(".")[-1] if isinstance(m, dict) else m
                for m in d["OverrideMaterials"]]
        k = heads.index(tail(a)) if heads is not None and tail(a) in heads else None
        out[lab] = {"mats": mats, "t0": t0, "t1": t1,
                    "in_ctrl": None if heads is None else k is not None,
                    "shown": shown[k] if k is not None and k < len(shown) else None}
    return {"heads": out, "n_ctrl": None if heads is None else len(heads)}
'''

BASE = "MI_jctTrafficLightBase"
LIT = {0: ("g", "MI_jctTrafficLight_Green"), 1: ("g", "MI_jctTrafficLight_Yellow"),
       2: ("r", "MI_jctTrafficLight_Red"), 3: ("g", "MI_jctTrafficLight_Green_b"),
       4: ("r", "MI_jctTrafficLight_Red_b")}
TICK_S = 0.25                   # UpdateLamps' timer: the read may be one update either side


def expected(code: int, g: int, r: int) -> dict:
    """{slot: material} after UpdateLamps shows `code` on a head with lamp
    slots g and r (equal for a single-lamp pedestrian head)."""
    if code == 5:
        return {g: BASE, r: BASE}
    which, mat = LIT[code]
    lit, other = (g, r) if which == "g" else (r, g)
    out = {lit: mat}
    if other != lit:
        out[other] = BASE
    return out


def codes_between(ta: float, tb: float, off: float, role: int, step: float = 0.01) -> set:
    """Every code the plan gives in [ta, tb], sampled finer than any phase."""
    n = max(1, int(math.ceil((tb - ta) / step)))
    return {L.lamp_code(ta + (tb - ta) * k / n, off, role) for k in range(n + 1)}


def shows(mats: list, want: dict) -> bool:
    return all(s < len(mats) and mats[s] == m for s, m in want.items())


def main():
    n = int(sys.argv[sys.argv.index("--n") + 1]) if "--n" in sys.argv else 30
    plan = L.plan()
    step = max(1, len(plan) // n)
    pick = {p["label"]: p for p in plan[::step]}
    survey = {h["label"]: L.slots_of(h) for h in json.loads(L.SURVEY.read_text(encoding="utf-8"))["heads"]}
    code = EDITOR.replace("__WANT__", json.dumps(json.dumps(sorted(pick))))
    res = ue_rpc.run_script(code)
    if "error" in res:
        print(res["error"])
        sys.exit(1)
    ok, bad, not_driven = 0, 0, []
    for lab, h in sorted(res["heads"].items()):
        p = pick[lab]
        # an empty or absent override slot shows the mesh's own material
        default, raw = survey.get(lab, []), h["mats"]
        mats = []
        for k in range(max(len(raw), len(default))):
            m = raw[k] if k < len(raw) else None
            mats.append(m if m not in (None, "None") else (default[k] if k < len(default) else None))
        if h["in_ctrl"] is False:
            not_driven.append((lab, "not in the controller's Heads"))
        elif h["shown"] == -1:
            not_driven.append((lab, "HShown -1: never updated"))
        cand = codes_between(h["t0"] - TICK_S, h["t1"] + TICK_S, p["off"], p["role"])
        good = any(shows(mats, expected(c, p["g"], p["r"])) for c in cand)
        ok += int(good)
        bad += int(not good)
        if not good:
            print("MISMATCH", lab, "role", p["role"], p["axis"], "want", sorted(cand),
                  "slots g/r", p["g"], p["r"], "shows", mats)
    missing = sorted(set(pick) - set(res["heads"]))
    for lab, why in not_driven:
        print("NOT DRIVEN", lab, why)
    if missing:
        print("NOT IN THE PIE WORLD", missing)
    checked = res.get("n_ctrl") is not None
    if not checked:
        print("controller Heads/HShown unreadable: whether a head is driven was NOT CHECKED")
    ts = [x for h in res["heads"].values() for x in (h["t0"], h["t1"])]
    print(json.dumps({"t_game": [min(ts), max(ts)] if ts else None, "sampled": len(res["heads"]),
                      "ok": ok, "mismatch": bad, "not_driven": len(not_driven),
                      "missing": len(missing), "driven_checked": checked,
                      "controller_heads": res.get("n_ctrl")}))
    sys.exit(0 if (bad == 0 and not not_driven and not missing and checked and res["heads"]) else 1)


if __name__ == "__main__":
    main()
