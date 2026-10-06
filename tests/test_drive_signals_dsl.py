"""The rendered UpdateEffSpeed graph is well formed, and its ZebraWait can fire.

tools/citylife_mcp/drive_signals.py renders eff_signals.dsl into the
write_graph_dsl text the editor compiles. Nothing here can run Unreal, so
this file reads that text the way the DSL does - an `(else ...)` or
`(elif ...)` only as the LAST form of an `if` - and RUNS the junction block
with a small evaluator of the forms it uses. That is how the ZebraWait
finding was confirmed: its test sat where the front is always >= 1450 cm out
(the zebra ends at 1400), so it could never count, and a Simulate that read
0 had measured nothing.

The same evaluator runs signals.py's UpdateLamps against what
verify_signals.py expects of a head: BOTH lamp slots, since a head nothing
drives keeps red and green lit and passed the old one-slot check. The parser
and the evaluator are also used by tests/test_ped_walk_plan.py.

Run either way:
    pytest tests/test_drive_signals_dsl.py -v
    python tests/test_drive_signals_dsl.py
"""
import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import citylife_routes as R                      # noqa: E402
from tools import citylife_traffic_model as T               # noqa: E402
from tools.citylife_mcp import drive_signals as D           # noqa: E402
from tools.citylife_mcp import drive_tick as DT             # noqa: E402
from tools.citylife_mcp import signals as SL                # noqa: E402
from tools.citylife_mcp import verify_signals as VS         # noqa: E402


# ------------------------------------------------------------------ reading the DSL

class Str(str):
    """A "quoted" literal, kept apart from a symbol."""


_NUM = re.compile(r"^-?\d+(\.\d*)?$")


def tokenize(src: str) -> list:
    toks, i, n = [], 0, len(src)
    while i < n:
        ch = src[i]
        if ch.isspace():
            i += 1
        elif ch == ";":
            j = src.find("\n", i)
            i = n if j < 0 else j
        elif ch in "()":
            toks.append(ch)
            i += 1
        elif ch == '"':
            j = src.index('"', i + 1)
            toks.append(Str(src[i + 1:j]))
            i = j + 1
        else:
            j = i
            while j < n and not src[j].isspace() and src[j] != ")":
                # Min(Float), %(Integer), Get(acopy): a "(" glued to a name is part of it
                j = src.index(")", j) + 1 if src[j] == "(" else j + 1
            toks.append(src[i:j])
            i = j
    return toks


def parse(src: str) -> list:
    """Top-level forms as nested lists; symbols str, numbers int/float, literals Str."""
    toks, pos = tokenize(src), 0

    def one():
        nonlocal pos
        if pos >= len(toks):
            raise ValueError("unexpected end: an unclosed '('")
        t = toks[pos]
        pos += 1
        if t == "(":
            out = []
            while pos < len(toks) and toks[pos] != ")":
                out.append(one())
            if pos >= len(toks):
                raise ValueError("unclosed '('")
            pos += 1
            return out
        if t == ")":
            raise ValueError("unexpected ')'")
        if isinstance(t, Str):
            return t
        if _NUM.match(t):
            return float(t) if "." in t else int(t)
        return t

    forms = []
    while pos < len(toks):
        forms.append(one())
    return forms


def structure_errors(forms: list) -> list:
    """Where the DSL would misread a branch: an else/elif that is not the last
    form of an if/elif, one outside an if, or an if without a statement."""
    errs = []

    def walk(node, parent, k, nsib, path):
        if not isinstance(node, list) or not node:
            return
        head = node[0]
        if head in ("else", "elif") and (parent not in ("if", "elif") or k != nsib - 1):
            errs.append("%s at %s is not the last form of an if" % (head, path))
        if head in ("if", "elif") and len(node) < 3:
            errs.append("%s at %s has no statement" % (head, path))
        for i, ch in enumerate(node):
            walk(ch, head if isinstance(head, str) else None, i, len(node), path + [i])

    for i, f in enumerate(forms):
        walk(f, None, 0, 1, [i])
    return errs


def find(node, pred):
    """First list (depth first) that satisfies pred, or None."""
    if isinstance(node, list):
        if node and pred(node):
            return node
        for ch in node:
            hit = find(ch, pred)
            if hit is not None:
                return hit
    return None


def var_names(src: str, kind: str = "[GS]et") -> set:
    return set(re.findall(r"Variables\|Default\|%s([A-Za-z0-9_]+)" % kind, src))


# ------------------------------------------------------------------ running it

class Obj:
    """A Blueprint instance: its variables, and what Transformation reads."""

    def __init__(self, cls="", vars=None, loc=(0.0, 0.0, 0.0), fwd=(1.0, 0.0, 0.0),
                 vel=(0.0, 0.0, 0.0)):
        self.cls, self.vars = cls, dict(vars or {})
        self.loc, self.fwd, self.vel = tuple(loc), tuple(fwd), tuple(vel)


def _vec(a, b, op):
    if isinstance(a, tuple) and isinstance(b, tuple):
        return tuple(op(x, y) for x, y in zip(a, b))
    if isinstance(a, tuple) or isinstance(b, tuple):
        raise TypeError("vector with scalar")
    return op(a, b)


class Lazy:
    """A `bind` of a PURE node: Blueprint re-evaluates a pure node at every
    use, so its value follows any member it reads that changed in between.
    Capturing it once (as this evaluator first did) hid the pedestrian bug of
    2026-09-30: a node kind read after SetIdx was the NEXT point's kind."""

    def __init__(self, expr):
        self.expr = expr


# Nodes with an exec pin: evaluated once, where the bind stands.
IMPURE_PREFIXES = ("Actor|GetAllActorsOfClass", "CallFunction|")


class Machine:
    """Runs the forms the car and pedestrian graphs use, as Blueprint pure
    nodes do (both sides of and/or and select are evaluated; a bound pure
    node is re-evaluated at each use). A form it does not know raises
    NotImplementedError, and a variable read before it was set raises
    KeyError: nothing silently reads as 0."""

    def __init__(self, me: Obj, world: dict = None):
        self.me, self.world, self.calls = me, dict(world or {}), []

    def run(self, forms, local=None) -> dict:
        local = {} if local is None else local
        for f in forms:
            self.ev(f, local)
        return local

    @staticmethod
    def _kw(args):
        pos, kw, i = [], {}, 0
        while i < len(args):
            a = args[i]
            if isinstance(a, str) and not isinstance(a, Str) and a.startswith(":"):
                kw[a[1:]] = args[i + 1]
                i += 2
            else:
                pos.append(a)
                i += 1
        return pos, kw

    def _if(self, args, L):
        cond, body = args[0], list(args[1:])
        tail = None
        if body and isinstance(body[-1], list) and body[-1] and body[-1][0] in ("else", "elif"):
            tail = body.pop()
        if self.ev(cond, L):
            for s in body:
                self.ev(s, L)
        elif tail is not None:
            if tail[0] == "else":
                for s in tail[1:]:
                    self.ev(s, L)
            else:
                self._if(tail[1:], L)

    def ev(self, node, L):
        if isinstance(node, Str):
            return str(node)
        if isinstance(node, (int, float)):
            return node
        if isinstance(node, str):
            if node in ("true", "false"):
                return node == "true"
            if node == "self":
                return self.me
            if node in L:
                v = L[node]
                return self.ev(v.expr, L) if isinstance(v, Lazy) else v
            raise NameError(node)
        head, args = node[0], node[1:]
        ev = lambda x: self.ev(x, L)                               # noqa: E731
        if head == "if":
            return self._if(args, L)
        if head in ("else", "elif"):
            raise SyntaxError("%s outside an if" % head)
        if head == "bind":
            name, expr, branches = args[0], args[1], args[2:]
            if isinstance(expr, list) and expr[0].startswith("Utilities|Casting|CastTo"):
                _, kw = self._kw(expr[1:])
                obj = ev(kw["Object"])
                want = expr[0][len("Utilities|Casting|CastTo"):]
                ok = isinstance(obj, Obj) and obj.cls == want
                if ok:
                    L[name] = obj
                for br in branches:
                    if br[0] == (":then" if ok else ":CastFailed"):
                        for s in br[1:]:
                            self.ev(s, L)
                return None
            if (isinstance(expr, list) and expr and isinstance(expr[0], str)
                    and not expr[0].startswith(IMPURE_PREFIXES)):
                L[name] = Lazy(expr)
            else:
                L[name] = ev(expr)
            return None
        if head == "for":
            var, items, body = args[0], ev(args[1]), args[2:]
            for it in list(items):
                L[var] = it
                for s in body:
                    self.ev(s, L)
            return None
        if head == "range":
            return range(int(ev(args[0])))
        if head == "switch":                                       # (switch int X (:0 ...) (:Default ...))
            key = str(ev(args[1]))
            branches = {br[0][1:]: br[1:] for br in args[2:]}
            for s in branches.get(key, branches.get("Default", [])):
                self.ev(s, L)
            return None
        if head in ("and", "or"):
            a, b = bool(ev(args[0])), bool(ev(args[1]))
            return (a and b) if head == "and" else (a or b)
        if head == "not":
            return not ev(args[0])
        if head == "select":
            c, a, b = ev(args[0]), ev(args[1]), ev(args[2])
            return a if c else b
        if head in ("==", "!=", "<", ">", "<=", ">="):
            a, b = ev(args[0]), ev(args[1])
            return {"==": a == b, "!=": a != b, "<": a < b, ">": a > b,
                    "<=": a <= b, ">=": a >= b}[head]
        if head in ("+", "-", "*"):
            a, b = ev(args[0]), ev(args[1])
            op = {"+": lambda x, y: x + y, "-": lambda x, y: x - y, "*": lambda x, y: x * y}[head]
            return _vec(a, b, op)
        if head == "/":
            a, b = ev(args[0]), ev(args[1])
            if b == 0:
                raise ZeroDivisionError("DSL divides by zero")
            return a / b
        if head in (".x", ".y", ".z"):
            return ev(args[0])["xyz".index(head[1])]
        pos, kw = self._kw(args)
        P_ = [ev(a) for a in pos]
        if head.startswith("Variables|Default|Get"):
            name = head[len("Variables|Default|Get"):]
            if name not in self.me.vars:
                raise KeyError("read before set: " + name)
            return self.me.vars[name]
        if head.startswith("Variables|Default|Set"):
            self.me.vars[head[len("Variables|Default|Set"):]] = P_[0]
            return None
        if head.startswith("Class|") and "|Get" in head:
            obj = ev(kw["self"])
            return obj.vars[head.split("|Get", 1)[1]]
        if head == "CallFunction|SpeedOf":
            return ev(kw["self"]).vars["CurSpeed"]
        tgt = ev(kw["self"]) if "self" in kw else self.me
        if head == "Transformation|GetActorLocation":
            return tgt.loc
        if head == "Transformation|GetActorForwardVector":
            return tgt.fwd
        if head == "Transformation|GetVelocity":
            return tgt.vel
        if head == "Actor|GetAllActorsOfClass":
            return list(self.world.get("actors", {}).get(P_[0], []))
        if head == "Utilities|Time|GetGameTimeInSeconds":
            return self.world["t"]
        if head == "Utilities|Array|Length":
            return len(P_[0])
        if head == "Utilities|Array|Get(acopy)":
            return P_[0][int(P_[1])]
        if head == "Transformation|SetActorLocation":
            tgt.loc = tuple(ev(kw["NewLocation"]))
            return None
        if head == "Pawn|Input|AddMovementInput":
            self.calls.append(("move", ev(kw["WorldDirection"])))
            return None
        if head == "Utilities|Array|SetArrayElem":
            P_[0][int(P_[1])] = P_[2]
            return None
        if head == "Rendering|Material|SetMaterial":
            tgt.vars.setdefault("mats", {})[int(ev(kw["ElementIndex"]))] = ev(kw["Material"]).split(".")[-1]
            return None
        if head == "Math|Random|RandomBoolWithWeight":
            return False
        if head == "Math|Random|RandomFloatInRange":
            return P_[0]
        f = {
            "Math|Float|Min(Float)": lambda a, b: min(a, b),
            "Math|Float|Max(Float)": lambda a, b: max(a, b),
            "Math|Float|Absolute(Float)": abs,
            "Math|Float|Sign(Float)": lambda a: float((a > 0) - (a < 0)),
            "Math|Float|Clamp(Float)": lambda v, lo, hi: max(lo, min(hi, v)),
            "Math|Float|%(Float)": math.fmod,
            "Math|Integer|%(Integer)": lambda a, b: int(math.fmod(a, b)),
            "Math|Float|Truncate": lambda a: int(math.trunc(a)),
            "Math|Trig|Cos(Degrees)": lambda a: math.cos(math.radians(a)),
            "Math|Trig|Sin(Degrees)": lambda a: math.sin(math.radians(a)),
            "Math|Vector|MakeVector": lambda x, y, z: (x, y, z),
            "Math|Vector|DotProduct": lambda a, b: sum(p * q for p, q in zip(a, b)),
            "Math|Vector|VectorLengthXY": lambda a: math.hypot(a[0], a[1]),
            "Math|Vector|Normalize": lambda a: (tuple(c / math.sqrt(sum(q * q for q in a)) for c in a)
                                                if sum(q * q for q in a) > 0 else (0.0, 0.0, 0.0)),
        }.get(head)
        if f is None:
            raise NotImplementedError(head)
        return f(*P_)


# ------------------------------------------------------------------ the graph

EFF = D.render()
TREE = parse(EFF)
K = D.constants()


def _junction_block():
    """The (if {CheckJ} ...) of the junction rules: its first statement sets DLine."""
    return find(TREE, lambda n: n[0] == "if" and n[1] == ["Variables|Default|GetCheckJ"]
                and isinstance(n[2], list) and n[2][0] == "Variables|Default|SetDLine")


BASE = dict(CheckJ=True, NJi=0, SneakJ=-1, HeldTurn=0.0, Hold=0, Sig=0, NextJ=(4100.0, 4100.0, 0.0),
            CurSpeed=0.0, Ph=10.0, Conflict=False, OncGo=False, Give=False, ExitBlk=False,
            ExitPed=False, Dt=0.1, StopCm=99999.0, GapCm=99999.0, Target=0.0, LateHold=0,
            SigHolds=0, YieldTicks=0, ZebraWait=0, Sneaks=0, PrevNJ=-1, PrevDL=0.0, RedViol=0,
            Sneaking=False, DLine=0.0)


def run_junction(along: float, **kw) -> dict:
    """Run the junction block for a car whose centre is `along` cm from NextJ's
    centre along its heading (JEdge = along - JNEAR); returns its variables."""
    car = Obj("BP_CityCar", dict(BASE, JEdge=along - K["JNEAR"], **kw))
    Machine(car).run([_junction_block()])
    return car.vars


def test_the_rendered_graph_is_balanced_and_complete():
    D.check_parens(EFF)
    for bad in ("@", "{", "}", "__CAR__", "__PED__"):
        assert bad not in EFF, bad
    assert len(TREE) == 1 and TREE[0][:2] == ["fn", "UpdateEffSpeed"], TREE[0][:2]
    assert K["ZEBRA_INNER"] == T.ZEBRA_CENTRE_CM - T.ZEBRA_HALF_DEPTH_CM == 800.0
    assert K["QUEUE"] == T.QUEUE_CM == 650.0


def test_else_and_elif_are_only_ever_the_last_form_of_an_if():
    assert structure_errors(TREE) == [], structure_errors(TREE)[:5]
    # the check itself fires: an else followed by a statement, an else outside an if
    assert structure_errors(parse("(if a (else b) c)"))
    assert structure_errors(parse("(for _i x (else b))"))
    assert not structure_errors(parse("(if a b (elif c d (else e)))"))


def test_every_variable_it_reads_or_writes_exists_on_the_car():
    """A typo in a Get/Set name compiles to a missing variable, not an error
    here: every name must be declared by NEW_VARS or already used by the
    working drive_tick.py graphs."""
    src = Path(DT.__file__).read_text(encoding="utf-8")
    known = (var_names(src) | set(re.findall(r"\{([A-Za-z][A-Za-z0-9_]*)\}", src))
             | {v[0] for v in DT.NEW_VARS} | {v[0] for v in D.NEW_VARS})
    assert var_names(EFF) <= known, sorted(var_names(EFF) - known)


def test_a_committed_car_held_for_the_box_on_the_zebra_counts():
    """Committed (DLine <= -50) with a car in the box: the late target JEdge -
    300 leaves the centre 1400 cm out and the front on the zebra."""
    v = run_junction(1400.0, Conflict=True)
    assert v["DLine"] == 1400.0 - K["STOP"] and v["DLine"] <= K["NEG_COMMIT"]
    assert v["Hold"] == 7 and v["LateHold"] == 1
    assert v["Target"] == v["StopCm"] == 1400.0 - K["JNEAR"] - 300.0
    assert v["ZebraWait"] == 1, v["ZebraWait"]
    assert v["YieldTicks"] == 0 and v["SigHolds"] == 0          # as the model: late_hold only


def test_a_late_box_hold_in_the_held_branch_is_hold_7_too():
    v = run_junction(1720.0, Conflict=True, CurSpeed=400.0)      # DLine -10: cannot stop at the line
    assert v["Hold"] == 7 and v["Target"] == 1720.0 - K["JNEAR"] - 300.0 and v["LateHold"] == 1
    assert v["ZebraWait"] == 0                                   # moving, and 1490 out


def test_no_count_unless_held_stopped_binding_and_on_the_band():
    assert run_junction(1400.0)["ZebraWait"] == 0                              # no conflict: committed
    assert run_junction(1400.0, Conflict=True, CurSpeed=50.0)["ZebraWait"] == 0  # still moving
    assert run_junction(1400.0, Conflict=True, GapCm=500.0)["ZebraWait"] == 0   # the queue binds
    assert run_junction(1400.0, Conflict=True, StopCm=-20.0)["ZebraWait"] == 0  # a crossing stop binds
    assert run_junction(1400.0, Conflict=True, StopCm=0.0)["ZebraWait"] == 1    # a tie counts


def test_a_red_hold_at_the_line_is_off_the_zebra():
    """What the old test could see: held at the line the front is 1500 out."""
    v = run_junction(K["STOP"], Sig=2)
    assert v["Hold"] == 1 and v["SigHolds"] == 1 and v["StopCm"] == 0.0
    assert v["ZebraWait"] == 0


def test_the_band_is_the_models_zebra_overlap_on_a_straight_approach():
    """Every along-distance a hold can leave a stopped car at, on the four
    approaches of (4100, 4100) (a zebra on each leg): the DSL counts exactly
    when the model's _on_zebra (rotated footprint against all seven zebra
    boxes) says the body is on one."""
    jx, jy = 4100.0, 4100.0
    n = 0
    for u in ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)):
        lx, ly = R.left_of(u)
        for along in range(1105, 3600, 10):
            x = jx - u[0] * along + lx * R.LANE_OFFSET_CM
            y = jy - u[1] * along + ly * R.LANE_OFFSET_CM
            model = T._on_zebra(SimpleNamespace(st=SimpleNamespace(x=x, y=y), fx=u[0], fy=u[1]))
            if along - K["STOP"] > K["NEG_COMMIT"]:
                v = run_junction(float(along), Sig=2)            # held at the line by red
            else:
                v = run_junction(float(along), Conflict=True)    # committed, late box hold
            assert (v["ZebraWait"] == 1) == model, (u, along, v["ZebraWait"], model)
            n += model
    assert n == 4 * len(range(1105, 1630, 10)), n                # the band was reached


def test_a_leg_without_a_zebra_still_counts_as_drive_signals_documents():
    """The difference drive_signals.py documents: the band is tested at every
    NextJ, and eleven of the thirteen junctions have no zebra. A late box
    hold 1400 cm out from (12300, 12300) counts, where the model's _on_zebra
    says no body is on a zebra. If this fails, that docstring is stale."""
    jx, jy, u = 12300.0, 12300.0, (1.0, 0.0)
    assert all(math.hypot(cx - jx, cy - jy) > 3000.0 for cx, cy in R.CROSSINGS)
    lx, ly = R.left_of(u)
    x, y = jx - 1400.0 * u[0] + lx * R.LANE_OFFSET_CM, jy - 1400.0 * u[1] + ly * R.LANE_OFFSET_CM
    assert not T._on_zebra(SimpleNamespace(st=SimpleNamespace(x=x, y=y), fx=u[0], fy=u[1]))
    assert run_junction(1400.0, Conflict=True, NextJ=(jx, jy, 0.0))["ZebraWait"] == 1


# ------------------------------------------------------------------ the lamps verify_signals expects

LAMPS = parse(SL.render(SL.LAMPS))[0]


def _lamps_run(head, t):
    ctrl = Obj("BP_SignalController", {"Heads": [head], "HOff": [head.vars["off"]],
                                       "HRole": [head.vars["role"]], "HG": [head.vars["g"]],
                                       "HR": [head.vars["r"]], "HShown": head.vars["shown"]})
    Machine(ctrl, {"t": t}).run(LAMPS[3:])


def test_verify_signals_expects_both_slots_as_update_lamps_writes_them():
    """Run the rendered UpdateLamps for five minutes, heads of every kind
    in the plan (vehicle, one-lamp and two-lamp pedestrian), every 0.13 s: the
    two slots it leaves equal verify_signals.expected() for the plan's code
    at that time, from whatever it showed before."""
    kinds = {}
    for p in SL.plan():
        kinds.setdefault((p["role"], p["g"] == p["r"]), p)
    assert set(kinds) >= {(0, False), (1, True)}, set(kinds)
    for p in kinds.values():
        comp = Obj("comp", {"mats": {p["g"]: "MI_jctTrafficLight_Red", p["r"]: "MI_jctTrafficLight_Green"}})
        head = Obj("head", {"StaticMeshComponent": comp, "off": p["off"], "role": p["role"],
                            "g": p["g"], "r": p["r"], "shown": [-1]})
        for k in range(0, 300 * 100, 13):
            t = k / 100.0
            _lamps_run(head, t)
            want = VS.expected(SL.lamp_code(t, p["off"], p["role"]), p["g"], p["r"])
            got = {s: comp.vars["mats"].get(s) for s in want}
            assert got == want, (p["label"], t, got, want)


def test_a_head_that_never_switched_matches_no_code():
    """The mesh default, red AND green lit: the old one-slot check accepted it
    whenever the plan said red or green; both slots accept it never."""
    for kind in ("A", "B", "C"):
        h = {"kind": kind, "label": "x", "junction_cm": [4100.0, 4100.0], "rel_cm": [1530.0, 830.0]}
        p, mats = SL.head_plan(h), SL.slots_of(h)
        assert p is not None and p["role"] == 0, p
        assert not any(VS.shows(mats, VS.expected(c, p["g"], p["r"])) for c in range(6)), kind
        assert mats[p["g"]] == "MI_jctTrafficLight_Green" and mats[p["r"]] == "MI_jctTrafficLight_Red"


def test_a_single_lamp_head_left_alone_shows_a_code_of_the_plan():
    """What the lamp comparison cannot see (verify_signals.py docstring): a
    kind E head has one lamp (g == r), so left alone it matches walk or don't
    walk, and only the driven check (Heads / HShown) catches it."""
    for m, code in (("MI_jctTrafficLight_Green_b", 3), ("MI_jctTrafficLight_Red_b", 4)):
        h = {"kind": "E", "label": "x", "junction_cm": [4100.0, 4100.0], "rel_cm": [1530.0, 830.0],
             "override_slots": [None, m]}
        p, mats = SL.head_plan(h), SL.slots_of(h)
        assert p is not None and p["role"] == 1 and p["g"] == p["r"], p
        assert [c for c in range(6) if VS.shows(mats, VS.expected(c, p["g"], p["r"]))] == [code], m


def test_the_read_window_holds_a_blink_only_across_its_edge():
    p = next(q for q in SL.plan() if q["role"] == 1)
    edge = next(k / 100.0 for k in range(1, 12000)
                if SL.lamp_code(k / 100.0, p["off"], 1) == 5 and SL.lamp_code((k - 1) / 100.0, p["off"], 1) == 3)
    assert VS.codes_between(edge - 0.1, edge + 0.1, p["off"], 1) == {3, 5}
    assert VS.codes_between(edge + 0.05, edge + 0.45, p["off"], 1) == {5}
    assert VS.codes_between(edge - 0.45, edge - 0.05, p["off"], 1) == {3}


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
