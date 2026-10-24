"""
The follow expert held to the VLA's decision rate: the CEILING flight for the
follow fine-tune (demo/decide_rate.py, follow_vlm --decide-every-s P
[--decide-delay-s D], scripts/run_citylife_follow.ps1 -DecideEveryS /
-DecideDelayS).

Run either way:
    pytest tests/test_follow_decide_rate.py -v
    python tests/test_follow_decide_rate.py

WHAT IS AT STAKE (2026-10-24)

The fine-tuned AerialVLA decides once per ~0.8-1.3 s (capture to answer p50
~0.75-0.8 s in flight; one step is the motion over the next 1.0 s). The
hand-written follow acts every 0.1 s and is within 30 m on 0.963 of its ticks
after acquisition; the VLA flights score 0.16-0.24. The ceiling flight asks
how well the EXPERT follows when it may act only at the VLA's rate: near 0.9,
better learning can close the gap; collapsed, the decision rate is the
bottleneck. Its number is read against 0.963, so pinned here:

  1. OFF BY DEFAULT, and the flight unchanged when off: nothing imported or
     built, no row key, no metrics key; the loop's Shield call and the
     perturbers' call site as they were.
  2. THE RULE on a synthetic command sequence (P = 1.0, D = 0.8): a decision
     at the first tick at or after each k*P of the mission clock, carrying
     the expert's own command from D before that tick (the newest stamped at
     or before tick - D; a hover before one exists), held exactly between
     decisions - checked tick by tick, and against a brute-force reference
     on a jittered tick. The grid does not drift with the tick period, a
     stall takes one decision and counts the instants it skipped, and with
     D = 0 and a decision every tick it is the expert itself.
  3. THE REFUSALS: --decide-delay-s alone, a period or delay out of range
     (milliseconds typed as seconds among them), and with --record-perturb,
     --record-perturb-vla or --record-vla.
  4. THE RECORDS: every tick's `decide` (decision index, held, the
     decision's age, the flown command's age and stamp, hover) and the
     metrics block (P, D, decisions, mean hold, hovers, skipped instants,
     the command ages, the rule).
  5. THE SCORE is computed as on any flight: track_truth's scorers give the
     same numbers with or without the rows' `decide`, and the
     frac_within_30m line and its separation are untouched.
  6. THE LAUNCHER passes -DecideEveryS / -DecideDelayS outside the
     -RecordVla block and refuses them alone, out of range, or with
     -RecordPerturb, -RecordPerturbVla or -RecordVla, before the simulator
     starts.

No simulator, GPU, model or network: synthetic ticks only, and follow_vlm's
fly() replaced by a stub wherever main() is asked to refuse, so a refusal that
regressed fails here instead of starting a flight.
"""
from __future__ import annotations

import ast
import contextlib
import io
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.models import Action4D                               # noqa: E402

SKIP = "SKIP"
FOLLOW = ROOT / "demo" / "follow_vlm.py"
PS1 = ROOT / "scripts" / "run_citylife_follow.ps1"
NO_PORT = "127.0.0.1:59999"      # parsed only, never connected to
HOVER = (0.0, 0.0, 0.0, 0.0)


def _DR():
    import decide_rate as DR
    return DR


def _cmd(j: int) -> Action4D:
    """The expert's command at tick j: every axis a different function of j,
    so a command taken from the wrong tick cannot pass for the right one."""
    return Action4D(vx=0.5 + 0.01 * j, vy=-0.02 * j, vz_up=0.001 * j,
                    yaw_rate=0.003 * j - 0.1)


def _v(a) -> tuple:
    return (a.vx, a.vy, a.vz_up, a.yaw_rate)


def _refused(argv: list[str]) -> str:
    """follow_vlm.main(argv) must exit 2 before flying; its stderr. fly() is a
    stub that fails the test, so an accepted argv never starts a flight."""
    import follow_vlm as F

    async def _no_flight(args):
        raise AssertionError(f"{argv} was accepted and would have flown")

    real, F.fly = F.fly, _no_flight
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            F.main(argv)
    except SystemExit as exc:
        assert exc.code == 2, (argv, exc.code, err.getvalue()[-300:])
    else:
        raise AssertionError(f"{argv} was accepted")
    finally:
        F.fly = real
    return err.getvalue()


def _tree():
    src = FOLLOW.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fly = next(n for n in ast.walk(tree) if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    parents = {c: n for n in ast.walk(tree) for c in ast.iter_child_nodes(n)}

    def guarded(node, test):
        p = node
        while p in parents:
            p = parents[p]
            if isinstance(p, ast.If) and test(ast.unparse(p.test)):
                return True
        return False
    return src, tree, fly, guarded


# =========================================================================== #
# 1. Off by default; the flight unchanged when off
# =========================================================================== #
def test_off_by_default_and_the_flight_unchanged_when_off():
    import follow_vlm as F
    a = F.build_arg_parser().parse_args([])
    assert a.decide_every_s is None and a.decide_delay_s is None
    src, tree, fly, guarded = _tree()
    flag = lambda s: "args.decide_every_s" in s                       # noqa: E731
    on = lambda s: "decider is not None" in s or "args.decide_every_s" in s   # noqa: E731
    # decide_rate is imported only under the flag (in main's check and in
    # fly), never at module level, so a default flight never loads it.
    imports = [n for n in ast.walk(tree) if (isinstance(n, ast.ImportFrom)
                                              and n.module == "decide_rate")
               or (isinstance(n, ast.Import) and any(x.name == "decide_rate" for x in n.names))]
    assert len(imports) >= 2, "decide_rate is not wired into follow_vlm"
    for n in imports:
        assert guarded(n, flag), f"line {n.lineno}: decide_rate imported outside its flag"
    # `decider` is None unless the flag built it, and every use is guarded.
    made = [n for n in ast.walk(fly) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "decider" for t in n.targets)]
    none = [n for n in made if isinstance(n.value, ast.Constant) and n.value.value is None]
    assert len(none) == 1 and not guarded(none[0], lambda s: True), "decider starts as None"
    built = [n for n in made if n not in none]
    assert len(built) == 1 and guarded(built[0], flag), [n.lineno for n in built]
    uses = [n for n in ast.walk(fly) if isinstance(n, ast.Attribute)
            and isinstance(n.value, ast.Name) and n.value.id == "decider"]
    assert len(uses) >= 3, "the decider is not wired into fly()"
    for u in uses:
        assert guarded(u, on), f"line {u.lineno}: decider.{u.attr} outside its guard"
    # ONE call, on the expert's `smooth` (after the controller and its slew
    # limiter), at the perturbers' point: after their call, before the Shield,
    # its result the Shield's input.
    calls = [n for n in ast.walk(fly) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == "apply"
             and isinstance(n.func.value, ast.Name) and n.func.value.id == "decider"]
    assert len(calls) == 1, [c.lineno for c in calls]
    c = calls[0]
    assert guarded(c, lambda s: "decider is not None" in s)
    assert isinstance(c.args[0], ast.Name) and c.args[0].id == "smooth"
    assign = next(n for n in ast.walk(fly) if isinstance(n, ast.Assign) and n.value is c)
    tgt = assign.targets[0]
    assert isinstance(tgt, ast.Tuple) and [e.id for e in tgt.elts] == ["flown_in", "decide_rec"]
    pcall = next(n for n in ast.walk(fly) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == "apply"
                 and isinstance(n.func.value, ast.Name) and n.func.value.id == "perturber")
    shield = next(n for n in ast.walk(fly) if isinstance(n, ast.Assign)
                  and ast.unparse(n) == "d = shield.filter(state, flown_in)")
    assert pcall.lineno < c.lineno < shield.lineno, (pcall.lineno, c.lineno, shield.lineno)
    assert src.count("d = shield.filter(state, flown_in)") == 1, "the loop's Shield call moved"
    # The limiter never sees what the hold flies: its state stays the expert's.
    for n in ast.walk(fly):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "limiter":
            names = {x.id for a in n.args for x in ast.walk(a) if isinstance(x, ast.Name)}
            assert not names & {"flown_in", "decide_rec"}, n.lineno
    # A row says "decide", and metrics "decide_rate", only on a held flight.
    rows = [n for n in ast.walk(fly) if isinstance(n, ast.Subscript)
            and isinstance(n.slice, ast.Constant) and n.slice.value == "decide"]
    assert rows and all(guarded(n, lambda s: "decide_rec is not None" in s) for n in rows)
    mets = [n for n in ast.walk(fly) if isinstance(n, ast.Subscript)
            and isinstance(n.slice, ast.Constant) and n.slice.value == "decide_rate"]
    assert mets and all(guarded(n, lambda s: "decider is not None" in s) for n in mets)
    # Built before the simulator is touched, like the DAgger perturber.
    assert built[0].lineno < src[:src.index("client = ProjectAirSimClient()")].count("\n") + 1


# =========================================================================== #
# 2. The rule
# =========================================================================== #
def test_a_decision_every_p_carries_the_experts_command_from_d_before_and_is_held_between():
    DR = _DR()
    h = DR.DecideRate(1.0, 0.8)
    ts = [round(0.1 * j, 6) for j in range(120)]                  # 10 Hz, 12 s
    out = [h.apply(_cmd(j), t) for j, t in enumerate(ts)]
    decided = [j for j, (_, r) in enumerate(out) if not r["held"]]
    assert decided == list(range(0, 120, 10)), decided
    for j, (a, r) in enumerate(out):
        k = j // 10                         # the decision in force
        src = 10 * k - 8                    # the tick 0.8 s before it was taken
        assert r["k"] == k, (j, r)
        assert r["held"] == (j % 10 != 0), (j, r)
        assert math.isclose(r["since_s"], ts[j] - ts[10 * k], abs_tol=1e-6), (j, r)
        if src < 0:
            # The mission's first decision (t = 0): nothing is 0.8 s old yet.
            assert _v(a) == HOVER and r["hover"], (j, r)
            assert r["age_s"] is None and r["cmd_t"] is None, (j, r)
        else:
            assert _v(a) == _v(_cmd(src)), (j, _v(a), src)
            assert not r["hover"] and math.isclose(r["cmd_t"], ts[src]), (j, r)
            assert math.isclose(r["age_s"], ts[j] - ts[src], abs_tol=1e-6), (j, r)
    # Between decisions the SAME command object flies (a zero-order hold).
    for j in range(11, 20):
        assert out[j][0] is out[10][0]
    # The pose is accepted (the perturbers' interface) and not read.
    assert h.apply(_cmd(120), 12.0, pose=(1.0, 2.0, 8.0, 0.3))[1]["k"] == 12


def test_on_a_jittered_tick_it_matches_a_brute_force_reference_and_the_grid_does_not_drift():
    DR = _DR()
    rng = random.Random(5)
    ts, t = [], 0.03
    for _ in range(700):
        ts.append(t)
        t += rng.uniform(0.07, 0.16)                       # 6-14 Hz, a loaded loop
    for every, delay in ((1.0, 0.8), (1.3, 0.75), (0.8, 1.2)):
        h = DR.DecideRate(every, delay)
        k, next_k, held, taken = -1, 0, None, []
        for j, s in enumerate(ts):
            a, r = h.apply(_cmd(j), s)
            if s >= next_k * every:                  # the reference, by brute force
                k, next_k = k + 1, math.floor(s / every) + 1
                old = [i for i in range(j + 1) if ts[i] <= s - delay]
                held = old[-1] if old else None
                taken.append(s)
            assert r["k"] == k and r["held"] == (taken[-1] != s), (every, delay, j, r)
            want = HOVER if held is None else _v(_cmd(held))
            assert _v(a) == want, (every, delay, j)
            if held is not None:
                # The command flown is never younger than D.
                assert r["age_s"] >= delay - 1e-6, (every, delay, j, r)
        st = h.summary()
        assert st["decisions"] == len(taken) and st["missed_instants"] == 0, st
        # Anchored to the grid k*P: the mean hold is P, not P plus the average
        # overshoot of a tick past each instant (~0.06 s here).
        assert abs(st["mean_hold_s"] - every) < 0.01, (every, st["mean_hold_s"])


def test_a_stall_takes_one_decision_and_counts_the_instants_it_skipped():
    DR = _DR()
    h = DR.DecideRate(1.0, 0.8)
    ts = [round(0.1 * j, 6) for j in range(15)] + [3.75, 3.85, 3.95, 4.05, 4.15]
    out = [h.apply(_cmd(j), t) for j, t in enumerate(ts)]
    decided = [ts[j] for j, (_, r) in enumerate(out) if not r["held"]]
    # 0.0 and 1.0 as usual; the stall from 1.4 to 3.75 s passes 2.0 and 3.0,
    # takes ONE decision at 3.75 (instant 3.0; 2.0 skipped), and the grid goes
    # on at 4.0, not at 3.75 + 1.0.
    assert decided == [0.0, 1.0, 3.75, 4.05], decided
    a, r = out[15]
    assert _v(a) == _v(_cmd(14)) and math.isclose(r["age_s"], 3.75 - 1.4), r   # newest <= 2.95
    st = h.summary()
    assert st["decisions"] == 4 and st["missed_instants"] == 1, st


def test_with_no_delay_and_a_decision_every_tick_it_is_the_expert_itself():
    DR = _DR()
    h = DR.DecideRate(0.1, 0.0)
    for j in range(60):
        c = _cmd(j)
        a, r = h.apply(c, round(0.1 * j, 6))       # 0.3 / 0.1 = 2.9999999999999996
        assert a is c and not r["held"] and r["age_s"] == 0.0, (j, r)
    # A pure delay: a decision every tick, each the command from 8 ticks before.
    h = DR.DecideRate(0.1, 0.8)
    for j in range(60):
        a, r = h.apply(_cmd(j), round(0.1 * j, 6))
        assert not r["held"] and _v(a) == (HOVER if j < 8 else _v(_cmd(j - 8))), (j, r)


# =========================================================================== #
# 3. The refusals
# =========================================================================== #
def test_refused_alone_out_of_range_and_with_a_perturbation_or_a_recording():
    DR = _DR()
    for every, delay in ((1.0, 0.8), (1.0, 0.0), (0.05, 10.0), (10.0, 0.0)):
        assert DR.check_args(every, delay) is None, (every, delay)
    for every, delay in ((0.0, 0.0), (-1.0, 0.0), (float("nan"), 0.0), (float("inf"), 0.0),
                         (11.0, 0.0), (1.0, -0.1), (1.0, 800.0), (1.0, float("nan"))):
        why = DR.check_args(every, delay)
        assert why, (every, delay)
        try:
            DR.DecideRate(every, delay)
        except ValueError:
            pass
        else:
            raise AssertionError(f"DecideRate({every}, {delay}) was built")
    for argv, says, not_says in (
            (["--decide-delay-s", "0.8"], "--decide-delay-s needs --decide-every-s", None),
            (["--decide-every-s", "0"], "--decide-every-s", None),
            (["--decide-every-s", "nan"], "--decide-every-s", None),
            (["--decide-every-s", "1000"], "seconds", None),
            (["--decide-every-s", "1", "--decide-delay-s", "800"], "seconds", None),
            (["--decide-every-s", "1", "--decide-delay-s", "-0.1"], "--decide-delay-s", None),
            (["--decide-every-s", "1", "--record-perturb"], "--record-perturb", "labels"),
            (["--decide-every-s", "1", "--record-vla", "d", "--record-perturb"],
             "--record-perturb", "labels"),
            (["--decide-every-s", "1", "--record-vla", "d", "--record-perturb-vla", NO_PORT],
             "--record-perturb-vla", "labels"),
            (["--decide-every-s", "1", "--record-vla", "d"], "--record-vla", None)):
        err = _refused(argv)
        assert says in err, (argv, err[-400:])
        if not_says:
            assert not_says not in err, (argv, err[-400:])
    import follow_vlm as F
    a = F.build_arg_parser().parse_args(["--decide-every-s", "1.0", "--decide-delay-s", "0.8"])
    assert a.decide_every_s == 1.0 and a.decide_delay_s == 0.8


# =========================================================================== #
# 4. The records
# =========================================================================== #
def test_every_tick_records_its_decision_and_the_summary_says_the_rule():
    DR = _DR()
    h = DR.DecideRate(1.0, 0.8)
    recs = [h.apply(_cmd(j), round(0.1 * j, 6))[1] for j in range(100)]      # 0 .. 9.9 s
    assert set(recs[0]) == {"k", "held", "since_s", "age_s", "cmd_t", "hover"}, recs[0]
    assert recs[0] == {"k": 0, "held": False, "since_s": 0.0, "age_s": None, "cmd_t": None,
                       "hover": True}
    assert recs[17] == {"k": 1, "held": True, "since_s": 0.7, "age_s": 1.5, "cmd_t": 0.2,
                        "hover": False}, recs[17]
    json.dumps(recs)
    st = h.summary()
    json.dumps(st)
    assert st["every_s"] == 1.0 and st["delay_s"] == 0.8, st
    assert st["ticks"] == 100 and st["decisions"] == 10 and st["held_ticks"] == 90, st
    assert st["hover_decisions"] == 1 and st["missed_instants"] == 0, st
    assert st["mean_hold_s"] == 1.0 and st["mean_hold_ticks"] == 10.0, st
    # The age of the expert's command: D at each decision (0.8 on this exact
    # tick), and 0.8-1.7 s while it is held - 1.25 s on average.
    assert st["decision_age_s_mean"] == 0.8, st
    assert st["flown_age_s_mean"] == 1.25 and st["flown_age_s_max"] == 1.7, st
    for word in ("Shield", "slew limiter", "hover", "mission", "zero-order hold"):
        assert word in st["rule"], (word, st["rule"])
    d = h.describe()
    assert "1 s" in d and "0.8 s" in d and "Shield" in d, d
    # An empty run says so rather than dividing by zero.
    e = DR.DecideRate(1.0, 0.8).summary()
    assert e["decisions"] == 0 and e["mean_hold_s"] is None and e["flown_age_s_mean"] is None


# =========================================================================== #
# 5. The follow score is computed as on any flight
# =========================================================================== #
def test_the_follow_score_is_computed_as_on_any_flight():
    import track_truth as TT
    DR = _DR()
    rows = []
    for i in range(80):
        x, tx, ty = 1.5 * i, 1.5 * i + 18.0 + 4.0 * math.sin(i / 9.0), 3.0 * math.cos(i / 7.0)
        brg = math.atan2(ty, tx - x)
        rows.append({"t": round(0.1 * i, 3), "tick": i + 1, "x": x, "y": 0.0, "up": 8.0,
                     "psi": 0.0, "seen": i % 4 != 3,
                     "det": (None if i % 4 == 3 else
                             {"cx": 384 + 384 * math.tan(brg) + (i % 5 - 2) * 6.0, "cy": 260.0,
                              "w": 60.0, "score": 0.3, "colour": 0.5, "img_w": 768,
                              "switched": False}),
                     "est_xy": [tx + 1.0, ty - 0.5],
                     "truth": {"class": "car", "pts": [[tx, ty]], "names": ["Car_10"]},
                     "tgt_x": tx, "tgt_y": ty})
    h = DR.DecideRate(1.0, 0.8)
    held = [dict(r, decide=h.apply(_cmd(i), r["t"])[1]) for i, r in enumerate(rows)]
    base = TT.score_rows(rows)
    assert base["det_scored"] > 0, base                  # a comparison of something
    for fn in (TT.score_rows, TT.score_estimate, TT.score_instance):
        assert fn(rows) == fn(held), fn.__name__
    # fly()'s separation and frac_within_30m: the same expressions, which read
    # the rows' position and truth only.
    src, _tree_, fly, _g = _tree()
    line = ('"frac_within_30m": round(float(np.mean([s <= 30 for s in seps])), 3) '
            'if seps else None,')
    assert src.count(line) == 1, "frac_within_30m is no longer computed as before"
    assert src.count("seps = [s for s in (_sep(r) for r in rows) if s is not None]") == 1
    sep = next(n for n in ast.walk(fly) if isinstance(n, ast.FunctionDef) and n.name == "_sep")
    assert "decide" not in ast.unparse(sep)


# =========================================================================== #
# 6. The launcher
# =========================================================================== #
def test_the_launcher_passes_the_rate_and_refuses_it_before_the_simulator():
    src = PS1.read_text(encoding="utf-8")
    for p in ("[double]$DecideEveryS", "[double]$DecideDelayS"):
        assert p in src, p
    lines = src.splitlines()
    for flag, guard in (('"--decide-every-s"', "if ($DecideEveryS)"),
                        ('"--decide-delay-s"', "if ($DecideDelayS)")):
        assert src.count(flag) == 1, flag
        fi = next(n for n, ln in enumerate(lines) if flag in ln)
        gi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith(guard))
        ind = len(lines[gi]) - len(lines[gi].lstrip())
        for ln in lines[gi + 1:fi + 1]:
            assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, (flag, ln)
    # -DecideDelayS only inside -DecideEveryS's block, and that block at the
    # top level: not inside -RecordVla's (a ceiling flight records nothing).
    ge = next(n for n, ln in enumerate(lines) if ln.startswith("if ($DecideEveryS) {"))
    gd = next(n for n, ln in enumerate(lines) if ln.lstrip().startswith("if ($DecideDelayS) {"))
    assert ge < gd and lines[gd].startswith("    "), (ge, gd)
    sim = src.index("if (-not $SkipSim) { Start-Sim }")
    for refusal in ("if ($PSBoundParameters.ContainsKey('DecideDelayS') -and -not $DecideEveryS)",
                    "if ($DecideEveryS -and ($RecordPerturb -or $RecordPerturbVla))",
                    "if ($DecideEveryS -and $RecordVla)",
                    "if ($PSBoundParameters.ContainsKey('DecideEveryS') -and "
                    "-not ($DecideEveryS -gt 0 -and $DecideEveryS -le 10))",
                    "if ($PSBoundParameters.ContainsKey('DecideDelayS') -and "
                    "-not ($DecideDelayS -ge 0 -and $DecideDelayS -le 10))"):
        assert refusal in src and src.index(refusal) < sim, refusal
    assert "-DecideEveryS 1.0 -DecideDelayS 0.8" in src, "the header shows a ceiling flight"
    import follow_vlm as F
    a = F.build_arg_parser().parse_args(["--decide-every-s", "1", "--decide-delay-s", "0.8"])
    assert a.decide_every_s == 1.0 and a.decide_delay_s == 0.8


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
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
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
