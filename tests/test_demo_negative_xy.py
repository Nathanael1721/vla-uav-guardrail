"""The demo flight CLIs take a negative first value in their X,Y options.

    python tests/test_demo_negative_xy.py      (no pytest, no simulator)

WHY THIS FILE EXISTS

argparse takes a separate value that starts with '-' only when it reads as a
negative NUMBER, and "-20,15" does not (the comma). So
`demo/follow_vlm.py --park-at -20,15` - a car parked west of the origin - and
`demo/semantic_seek.py --target-scale -1,1,1` (or `--target-xy -38,26`, a
target south of it) exited with status 2 ("expected one argument") before
anything flew. The rails (sitl/mavlink_adapter_node.join_negative_xy) and
the costmap CLI (guardrail/compiler.py) were fixed on 2026-10-19; these two
were left out of that scope (review of F8-harness-tools, fixer's not_fixed).
Both now parse through guardrail.compiler.join_negative_xy with their own
flags. The `=` spelling, which always parsed, is the null.

Shown failing before the fix: neither script had parse_cli(), and main()
took no argv. Each test runs main() itself with the flight stubbed out, so
what is checked is the parse the real entry point does.
"""
from __future__ import annotations

import asyncio
import contextlib
import io
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

SKIP = "SKIP"


def _import(name):
    """The demo module, or None when this host cannot import it (it reads
    numpy, shapely and the guardrail package at import, not the simulator)."""
    try:
        return __import__(name)
    except ImportError as e:
        print(f"      (cannot import demo/{name}.py here: {e})")
        return None


def _exit_code(fn):
    """fn()'s SystemExit code with argparse's message swallowed, or None."""
    with contextlib.redirect_stderr(io.StringIO()), \
            contextlib.redirect_stdout(io.StringIO()):
        try:
            fn()
        except SystemExit as e:
            return e.code
    return None


def _main_args(mod, argv, **stubs):
    """mod.main(argv) with mod.fly (and any `stubs`) replaced: the args the
    flight would have been handed, and main's return code."""
    seen = []

    async def no_flight(args):
        seen.append(args)
        return 0
    saved = {k: getattr(mod, k) for k in ["fly", *stubs]}
    try:
        mod.fly = no_flight
        for k, v in stubs.items():
            setattr(mod, k, v)
        with contextlib.redirect_stdout(io.StringIO()):
            rc = mod.main(argv)
    finally:
        for k, v in saved.items():
            setattr(mod, k, v)
    assert len(seen) == 1, "main() did not reach the flight"
    return seen[0], rc


def test_follow_vlm_parks_the_car_at_a_negative_x():
    F = _import("follow_vlm")
    if F is None:
        return SKIP
    # The defect: the plain parser refuses the separate negative value.
    assert _exit_code(lambda: F.build_arg_parser().parse_args(
        ["--park-at", "-20,15"])) == 2, "argparse no longer needs the join?"
    timers = {"_fine_timer_begin": lambda: None, "_fine_timer_end": lambda w: None}
    args, rc = _main_args(F, ["--park-at", "-20,15"], **timers)
    assert rc == 0 and args.park_at == "-20,15", (rc, args.park_at)
    eq, _ = _main_args(F, ["--park-at=-20,15"], **timers)
    assert vars(eq) == vars(args), "the = spelling parses differently"
    # A negative fractional X too; an option after the flag is never taken
    # as its value, and a negative NUMBER option is left to argparse.
    assert F.join_negative_xy(["--park-at", "-.5,2", "--orbit-speed", "-1.5"],
                              F.XY_FLAGS) == ["--park-at=-.5,2", "--orbit-speed", "-1.5"]
    assert _exit_code(lambda: F.parse_cli(F.build_arg_parser(),
                                          ["--park-at", "--tag", "x"])) == 2


def test_semantic_seek_takes_a_negative_target_scale_and_target_xy():
    S = _import("semantic_seek")
    if S is None:
        return SKIP
    for flag, value in (("--target-scale", "-1,1,1"), ("--target-xy", "-38,26")):
        assert _exit_code(lambda: S.build_arg_parser().parse_args([flag, value])) == 2, flag
    argv = ["--target-scale", "-1,1,1", "--target-xy", "-38,26", "--tag", "t"]
    args, rc = _main_args(S, argv)
    assert rc == 0, rc
    assert (args.target_scale, args.target_xy, args.tag) == ("-1,1,1", "-38,26", "t"), vars(args)
    eq, _ = _main_args(S, ["--target-scale=-1,1,1", "--target-xy=-38,26", "--tag", "t"])
    assert vars(eq) == vars(args), "the = spelling parses differently"
    # the values the flight splits are the ones written
    assert [float(v) for v in args.target_scale.split(",")] == [-1.0, 1.0, 1.0]
    assert _exit_code(lambda: S.parse_cli(["--target-xy", "--tag", "x"])) == 2


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
