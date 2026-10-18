"""guardrail/vla_backends.py: the swappable slot's factory and its skeleton.

Run either way:
    pytest tests/test_vla_backends.py -v
    python tests/test_vla_backends.py

WHAT IS BEING CLAIMED

AeroVLABackend is a skeleton for a real camera+language VLA. Unfilled, it
must fail loudly on the first tick that would run the model, never hover on
zero commands while the run looks like a VLA run. Its action-chunking cadence
- run the model on ticks 1, 1+N, 1+2N, ... and hold the last action between -
is finished code that a filled-in backend inherits, so it is tested here
through a subclass that overrides only `_infer`.

(tests/test_vla_backend_table.py is a different module: tools/vla_backend_table.py.)
No model, GPU or simulator is needed.
"""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.models import Action4D, State                   # noqa: E402
from guardrail.vla_backends import AeroVLABackend, make_backend  # noqa: E402

SKIP = "SKIP"


def _frame():
    return [[[0, 0, 0]]]


class _Counting(AeroVLABackend):
    """A filled-in backend as the class docstring asks for one: only `_infer`
    is overridden, so `act` (the cadence) is the real code."""

    def __init__(self, vla_hz: float):
        super().__init__("follow the red car", _frame, "nowhere", vla_hz=vla_hz)
        self.calls: list[int] = []

    def _infer(self, frame, state):
        self.calls.append(self._tick)
        return Action4D(vx=float(self._tick))


def test_the_unfilled_skeleton_raises_instead_of_hovering():
    """make_backend('aerovla') built a backend whose act() returned Action4D()
    (hover) on every tick, whatever model_dir said. Shown failing before
    2026-10-19: act() returned vx=vy=vz_up=yaw_rate=0 without a word."""
    mission = SimpleNamespace(task_text="follow the red car")
    be = make_backend("aerovla", mission, None, get_frame=_frame, model_dir="nowhere")
    try:
        got = be.act(State(x=0.0, y=0.0, up=10.0))
    except NotImplementedError as exc:
        assert "skeleton" in str(exc), exc
    else:
        raise AssertionError(f"the unfilled skeleton returned {got!r}")


def test_the_model_runs_on_ticks_1_1_plus_n_and_holds_in_between():
    """`tick % every != 1` held the last action on EVERY tick when every == 1
    (vla_hz above ~6.7, the 10 Hz contract rate included): the model never
    ran. Shown failing before 2026-10-19 for vla_hz=10: no call at all."""
    st = State(x=0.0, y=0.0, up=10.0)
    for hz, want in ((10.0, list(range(1, 11))), (3.0, [1, 4, 7, 10]),
                     (5.0, [1, 3, 5, 7, 9]), (2.0, [1, 6])):
        be = _Counting(hz)
        out = [be.act(st) for _ in range(10)]
        assert be.calls == want, (hz, be.calls)
        # Each held tick returns the action from the last model call.
        for tick, a in enumerate(out, start=1):
            last = max(c for c in want if c <= tick)
            assert a.vx == float(last), (hz, tick, a)


def test_the_factory_forwards_the_rate():
    """make_backend dropped vla_hz, so through the factory the rate was always
    the 3 Hz default."""
    mission = SimpleNamespace(task_text="x")
    be = make_backend("aerovla", mission, None, get_frame=_frame,
                      model_dir="nowhere", vla_hz=10.0)
    assert be._every == 1
    assert make_backend("aerovla", mission, None, get_frame=_frame,
                        model_dir="nowhere")._every == 3


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
