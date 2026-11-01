"""tools/citylife_mcp/car_speed.py: set one CityLife car's top speed and prove
the change reached the level file.

Run either way:
    pytest tests/test_car_speed.py -v
    python tests/test_car_speed.py

The level is gitignored and the editor is not available here, so a fake editor
stands in for ue_rpc.run_script: it parses the WANT the real script would carry,
applies it to its own table and, when told to, touches the stand-in .umap the
way a save does. What these guard against: a save_assets that returns True
while the file stays locked (it happened after the disk ran full), a set that
the editor silently ignores, and a speed written to the wrong car.
"""
import contextlib
import io
import json
import os
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.citylife_mcp import car_speed as CS               # noqa: E402

FLEET = {"Car_07": 560.0, "Car_10": 320.0, "Car_22": 505.0}


class FakeEditor:
    def __init__(self, umap, saves=True, obeys=True):
        self.speed = dict(FLEET)
        self.umap, self.saves, self.obeys = umap, saves, obeys
        self.sets = []

    def run_script(self, code):
        compile(code, "<editor script>", "exec")       # the substituted script must parse
        lit = re.search(r"WANT = json\.loads\((\".*\")\)", code).group(1)
        want = json.loads(json.loads(lit))
        for car in want:
            if car not in self.speed:
                return {"error": "MISSING " + car}
        for car, v in want.items():
            self.sets.append((car, v))
            if self.obeys:
                self.speed[car] = v
        if want and self.saves:
            t = os.path.getmtime(self.umap) + 10
            os.utime(self.umap, (t, t))
        return {"speed": dict(self.speed), "saved": True if want else None}


def _run(argv, **kw):
    with tempfile.TemporaryDirectory() as d:
        umap = os.path.join(d, "CityLife_Day.umap")
        Path(umap).write_bytes(b"level")
        ed = FakeEditor(umap, **kw)
        old = (CS.ue_rpc.run_script, CS.UMAP, sys.argv)
        CS.ue_rpc.run_script, CS.UMAP, sys.argv = ed.run_script, umap, ["car_speed"] + argv
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):
                CS.main()
            err = None
        except SystemExit as e:
            err = str(e)
        finally:
            CS.ue_rpc.run_script, CS.UMAP, sys.argv = old
        return out.getvalue(), err, ed


def test_without_arguments_it_only_reads():
    out, err, ed = _run([])
    assert err is None and ed.sets == []
    assert "Car_07 560.0" in out and "Car_22 505.0" in out


def test_a_set_prints_the_old_value_and_checks_the_file():
    out, err, ed = _run(["Car_07", "320"])
    assert err is None, err
    assert "Car_07 SpeedCmS was 560.0" in out
    assert "now 320.0" in out and "rewritten" in out
    assert ed.sets == [("Car_07", 320.0)]
    assert ed.speed["Car_10"] == 320.0 and ed.speed["Car_22"] == 505.0


def test_a_save_that_never_reaches_the_disk_fails():
    out, err, _ = _run(["Car_07", "320"], saves=False)
    assert err and "NOT rewritten" in err, err


def test_a_set_the_editor_ignores_fails():
    out, err, _ = _run(["Car_07", "320"], obeys=False)
    assert err and "READBACK MISMATCH" in err, err


def test_an_unknown_car_is_refused_before_anything_is_written():
    out, err, ed = _run(["Car_99", "320"])
    assert err and "Car_99" in err, err
    assert ed.sets == []


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
