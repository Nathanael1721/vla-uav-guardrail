"""
WP1-14: the six legacy AirSim flight scripts fly WGS84 policies only (2026-11-21).

    python tests/test_legacy_rails_wgs84_only.py      (no pytest needed)

The grant (Policy DSL page): "WGS84 latitude / longitude is canonical ... the
DSL itself never carries projected coordinates". bundle.load_for_flight has
refused a policy stored in the legacy metre form since 2026-11-18, but six
flight scripts read their --policy with models.load_policy and so flew one
without a word: demo/run_demo.py, demo/real_vla_demo.py, demo/aerialvla_demo.py,
demo/aerialvla_pas_demo.py, demo/semantic_demo.py and demo/semantic_seek.py
(bundle.LEGACY_YAML_RAILS). Each now loads through its load_flight_policy
(bundle.refuse_or_load_yaml -> load_yaml_for_flight -> load_for_flight). This
file imports each script's function (no simulator is started) and checks:

  * a metre policy is refused with the reason (SystemExit naming the legacy
    form), and load_yaml_for_flight raises LegacyPointFormRefused;
  * a WGS84 policy loads as models.load_policy loaded it (same hash, no
    legacy key in the record), for each script's own default --policy too;
  * VLAGUARD_LEGACY_READ=1 re-runs a recorded flight under the metre policy,
    announced on the console and recorded (real_vla_demo puts the keys in
    metrics.json; a WGS84 run's metrics gain no key);
  * reading the source: the flight function calls load_flight_policy, never
    models.load_policy, and before it creates or clears its output folder;
    no demo script reads `load_policy(args.` any more.
"""
from __future__ import annotations

import ast
import contextlib
import copy
import io
import os
import sys
import tempfile
import types
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail.bundle import (LEGACY_READ_ENV, LEGACY_YAML_RAILS,   # noqa: E402
                              LegacyPointFormRefused, load_yaml_for_flight)
from guardrail.models import (POINTS_LEGACY_METRES, POINTS_WGS84,   # noqa: E402
                              ProjectedCoordinatesRefused, load_legacy_metre_policy,
                              load_policy)

RULE = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "project_fix"}
# A policy written only in metres, the legacy read form of the pre-3-Nov
# bundles (as tests/test_wgs84_frameless_writers.py METRE_DOC).
METRE_DOC = {"policy_id": "legacy-metre", "version": "0.0.1", "constraints": [
    {"id": "fence", "type": "polygon_fence", **RULE, "margin_m": 1.0,
     "vertices": [{"x": 10, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 10}]},
    {"id": "alt", "type": "altitude_envelope", **RULE, "alt_min_m": 2,
     "alt_max_m": 60}]}
WGS84 = ROOT / "policies" / "sim_demo_policy.yaml"
LEGACY_KEYS = {"legacy_read", "point_form", "point_note"}
# The function of each script that loads the policy and flies.
FLIGHT_FN = {"run_demo": "main", "real_vla_demo": "main", "aerialvla_demo": "main",
             "aerialvla_pas_demo": "fly", "semantic_demo": "fly",
             "semantic_seek": "fly"}


@contextlib.contextmanager
def _legacy_read(value: str | None):
    saved = os.environ.get(LEGACY_READ_ENV)
    if value is None:
        os.environ.pop(LEGACY_READ_ENV, None)
    else:
        os.environ[LEGACY_READ_ENV] = value
    try:
        yield
    finally:
        if saved is None:
            os.environ.pop(LEGACY_READ_ENV, None)
        else:
            os.environ[LEGACY_READ_ENV] = saved


@contextlib.contextmanager
def _metre_yaml():
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "legacy-metre.yaml"
        p.write_text(yaml.safe_dump(copy.deepcopy(METRE_DOC), sort_keys=False),
                     encoding="utf-8")
        yield p


def _modules() -> dict:
    """Each legacy rail's module, imported (run_demo imports airsim at the
    top; a stand-in is enough, nothing here talks to a simulator)."""
    if "airsim" not in sys.modules:
        try:
            import airsim  # noqa: F401
        except ImportError:
            sys.modules["airsim"] = types.ModuleType("airsim")
    return {Path(rel).stem: __import__(Path(rel).stem) for rel in LEGACY_YAML_RAILS}


def test_the_six_rails_are_named():
    assert {Path(r).stem for r in LEGACY_YAML_RAILS} == set(FLIGHT_FN), LEGACY_YAML_RAILS
    for rel in LEGACY_YAML_RAILS:
        assert (ROOT / rel).is_file(), rel


def test_the_shared_loader_refuses_metres_and_reads_them_only_as_a_read():
    with _metre_yaml() as p, _legacy_read(None):
        # what flew before; since 2026-11-26 load_policy refuses it too, and
        # only the declared legacy read reads it
        assert load_legacy_metre_policy(p).point_form == POINTS_LEGACY_METRES
        try:
            load_policy(p)
        except ProjectedCoordinatesRefused as e:
            assert "WP1-14" in str(e), e
        else:
            raise AssertionError("load_policy read a metre policy")
        try:
            load_yaml_for_flight(p, say=lambda s: None)
        except LegacyPointFormRefused as e:
            assert "local-metres" in str(e) and LEGACY_READ_ENV in str(e), e
        else:
            raise AssertionError("load_yaml_for_flight flew a metre policy")
        said = []
        with _legacy_read("1"):
            pol, rec = load_yaml_for_flight(p, say=said.append)
        assert pol.point_form == POINTS_LEGACY_METRES
        assert rec["legacy_read"] is True and rec["point_form"] == POINTS_LEGACY_METRES
        assert len(said) == 1 and "LEGACY READ" in said[0], said


def test_each_rail_refuses_a_metre_policy():
    mods = _modules()
    with _metre_yaml() as p, _legacy_read(None):
        for name, mod in mods.items():
            fn = getattr(mod, "load_flight_policy", None)
            assert callable(fn), f"demo/{name}.py has no load_flight_policy"
            try:
                fn(str(p))
            except SystemExit as e:
                msg = str(e.code)
                assert "REFUSING TO FLY" in msg and "local-metres" in msg, (name, msg)
                assert "WP1-14" in msg and LEGACY_READ_ENV in msg, (name, msg)
            else:
                raise AssertionError(f"demo/{name}.py flew a metre policy")


def test_each_rail_flies_a_wgs84_policy_as_before():
    mods = _modules()
    want = load_policy(WGS84)
    assert want.point_form == POINTS_WGS84
    with _legacy_read(None):
        for name, mod in mods.items():
            pol, rec = mod.load_flight_policy(str(WGS84))
            assert pol.policy_hash == want.policy_hash, name
            assert pol.point_form == POINTS_WGS84, name
            assert rec["kind"] == "yaml" and not LEGACY_KEYS & set(rec), (name, rec)


def test_each_rails_default_policy_still_flies():
    """Each script's own --policy default (read from its source) loads
    through the new path with the hash load_policy gave it, so the scripts'
    default runs are unchanged."""
    mods = _modules()
    with _legacy_read(None):
        for name, mod in mods.items():
            src = Path(mod.__file__).read_text(encoding="utf-8")
            tree = ast.parse(src)
            default = None
            for node in ast.walk(tree):
                if (isinstance(node, ast.Call) and getattr(node.func, "attr", "") ==
                        "add_argument" and node.args and
                        isinstance(node.args[0], ast.Constant) and
                        node.args[0].value == "--policy"):
                    for kw in node.keywords:
                        if kw.arg == "default":
                            # ROOT / "policies" / "<name>.yaml": its strings in source order
                            parts = [c.value for c in sorted(
                                (c for c in ast.walk(kw.value) if isinstance(c, ast.Constant)
                                 and isinstance(c.value, str)),
                                key=lambda c: (c.lineno, c.col_offset))]
                            default = ROOT.joinpath(*parts)
            assert default is not None and default.is_file(), (name, default)
            pol, rec = mod.load_flight_policy(str(default))
            assert pol.policy_hash == load_policy(default).policy_hash, (name, default)
            assert not LEGACY_KEYS & set(rec), (name, rec)


def test_each_rail_reads_a_metre_policy_under_the_legacy_switch():
    mods = _modules()
    with _metre_yaml() as p, _legacy_read("1"):
        for name, mod in mods.items():
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                pol, rec = mod.load_flight_policy(str(p))
            assert pol.point_form == POINTS_LEGACY_METRES, name
            assert rec["legacy_read"] is True, (name, rec)
            assert "LEGACY READ" in out.getvalue(), (name, out.getvalue())


def test_real_vla_demo_records_a_legacy_read_in_its_metrics():
    rv = _modules()["real_vla_demo"]
    with _legacy_read(None):
        _, rec = rv.load_flight_policy(str(WGS84))
    assert rv.legacy_point_keys(rec) == {}
    with _metre_yaml() as p, _legacy_read("1"), contextlib.redirect_stdout(io.StringIO()):
        _, rec = rv.load_flight_policy(str(p))
    keys = rv.legacy_point_keys(rec)
    assert keys["legacy_read"] is True and keys["point_form"] == POINTS_LEGACY_METRES
    assert set(keys) == LEGACY_KEYS, keys
    src = (ROOT / "demo" / "real_vla_demo.py").read_text(encoding="utf-8")
    assert "**legacy_point_keys(policy_source)" in src


def _calls(fn: ast.AST) -> list[tuple[int, str]]:
    out = []
    for n in ast.walk(fn):
        if isinstance(n, ast.Call):
            f = n.func
            name = f.id if isinstance(f, ast.Name) else getattr(f, "attr", "")
            out.append((n.lineno, name))
    return sorted(out)


def test_each_rail_loads_through_the_flight_loader_before_writing():
    for rel in LEGACY_YAML_RAILS:
        name = Path(rel).stem
        tree = ast.parse((ROOT / rel).read_text(encoding="utf-8"))
        fns = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
               and n.name == FLIGHT_FN[name]]
        assert len(fns) == 1, (rel, FLIGHT_FN[name])
        calls = _calls(fns[0])
        names = [c for _, c in calls]
        assert "load_policy" not in names, f"{rel} still flies models.load_policy"
        assert names.count("load_flight_policy") == 1, (rel, names)
        at = next(ln for ln, c in calls if c == "load_flight_policy")
        writes = [ln for ln, c in calls if c in ("mkdir", "unlink", "write_text")]
        assert writes and at < min(writes), (
            f"{rel} creates or clears its output before the policy is accepted")


def test_no_demo_script_flies_its_policy_through_load_policy():
    for p in sorted((ROOT / "demo").glob("*.py")):
        src = p.read_text(encoding="utf-8")
        assert "load_policy(args." not in src, f"{p.name} reads its --policy with load_policy"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
