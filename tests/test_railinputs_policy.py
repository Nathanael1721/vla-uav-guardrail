"""Signed flight bundles, and a YAML policy only under --dev-yaml.

    python tests/test_railinputs_policy.py      (no pytest needed)

Until 2026-10-12 both SITL rails flew policies/sim_demo_policy.yaml, unsigned,
whenever no --bundle was given. Now a YAML flies only with --dev-yaml
(recorded in the policy source and as a reason the run is not KPI-grade), and
with no policy option the signed bundle of the default policy flies when
tools/build_flight_bundles.py has put it in bundles/. The bundles are built
here with a throw-away key and a trust store that trusts only it, never with
the lab key.

Shown failing first against a pristine HEAD 4418ae0 copy (git archive):
guardrail.manifest has no flight_policy_choice / flight_bundle_path, there is
no tools/build_flight_bundles.py, and run_sitl_demo.py flies `--policy P`
without --dev-yaml (the refusal test sees the flight complete).
"""
from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))
sys.path.insert(0, str(ROOT / "tests"))
# demo/ last, for demo/follow_vlm.py (the city rail's policy choice) only.
sys.path.append(str(ROOT / "demo"))

from guardrail import bundle as B                                    # noqa: E402
from guardrail import load_policy                                    # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _tool():
    spec = importlib.util.spec_from_file_location(
        "build_flight_bundles_unit", ROOT / "tools" / "build_flight_bundles.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _test_key():
    import os
    d = Path(tempfile.mkdtemp(prefix="flightkey_"))
    trust = d / "trust.json"
    if B.HAVE_CRYPTO:
        signer = B.generate_signing_key(d / "k.pem", authority="test-lab",
                                        trust_store=trust)
    else:
        signer = B.Signer(authority="test-lab",
                          private_key=B._PyEd25519TestKey(os.urandom(32)))
        B.register_public_key(signer, trust)
    return signer, trust


def test_the_policy_choice_flies_yaml_only_under_dev_yaml():
    from guardrail.manifest import flight_policy_choice
    d = Path(tempfile.mkdtemp(prefix="choice_"))
    dflt = d / "dflt.tar.gz"
    kw = dict(default_yaml=POLICY, default_bundle=dflt)
    try:
        flight_policy_choice(None, "p.yaml", dev_yaml=False, **kw)
    except ValueError as e:
        assert "--dev-yaml" in str(e) and "build_flight_bundles" in str(e), e
    else:
        raise AssertionError("a YAML policy flew without --dev-yaml")
    b, y, rec = flight_policy_choice(None, "p.yaml", dev_yaml=True, **kw)
    assert (b, y) == (None, "p.yaml") and rec["dev_yaml"] is True
    b, y, rec = flight_policy_choice("x.tar.gz", None, dev_yaml=False, **kw)
    assert (b, y, rec) == ("x.tar.gz", None, {})
    try:
        flight_policy_choice("x.tar.gz", None, dev_yaml=True, **kw)
    except ValueError as e:
        assert "give one" in str(e)
    else:
        raise AssertionError("--bundle with --dev-yaml was accepted")
    # Neither: the default bundle only when it is on disk.
    try:
        flight_policy_choice(None, None, dev_yaml=False, **kw)
    except ValueError as e:
        assert "not on disk" in str(e) and "--dev-yaml" in str(e)
    else:
        raise AssertionError("no policy and no bundle flew anyway")
    dflt.write_bytes(b"x")
    b, y, rec = flight_policy_choice(None, None, dev_yaml=False, **kw)
    assert (b, y, rec) == (str(dflt), None, {"default_bundle": True})
    b, y, rec = flight_policy_choice(None, None, dev_yaml=True, **kw)
    assert (b, y) == (None, str(POLICY)) and rec["dev_yaml"]
    # Both: handed on, so load_for_flight refuses them in its own words.
    assert flight_policy_choice("b", "y", dev_yaml=False, **kw)[:2] == ("b", "y")


def test_the_flight_bundle_is_named_as_the_bundle_cli_names_it():
    from guardrail.manifest import flight_bundle_path
    pol = load_policy(POLICY)
    assert flight_bundle_path(POLICY).name == f"{pol.policy_id}-v{pol.version}.tar.gz"
    assert flight_bundle_path(pol, "/x").parent == Path("/x")
    assert flight_bundle_path(POLICY).parent == ROOT / "bundles"


def test_the_tool_builds_signs_and_verifies_every_flight_bundle():
    T = _tool()
    signer, trust = _test_key()
    out = Path(tempfile.mkdtemp(prefix="flightbundles_"))
    rows = T.build(T.FLIGHT_POLICIES, out, signer=signer, trust_store=trust,
                   issued_at="2026-10-12T00:00:00Z")
    assert len(rows) == len(T.FLIGHT_POLICIES) == 8 and all(r["ok"] for r in rows), rows
    for r in rows:
        pol, src = B.load_for_flight(r["bundle"], trust_store=trust)
        assert src["signature"] == B.SIG_VERIFIED, src
        assert pol.policy_hash == load_policy(ROOT / r["policy"]).policy_hash
    names = sorted(Path(r["bundle"]).name for r in rows)
    assert "fase3-sim-demo-v0.1.1.tar.gz" in names, names
    # Same issue time -> same bytes (bundles are byte-derivable build products).
    again = Path(tempfile.mkdtemp(prefix="flightbundles_"))
    T.build(["sim_demo_policy"], again, signer=signer, trust_store=trust,
            issued_at="2026-10-12T00:00:00Z")
    a = out / "fase3-sim-demo-v0.1.1.tar.gz"
    assert a.read_bytes() == (again / a.name).read_bytes()
    # --check: OK here, MISSING in an empty folder, BAD under another trust store.
    assert all(T.check_one(n, out, trust)["ok"] for n in T.FLIGHT_POLICIES)
    empty = Path(tempfile.mkdtemp(prefix="flightbundles_"))
    assert T.check_one("sim_demo_policy", empty, trust)["why"] == "no bundle on disk"
    _, other = _test_key()
    assert not T.check_one("sim_demo_policy", out, other)["ok"]


def test_the_tool_never_writes_an_unsigned_or_untrusted_flight_bundle():
    T = _tool()
    signer, _ = _test_key()
    unregistered = B.Signer(authority="unregistered", private_key=signer.private_key)
    out = Path(tempfile.mkdtemp(prefix="flightbundles_"))
    try:
        T.build(["sim_demo_policy"], out, signer=unregistered)
    except ValueError as e:
        assert "trust store" in str(e)
    else:
        raise AssertionError("an unregistered key signed a flight bundle")
    assert not list(out.glob("*.tar.gz"))


def test_the_pymavlink_rail_refuses_a_yaml_policy_without_dev_yaml():
    import test_sitl_rails as T
    R = T._load(ROOT / "sitl" / "run_sitl_demo.py", "run_sitl_demo_policy_unit",
                {} if importlib.util.find_spec("pymavlink") else
                {"pymavlink": _fake_pymavlink()})
    tmp = Path(tempfile.mkdtemp(prefix="sitl_policy_"))
    R.ROOT = tmp
    with T._argv(["run_sitl_demo.py", "--policy", str(POLICY), "--tag", "t"]):
        try:
            R.main()
        except ValueError as e:
            assert "--dev-yaml" in str(e), e
        else:
            raise AssertionError("the rail flew a YAML policy without --dev-yaml")
    assert not (tmp / "demo" / "out" / "t").exists(), (
        "the refused run touched its output folder")


def _fake_pymavlink():
    import types
    pm = types.ModuleType("pymavlink")
    pm.mavutil = types.SimpleNamespace()
    return pm


def test_a_dev_yaml_run_says_so_everywhere_and_is_not_kpi_grade():
    import test_sitl_rails as T
    out, metrics, rows, events, sim, R = T._fly_pymavlink_rail(["--shield", "on"])
    assert metrics["policy_source"]["dev_yaml"] is True
    kpi = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert kpi["policy_source"]["dev_yaml"] is True and kpi["kpi_grade"] is False
    assert any("--dev-yaml" in r for r in kpi["kpi_grade_reasons"]), kpi
    idx = json.loads(__import__("tarfile").open(out / "t.replay.tar.gz")
                     .extractfile("replay.json").read())
    assert idx["policy_source"]["dev_yaml"] is True
    assert "(--dev-yaml)" in (out / "report.md").read_text(encoding="utf-8")


def test_both_rails_and_the_ros_script_wire_the_choice():
    """The ROS node does not import here; its parser does (the stress
    harness checks its command lines against it), and the script's default
    is read."""
    import ros2_shield_node as N
    a = N.make_parser().parse_args(["--policy", str(POLICY), "--dev-yaml"])
    assert a.dev_yaml and N.resolve_policy(a)[1] == str(POLICY)
    try:
        N.resolve_policy(N.make_parser().parse_args(["--policy", str(POLICY)]))
    except ValueError:
        pass
    else:
        raise AssertionError("the ROS node would fly a YAML without --dev-yaml")
    src = (ROOT / "sitl" / "run_ros2_demo.sh").read_text(encoding="utf-8")
    assert "flight_bundle_path" in src and 'EXTRA+=(--bundle "$DEFAULT_BUNDLE")' in src
    assert src.index("HAS_DEV_YAML") < src.index('nohup bash "$DIR/start_sitl.sh"'), (
        "the policy must be settled before SITL starts")


def test_the_ros_node_refuses_a_policy_other_than_the_pinned_hash():
    """REGRESSION (2026-10-13): the stress harness sends --expected-hash with
    every bundle, but the node did not declare the flag, so parse_known_args
    dropped it and any bundle would have flown. The node now declares it and
    refuses a policy whose hash differs, before take-off."""
    import ros2_shield_node as N
    pol = load_policy(POLICY)
    a = N.make_parser().parse_args(["--expected-hash", pol.policy_hash])
    assert a.expected_hash == pol.policy_hash
    assert N.check_expected_hash(pol, None) == {}
    rec = N.check_expected_hash(pol, pol.policy_hash.split(":", 1)[1])
    assert rec == {"expected_hash": pol.policy_hash, "expected_hash_ok": True}
    try:
        N.check_expected_hash(pol, "sha256:" + "0" * 64)
    except ValueError as e:
        assert "refusing" in str(e)
    else:
        raise AssertionError("a policy other than the pinned one would fly")
    import experiments.sweep_scenarios as S
    assert S.SITL_EXPECTED_HASH_FLAG in S.sitl_node_flags()


# --------------------------------------------------------------------------- #
# The city rail (demo/follow_vlm.py), ARCH-29, 2026-10-21
# --------------------------------------------------------------------------- #

CITY = """policy_id: city-arch29-test
version: 0.1.0
constraints:
  - id: kin-caps
    type: kinematic_envelope
    constraint_type: hard
    priority: P1
    violation_action: repair
    speed_max_mps: 5.0
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 45.0
"""


def _city_args(**kw):
    from types import SimpleNamespace
    base = {"policy": None, "bundle": None, "dev_yaml": False}
    base.update(kw)
    return SimpleNamespace(**base)


def _city_bundle(text: str = CITY):
    """(yaml, bundles dir, bundle): a policy file and its bundle, named as
    tools/build_flight_bundles.py names it. Unsigned: the choice reads only
    presence and the policy hash; load_for_flight judges the signature."""
    from guardrail.bundle import write_bundle
    from guardrail.manifest import flight_bundle_path
    d = Path(tempfile.mkdtemp(prefix="city_policy_"))
    y = d / "city.yaml"
    y.write_text(text, encoding="utf-8")
    bundles = d / "bundles"
    pol = load_policy(y)
    b = flight_bundle_path(pol, bundles)
    write_bundle(pol, b, signer=None)
    return y, bundles, b


def test_the_city_rail_flies_the_signed_bundle_of_its_policy():
    """REGRESSION (ARCH-29): follow_vlm loaded `--policy P` as unsigned YAML
    on every flight (citylife_redcar_nfz_motion2_20261019 kpi.json:
    policy_source yaml / unsigned) while the ArduPilot rails flew signed
    bundles by default. Shown failing first: follow_vlm had no
    city_policy_choice and no --dev-yaml."""
    import follow_vlm as fv
    y, bundles, b = _city_bundle()
    got = fv.city_policy_choice(_city_args(policy=str(y)), bundles_dir=bundles)
    assert got[0] == str(b) and got[1] is None, got
    assert got[2]["default_bundle"] is True and got[2]["bundle_of"].endswith("city.yaml"), got
    # The explicit fallback: the YAML itself, recorded as a development run.
    b2, y2, rec = fv.city_policy_choice(_city_args(policy=str(y), dev_yaml=True),
                                        bundles_dir=bundles)
    assert (b2, y2) == (None, str(y)) and rec["dev_yaml"] is True, rec
    # --bundle flies as given; with --dev-yaml it is two answers, refused.
    assert fv.city_policy_choice(_city_args(bundle=str(b)))[0] == str(b)
    try:
        fv.city_policy_choice(_city_args(bundle=str(b), dev_yaml=True))
    except ValueError as e:
        assert "--dev-yaml" in str(e), e
    else:
        raise AssertionError("--bundle with --dev-yaml was accepted")


def test_the_city_rail_refuses_a_missing_or_stale_bundle_naming_both_ways_out():
    import follow_vlm as fv
    y, bundles, b = _city_bundle()
    empty = Path(tempfile.mkdtemp(prefix="city_policy_empty_"))
    try:
        fv.city_policy_choice(_city_args(policy=str(y)), bundles_dir=empty)
    except ValueError as e:
        assert "build_flight_bundles" in str(e) and "--dev-yaml" in str(e), e
    else:
        raise AssertionError("a policy with no bundle flew as YAML without --dev-yaml")
    # Edited without a version bump: the bundle keeps its name, not its rules.
    y.write_text(CITY.replace("speed_max_mps: 5.0", "speed_max_mps: 6.0"),
                 encoding="utf-8")
    try:
        fv.city_policy_choice(_city_args(policy=str(y)), bundles_dir=bundles)
    except ValueError as e:
        assert "stale" in str(e), e
    else:
        raise AssertionError("a stale bundle flew")
    # No --policy: the default policy's bundle, named in the refusal.
    try:
        fv.city_policy_choice(_city_args(), bundles_dir=empty)
    except ValueError as e:
        assert "follow_car" in str(e), e
    else:
        raise AssertionError("the default policy flew with no bundle on disk")


def test_the_city_rail_wires_the_choice_and_a_dev_yaml_run_is_not_kpi_grade():
    """The parser takes --dev-yaml, main() loads what city_policy_choice
    chose (before the output folder is touched), and a --dev-yaml flight's
    kpi.json gives the reason it is not KPI-grade. The tool builds the
    bundle of every policy the city launchers name."""
    import follow_vlm as fv
    a = fv.build_arg_parser().parse_args(["--object", "a red car", "--dev-yaml"])
    assert a.dev_yaml is True
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    main_src = src[src.index("def fly("):] if "def fly(" in src else src
    assert main_src.index("city_policy_choice(args)") < main_src.index("load_for_flight(")
    assert "policy_source.update(_choice)" in src
    assert "if policy_source.get(\"dev_yaml\"):" in src
    T = _tool()
    for name in ("follow_car", "follow_car_nfz", "follow_pedestrian",
                 "follow_car_citylife", "follow_car_citylife_nfz"):
        assert name in T.FLIGHT_POLICIES, name
    for ps1 in ("run_citylife_follow.ps1", "run_follow_vlm.ps1", "run_retarget_demo.ps1"):
        text = (ROOT / "scripts" / ps1).read_text(encoding="utf-8-sig")
        assert "Get-FlightPolicyDev" in text, ps1


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
        # SystemExit too: a rail whose parser refuses a flag exits, and
        # that is this test failing, not the run ending.
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
