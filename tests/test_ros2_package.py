"""The ROS 2 packages under sitl/ros2_ws, checked without ROS.

Run either way:
    pytest tests/test_ros2_package.py -v
    python tests/test_ros2_package.py

The grant's Safety Shield Outputs table asks for a "ROS 2 node package". The
Shield used to be a single script started by hand (audit card WP3-24). These
tests pin what makes the two ament_python packages installable and their
launch file runnable: package.xml / setup.py agree on the name and build
type, every console entry point resolves to a function that exists, the
launch file starts the three node scripts that exist and ends the rail when
the Shield exits, and the entry points find the repository instead of
carrying a second copy of the node code. A colcon build in WSL
(`colcon build --symlink-install --base-paths sitl/ros2_ws/src`) is the
integration check; it was run on 2026-10-06 (see docs/DESIGN-ros2-interface.md).
"""
import ast
import atexit
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "sitl" / "ros2_ws" / "src"
PKGS = {"safety_shield": WS / "safety_shield",
        "mavlink_adapter": WS / "mavlink_adapter"}

# Every temporary keystore / PKI folder a test makes, removed when the
# interpreter exits, under either runner (the T1d review found sros2_ks_*,
# sros2_ev_* and sros2_pki_* folders left behind, on Windows and in WSL /tmp).
_TMP: list[Path] = []
atexit.register(lambda: [shutil.rmtree(d, ignore_errors=True) for d in _TMP])


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _setup_kwargs(pkg_dir: Path) -> dict:
    """The literal keyword arguments of setup(...) in setup.py."""
    tree = ast.parse((pkg_dir / "setup.py").read_text(encoding="utf-8"))
    consts = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
            consts[node.targets[0].id] = node.value.value
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "setup":
            out = {}
            for kw in node.keywords:
                if isinstance(kw.value, ast.Name) and kw.value.id in consts:
                    out[kw.arg] = consts[kw.value.id]
                else:
                    try:
                        out[kw.arg] = ast.literal_eval(kw.value)
                    except ValueError:
                        out[kw.arg] = ast.unparse(kw.value)
            return out
    raise AssertionError(f"{pkg_dir}/setup.py calls no setup()")


def test_each_package_is_ament_python_and_names_itself_consistently():
    for name, d in PKGS.items():
        xml = ET.parse(d / "package.xml").getroot()
        assert xml.findtext("name") == name, (name, xml.findtext("name"))
        assert xml.find("export/build_type").text == "ament_python"
        deps = {e.text for e in xml.findall("exec_depend")}
        assert {"rclpy", "std_msgs", "mavros_msgs"} <= deps, (name, deps)
        kw = _setup_kwargs(d)
        assert kw["name"] == name, kw["name"]
        assert (d / "resource" / name).is_file(), "ament index marker missing"
        cfg = (d / "setup.cfg").read_text(encoding="utf-8")
        assert f"$base/lib/{name}" in cfg, "ros2 run looks in lib/<package>"


def test_every_entry_point_resolves_to_a_function_that_exists():
    for name, d in PKGS.items():
        kw = _setup_kwargs(d)
        for spec in kw["entry_points"]["console_scripts"]:
            exe, target = (s.strip() for s in spec.split("="))
            mod, func = target.split(":")
            path = d.joinpath(*mod.split(".")).with_suffix(".py")
            assert path.is_file(), f"{name}: {exe} -> {path} missing"
            fns = {n.name for n in ast.parse(path.read_text(encoding="utf-8")).body
                   if isinstance(n, ast.FunctionDef)}
            assert func in fns, f"{name}: {exe} -> {mod}.{func} not defined"


def test_the_packages_ship_their_launch_files():
    for name, d in PKGS.items():
        kw = _setup_kwargs(d)
        assert "launch/*.launch.py" in str(kw["data_files"]), (name, kw["data_files"])
        assert list((d / "launch").glob("*.launch.py")), f"{name} has no launch file"


def test_the_rail_launch_file_starts_the_three_nodes_and_ends_with_the_shield():
    src = (PKGS["safety_shield"] / "launch" / "guardrail_rail.launch.py").read_text(
        encoding="utf-8")
    tree = ast.parse(src)
    assert any(isinstance(n, ast.FunctionDef) and n.name == "generate_launch_description"
               for n in tree.body)
    for script in ("ros2_vla_stub_node.py", "ros2_vla_bridge_node.py",
                   "mavlink_adapter_node.py", "ros2_shield_node.py"):
        assert script in src, f"the launch file does not start {script}"
        assert (ROOT / "sitl" / script).is_file(), f"sitl/{script} is missing"
    assert "OnProcessExit" in src and "target_action=shield" in src, (
        "the rail must shut down when the Shield node (the mission) exits")


def test_the_entry_points_find_the_repository_not_a_copy():
    """No node code is duplicated into the packages: they import sitl/*.py.
    With GUARDRAIL_ROOT unset the repository is found from the file's own
    location (what a symlink install resolves to); a wrong GUARDRAIL_ROOT is
    refused rather than silently used."""
    sys.path.insert(0, str(PKGS["safety_shield"]))
    sys.path.insert(0, str(PKGS["mavlink_adapter"]))
    import mavlink_adapter as MA
    import safety_shield as SS
    saved = os.environ.pop("GUARDRAIL_ROOT", None)
    try:
        assert SS.repo_root() == ROOT and MA.repo_root() == ROOT
        os.environ["GUARDRAIL_ROOT"] = str(ROOT / "docs")
        for f in (SS.repo_root, MA.repo_root):
            try:
                f()
            except RuntimeError:
                continue
            raise AssertionError(f"{f.__module__} accepted a root without guardrail/")
    finally:
        os.environ.pop("GUARDRAIL_ROOT", None)
        if saved is not None:
            os.environ["GUARDRAIL_ROOT"] = saved
    for d in PKGS.values():
        for py in d.rglob("*.py"):
            assert "class ShieldNode" not in py.read_text(encoding="utf-8"), (
                f"{py} carries a copy of the Shield node")


def test_the_demo_script_records_the_grants_topics_and_routes_through_the_adapter():
    sh = (ROOT / "sitl" / "run_ros2_demo.sh").read_text(encoding="utf-8")
    assert "ros2 bag record" in sh
    for topic in ("/vla/action_4d", "/mavros/setpoint_raw/local",
                  "/mavros/state", "/mavros/local_position/pose"):
        assert topic in sh, f"the bag does not record {topic}"
    assert "guardrail_rail.launch.py" in sh and "mavlink_adapter_node.py" in sh
    assert "start_router.sh" in sh and "udp://:" in sh, "MAVROS must use the router"
    assert "guardrail.replay pack" in sh, "the bundle must be re-packed with the bag"


def test_sitl_loads_the_fence_backstop_and_feeds_the_router():
    sh = (ROOT / "sitl" / "start_sitl.sh").read_text(encoding="utf-8")
    assert "fence/guardrail_fence.parm" in sh
    assert "udpclient:127.0.0.1" in sh, "SITL must send MAVLink to the router"
    conf = (ROOT / "sitl" / "mavlink-router" / "main.conf").read_text(encoding="utf-8")
    for needed in ("Port = 14550", "Port = 14555", "@GCS_HOST@", "Mode = Server"):
        assert needed in conf, f"router config lacks {needed!r}"


# --------------------------------------------------------------------------- #
# SROS 2 (2026-10-19). The grant's data-flow page: "SROS 2 turned on for the
# ROS 2 graph using the same root CA". sitl/run_ros2_demo.sh SROS2=1 runs
# every ROS 2 process of the rail in its own enclave under Enforce; these
# tests pin its configuration without ROS. The live check is
# tools/sros2_check.sh (6/6 in WSL Ubuntu 24.04 + Jazzy on 2026-10-19,
# docs/DESIGN-sros2.md). Every test here fails on c77c77d: there was no
# enclave list, no enclave_root:= in the launch file, no sitl/sros2_env.sh and
# no ros2_security_evidence.
# --------------------------------------------------------------------------- #

ENCLAVE_LIST = ROOT / "sitl" / "sros2_enclaves.txt"


def _enclaves() -> list[str]:
    return [ln.strip() for ln in ENCLAVE_LIST.read_text(encoding="utf-8").splitlines()
            if ln.strip() and not ln.strip().startswith("#")]


def _launch_processes(config: dict) -> dict:
    """Run guardrail_rail.launch.py's _nodes() against stand-in `launch`
    modules: process name -> the keyword arguments of its ExecuteProcess."""
    import importlib.util
    import types
    made = {}

    class _Rec:
        def __init__(self, *a, **kw):
            self.a, self.kw = a, kw

    class ExecuteProcess(_Rec):
        def __init__(self, **kw):
            super().__init__(**kw)
            made[kw["name"]] = kw

    class LaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    mods = {
        "launch": {"LaunchDescription": _Rec},
        "launch.actions": {n: _Rec for n in ("DeclareLaunchArgument", "EmitEvent", "LogInfo",
                                             "OpaqueFunction", "RegisterEventHandler")}
        | {"ExecuteProcess": ExecuteProcess},
        "launch.event_handlers": {"OnProcessExit": _Rec},
        "launch.events": {"Shutdown": _Rec},
        "launch.substitutions": {"LaunchConfiguration": LaunchConfiguration},
    }
    saved = {m: sys.modules.get(m) for m in mods}
    try:
        for name, attrs in mods.items():
            mod = types.ModuleType(name)
            mod.__dict__.update(attrs)
            sys.modules[name] = mod
        path = PKGS["safety_shield"] / "launch" / "guardrail_rail.launch.py"
        spec = importlib.util.spec_from_file_location("guardrail_rail_launch_t1d", path)
        launch_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch_mod)
        ctx = {"python": "/venv/bin/python", "repo_root": str(ROOT), "out": "",
               "shield_args": "--shield on", "vla": "stub", "vla_args": "",
               "adapter_args": "", "enclave_root": "", **config}
        launch_mod._nodes(ctx)
        # The declared default of enclave_root is empty (nothing changes).
        desc = launch_mod.generate_launch_description()
        decl = {d.a[0]: d.kw.get("default_value") for d in desc.a[0]
                if isinstance(d, _Rec) and d.a and isinstance(d.a[0], str)}
    finally:
        for m, v in saved.items():
            if v is None:
                sys.modules.pop(m, None)
            else:
                sys.modules[m] = v
    return {"processes": made, "defaults": decl}


def test_each_node_gets_its_own_enclave_only_when_asked():
    """enclave_root:=/guardrail -> ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/
    <process> for each of the three nodes, each in the enclave list; the
    default (empty) adds nothing to any process's environment."""
    off = _launch_processes({"enclave_root": ""})
    assert off["defaults"].get("enclave_root") == "", off["defaults"]
    assert set(off["processes"]) == {"vla_stub", "mavlink_adapter", "safety_shield"}
    for name, kw in off["processes"].items():
        assert kw["additional_env"] == {"GUARDRAIL_ROOT": str(ROOT)}, (name, kw)
    on = _launch_processes({"enclave_root": "/guardrail"})
    enclaves = set(_enclaves())
    for name, kw in on["processes"].items():
        enc = kw["additional_env"].get("ROS_SECURITY_ENCLAVE_OVERRIDE")
        assert enc == f"/guardrail/{name}", (name, enc)
        assert enc in enclaves, f"{enc} is not in {ENCLAVE_LIST.name}: sros2_setup.sh would not make it"
    try:
        _launch_processes({"enclave_root": "guardrail"})
    except RuntimeError as e:
        assert "absolute" in str(e), e
    else:
        raise AssertionError("a relative enclave_root was accepted")


def test_the_vla_slot_holds_the_stub_or_the_bridge_never_both():
    """vla:= (2026-10-23, to-do VLA-08): the default is the stub, exactly as
    before; vla:=bridge starts sitl/ros2_vla_bridge_node.py INSTEAD (one
    publisher on /vla/action_4d), with the same vla_args, `--out <out>` when
    out:= is given, and the VLA slot's enclave /guardrail/vla_stub (an
    enclave of its own would make every existing keystore refuse SROS2=1);
    anything else is refused."""
    stub = _launch_processes({})
    assert stub["defaults"].get("vla") == "stub", stub["defaults"]
    assert set(stub["processes"]) == {"vla_stub", "mavlink_adapter", "safety_shield"}
    assert stub["processes"]["vla_stub"]["cmd"][1].endswith("ros2_vla_stub_node.py")
    br = _launch_processes({"vla": "bridge", "vla_args": "--vla-server 10.0.0.2:8790",
                            "out": "/runs/x", "enclave_root": "/guardrail"})
    assert set(br["processes"]) == {"vla_bridge", "mavlink_adapter", "safety_shield"}
    cmd = br["processes"]["vla_bridge"]["cmd"]
    assert cmd[1].endswith("ros2_vla_bridge_node.py"), cmd
    assert cmd[2:] == ["--vla-server", "10.0.0.2:8790", "--out", "/runs/x"], cmd
    env = br["processes"]["vla_bridge"]["additional_env"]
    assert env["ROS_SECURITY_ENCLAVE_OVERRIDE"] == "/guardrail/vla_stub", env
    assert "/guardrail/vla_stub" in set(_enclaves())
    no_out = _launch_processes({"vla": "bridge"})["processes"]["vla_bridge"]["cmd"]
    assert "--out" not in no_out, no_out
    try:
        _launch_processes({"vla": "openvla"})
    except RuntimeError as e:
        assert "openvla" in str(e), e
    else:
        raise AssertionError("an unknown vla:= was accepted")


def test_every_ros_process_the_runner_starts_has_an_enclave_in_the_list():
    """sitl/run_ros2_demo.sh: each ROS 2 command carries its sros2_prefix
    array (empty with SROS2=0), each prefix names an enclave in the list,
    the launch gets enclave_root:= only through sros2_launch_args, and
    tools/sros2_setup.sh makes exactly the list's enclaves (no second copy)."""
    import re
    sh = (ROOT / "sitl" / "run_ros2_demo.sh").read_text(encoding="utf-8")
    enclaves = set(_enclaves())
    assert {"/guardrail/safety_shield", "/guardrail/mavlink_adapter", "/guardrail/vla_stub",
            "/guardrail/mavros", "/guardrail/bag_recorder"} == enclaves, enclaves
    prefixes = dict(re.findall(r"^sros2_prefix (\w+) (\w+)$", sh, re.M))
    assert {f"/guardrail/{v}" for v in prefixes.values()} == enclaves, prefixes
    assert 'source "$DIR/sros2_env.sh"' in sh and "sros2_prepare || exit 2" in sh
    # refused before anything starts: before the clean slate and SITL
    assert sh.index("sros2_prepare || exit 2") < sh.index('echo "=== fresh SITL ==="')
    for var, cmd in (("ENC_MAVROS", "ros2 run mavros mavros_node"),
                     ("ENC_BAG", "ros2 bag record"),
                     ("ENC_VLA", '"$DIR/ros2_vla_stub_node.py"'),
                     ("ENC_VLA", '"$DIR/ros2_vla_bridge_node.py"'),
                     ("ENC_ADAPTER", '"$DIR/mavlink_adapter_node.py"'),
                     ("ENC_SHIELD", '"$DIR/ros2_shield_node.py"')):
        line = next(ln for ln in sh.splitlines() if cmd in ln and "nohup" in ln
                    or cmd in ln and ln.lstrip().startswith('"${'))
        assert f'"${{{var}[@]}}"' in line, (var, line)
    assert '"${LAUNCH_SROS2[@]}"' in sh and "sros2_launch_args LAUNCH_SROS2" in sh
    # no security variable is set by the runner itself, only by sros2_env.sh
    assert "export ROS_SECURITY" not in sh and "ROS_SECURITY_ENABLE" not in sh
    setup = (ROOT / "tools" / "sros2_setup.sh").read_text(encoding="utf-8")
    assert "sitl/sros2_enclaves.txt" in setup and "ENCLAVES=(" not in setup
    assert "identity_ca.cert.pem" in setup and "ln -sfn ca.key.pem" in setup, (
        "create_keystore replaces a CA it does not find in both roles (sros2 0.13.6)")


def _bash() -> str | None:
    """A bash that takes this OS's paths: Git Bash on Windows (not
    System32's, which is WSL's), else the PATH's."""
    import shutil
    if os.name == "nt":
        for c in (r"C:\Program Files\Git\bin\bash.exe", r"C:\Program Files\Git\usr\bin\bash.exe"):
            if Path(c).is_file():
                return c
        return None
    return shutil.which("bash")


def _sros2_env(script: str, env: dict) -> tuple[int, str, str]:
    import subprocess
    lib = (ROOT / "sitl" / "sros2_env.sh").as_posix()
    full = {k: v for k, v in os.environ.items() if not k.startswith(("ROS_SECURITY", "SROS2"))}
    full.update(env)
    cp = subprocess.run([_bash(), "-c", f'source "{lib}"\n{script}'], capture_output=True,
                        text=True, env=full, timeout=60)
    return cp.returncode, cp.stdout, cp.stderr


SKIP = "SKIP"


def test_sros2_env_refuses_a_keystore_missing_an_enclave_and_is_inert_when_off():
    """sitl/sros2_env.sh with plain bash: SROS2=1 on a complete keystore
    exports the three variables and gives each process its prefix; one file
    missing refuses (exit 2, the file named, nothing exported); SROS2 unset
    exports nothing and every prefix is empty; another value is refused."""
    if _bash() is None:
        print("      SKIP: no bash on this host")
        return SKIP
    ks = _tmp("sros2_ks_")
    (ks / "enclaves").mkdir()
    (ks / "enclaves" / "governance.p7s").write_text("x")
    for enc in _enclaves():
        d = ks / "enclaves" / enc.lstrip("/")
        d.mkdir(parents=True)
        for f in ("cert.pem", "key.pem", "permissions.p7s", "governance.p7s",
                  "identity_ca.cert.pem", "permissions_ca.cert.pem"):
            (d / f).write_text("x")
    show = ('sros2_prepare; rc=$?; echo "rc=$rc"; env | grep "^ROS_SECURITY" | sort; '
            'sros2_prefix A safety_shield; echo "A=${A[*]}"; '
            'sros2_launch_args L; echo "L=${L[*]}"')
    rc, out, err = _sros2_env(show, {"SROS2": "1", "SROS2_KEYSTORE": ks.as_posix()})
    assert "rc=0" in out, (out, err)
    assert f"ROS_SECURITY_KEYSTORE={ks.as_posix()}" in out, out
    assert "ROS_SECURITY_ENABLE=true" in out and "ROS_SECURITY_STRATEGY=Enforce" in out, out
    assert "A=env ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/safety_shield" in out, out
    assert "L=enclave_root:=/guardrail" in out, out
    (ks / "enclaves" / "guardrail" / "mavros" / "permissions.p7s").unlink()
    rc, out, err = _sros2_env(show, {"SROS2": "1", "SROS2_KEYSTORE": ks.as_posix()})
    assert "rc=2" in out and "REFUSED" in err, (out, err)
    assert "enclaves/guardrail/mavros/permissions.p7s" in err, err
    assert "ROS_SECURITY" not in out.split("A=")[0], f"refused, yet exported: {out}"
    rc, out, err = _sros2_env(show, {})
    assert "rc=0" in out and "ROS_SECURITY" not in out.split("A=")[0], out
    assert "A=\n" in out and "L=\n" in out, f"prefixes must be empty with SROS2 off: {out!r}"
    rc, out, err = _sros2_env(show, {"SROS2": "yes"})
    assert "rc=2" in out and "must be 0 or 1" in err, (out, err)
    rc, out, err = _sros2_env('sros2_prefix B no_such_node; echo "rc=$?"',
                              {"SROS2": "1"})
    assert "rc=2" in out and "no enclave /guardrail/no_such_node" in err, (out, err)


def test_the_live_check_counts_a_refusal_only_in_the_security_layers_words():
    """tools/sros2_check.sh (review of the T1d branch, P6): check 3 passed on
    any non-zero exit with no PROBE line, so a probe that crashed for another
    reason (an import error, a missing module) read as "refused by
    Enforce"; check 5 read a missing received= count as 0, so a crashed
    foreign subscriber "heard nothing". Shown failing at 044f0fc: the script
    had neither helper, and check 5 tested ${n5:-0}. Now both verdicts go
    through refused_by_security / heard_nothing, run here with plain bash on
    probe outputs: the rcl refusal text recorded on 2026-10-19 in WSL, a
    PROBE line, and three deaths that are not a refusal."""
    if _bash() is None:
        print("      SKIP: no bash on this host")
        return SKIP
    import subprocess
    src = (ROOT / "tools" / "sros2_check.sh").read_text(encoding="utf-8")
    start, end = "# --- verdict helpers", "# --- end of the verdict helpers ---"
    assert start in src and end in src, "the verdict helpers are not marked"
    helpers = src.split(start, 1)[1].split("\n", 1)[1].split(end, 1)[0]
    assert 'refused_by_security "$WORK/sub3.txt"' in src, "check 3 does not use the helper"
    assert 'heard_nothing "$WORK/sub5.txt"' in src, "check 5 does not use the helper"
    assert "${n5:-0}" not in src, "check 5 still reads a missing count as 0"
    cases = {
        "refused": ("Traceback (most recent call last):\n  File \"probe.py\", line 6\n"
                    "rclpy._rclpy_pybind11.RCLError: error creating node: couldn't find all "
                    "security files!, at ./src/rcl/node.c:271\n"),
        "heard_zero": "PROBE role=sub node=mavlink_adapter enclave=/guardrail/mavlink_adapter "
                      "received=0 sent=0\n",
        "heard_some": "PROBE role=sub node=outsider enclave=- received=12 sent=0\n",
        "import_error": ("Traceback (most recent call last):\n"
                         "ModuleNotFoundError: No module named 'std_msgs'\n"),
        "other_rcl_error": ("rclpy._rclpy_pybind11.RCLError: failed to initialize rcl: "
                            "RMW implementation not found\n"),
        "nothing": "",
    }
    d = _tmp("sros2_verdict_")
    for name, text in cases.items():
        (d / f"{name}.txt").write_text(text, encoding="utf-8", newline="\n")
    loop = "".join(
        f'refused_by_security "{(d / n).as_posix()}.txt" && echo "{n}:refused"\n'
        f'heard_nothing "{(d / n).as_posix()}.txt" && echo "{n}:heard_nothing"\n'
        for n in cases)
    cp = subprocess.run([_bash(), "-c", helpers + "\n" + loop], capture_output=True,
                        text=True, timeout=60)
    got = set(cp.stdout.split())
    assert got == {"refused:refused", "refused:heard_nothing",
                   "heard_zero:heard_nothing"}, (got, cp.stderr)


def test_a_flight_records_sros2_only_when_it_is_on():
    """guardrail.manifest.ros2_security_evidence, which the Shield node puts
    in hil_evidence: None with security off (the record is what it was), and
    with it on the strategy, the enclave, whether its files are there and the
    identity CA's SHA-256 fingerprint (openssl's format), so "same root CA"
    can be read from the record."""
    import base64
    import hashlib
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from guardrail.manifest import ros2_security_evidence
    assert ros2_security_evidence({}) is None
    assert ros2_security_evidence({"ROS_SECURITY_ENABLE": "false"}) is None
    ks = _tmp("sros2_ev_")
    d = ks / "enclaves" / "guardrail" / "safety_shield"
    d.mkdir(parents=True)
    der = bytes(range(64))
    pem = ("-----BEGIN CERTIFICATE-----\n" + base64.b64encode(der).decode()
           + "\n-----END CERTIFICATE-----\n")
    for f in ("cert.pem", "key.pem", "permissions.p7s", "governance.p7s",
              "identity_ca.cert.pem", "permissions_ca.cert.pem"):
        (d / f).write_text(pem if f.endswith("ca.cert.pem") else "x")
    ev = ros2_security_evidence({"ROS_SECURITY_ENABLE": "true",
                                 "ROS_SECURITY_STRATEGY": "Enforce",
                                 "ROS_SECURITY_KEYSTORE": str(ks),
                                 "ROS_SECURITY_ENCLAVE_OVERRIDE": "/guardrail/safety_shield"})
    want = ":".join(f"{b:02X}" for b in hashlib.sha256(der).digest())
    assert ev == {"enabled": True, "strategy": "Enforce", "keystore": str(ks),
                  "enclave": "/guardrail/safety_shield",
                  "enclave_source": "ROS_SECURITY_ENCLAVE_OVERRIDE",
                  "files_present": True, "identity_ca_sha256": want,
                  "permissions_ca_sha256": want}, ev
    (d / "permissions.p7s").unlink()
    assert ros2_security_evidence({"ROS_SECURITY_ENABLE": "true", "ROS_SECURITY_KEYSTORE": str(ks),
                                   "ROS_SECURITY_ENCLAVE_OVERRIDE": "/guardrail/safety_shield"}
                                  )["files_present"] is False
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "ros2_security_evidence()" in src and 'hil_evidence["ros2_security"]' in src
    # Against a real certificate (where `cryptography` can make one), the
    # fingerprint is the one gen-pki prints for the lab CA.
    from guardrail.bundle import HAVE_CRYPTO
    if HAVE_CRYPTO:
        from guardrail.harness_service import fingerprint, generate_pki
        from guardrail.manifest import _pem_sha256_fingerprint
        files = generate_pki(_tmp("sros2_pki_"))
        assert _pem_sha256_fingerprint(files["ca_cert"]) == fingerprint(files["ca_cert"])
    else:
        print("      (no cryptography here: the fingerprint was checked on a synthetic "
              "certificate only)")


def test_no_shell_or_launch_file_has_windows_line_endings():
    crlf = bytes([13, 10])
    for f in list((ROOT / "sitl").rglob("*.sh")) + list(WS.rglob("*.py")) \
            + list(WS.rglob("*.xml")) + list(WS.rglob("setup.cfg")) \
            + [ROOT / "sitl" / "mavlink-router" / "main.conf",
               ROOT / "sitl" / "fence" / "guardrail_fence.parm"]:
        assert crlf not in f.read_bytes(), f"{f} has CRLF line endings"


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
