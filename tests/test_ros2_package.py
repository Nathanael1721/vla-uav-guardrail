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
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "sitl" / "ros2_ws" / "src"
PKGS = {"safety_shield": WS / "safety_shield",
        "mavlink_adapter": WS / "mavlink_adapter"}


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
    for script in ("ros2_vla_stub_node.py", "mavlink_adapter_node.py",
                   "ros2_shield_node.py"):
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


def test_no_shell_or_launch_file_has_windows_line_endings():
    crlf = bytes([13, 10])
    for f in list((ROOT / "sitl").rglob("*.sh")) + list(WS.rglob("*.py")) \
            + list(WS.rglob("*.xml")) + list(WS.rglob("setup.cfg")) \
            + [ROOT / "sitl" / "mavlink-router" / "main.conf",
               ROOT / "sitl" / "fence" / "guardrail_fence.parm"]:
        assert crlf not in f.read_bytes(), f"{f} has CRLF line endings"


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
