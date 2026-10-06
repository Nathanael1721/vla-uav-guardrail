"""Environment pins: pyproject.toml, the Python floor, and the ArduPilot pin.

Run either way:
    pytest tests/test_env_pins.py -v
    python tests/test_env_pins.py

WHY THIS FILE EXISTS

Three contract cards (WP1-25, WP2-19, X-06) say the grant locks Python 3.11+
for the non-VLA stack (Policy DSL p1, Prefix Compiler p1, Safety Shield p1,
Stress Testing p1) and that nothing in the repo stated it. A fourth (ARCH-31)
says sitl/setup_sitl.sh cloned whatever ArduPilot master was that day, so the
autopilot behind a SITL number could not be named. pyproject.toml and the pin
in setup_sitl.sh are the fixes; this file is what stops them rotting.

Most static tests here are about a second copy drifting from the first: the
dependency list in pyproject.toml against requirements.txt, the version
against VERSION, the pinned commit against the design doc, and the imports the
guardrail package really makes against the dependencies it declares. A
packaging file that agrees with nothing is worse than none, because it is
believed.

The behavioural tests run setup_sitl.sh itself, offline, against throwaway
git repositories: an "upstream" ArduPilot stand-in with a release tag, a newer
master, a nested mavlink/pymavlink submodule pair and a fake `waf` that
"builds" a binary carrying HEAD's 8-hex banner the way the real one does. They
require the script to say NO: to a tag that resolves to another commit, to a
binary built from another commit or with no banner, to a failed rebuild with
an old binary still on disk, to a nested submodule at another commit, to
edited tracked files (inside submodules too), to a missing build or checkout,
and to a git command that fails (silence from a crashed `git status` is not
"clean"); and to refuse rather than discard local edits. A verifier that only
ever printed "PIN OK" would pass a happy-path test and check nothing. The
mutants of setup_sitl.sh run against this file on 2026-10-06 are listed in
docs/DESIGN-python-versions.md ("ArduPilot pin"); every non-equivalent one
fails at least one test.

No test here can reach the network or WSL's real ~/ardupilot, even if the
script regresses: --verify runs get an ARDUPILOT_URL that does not exist and a
stub ~/venv-ap, and the bash is chosen by what it does, not by its name
(_bash).

Point the tests at other files (to show they fail on the old code):
    ENV_PINS_SETUP_SITL=<path> ENV_PINS_PYPROJECT=<path>
    ENV_PINS_REQUIREMENTS=<path> python tests/test_env_pins.py
"""
import ast
import atexit
import hashlib
import os
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(os.environ.get("VLA_REPO_ROOT") or Path(__file__).resolve().parents[1])
sys.path.insert(0, str(ROOT))

SETUP_SITL = Path(os.environ.get("ENV_PINS_SETUP_SITL") or ROOT / "sitl" / "setup_sitl.sh")
PYPROJECT = Path(os.environ.get("ENV_PINS_PYPROJECT") or ROOT / "pyproject.toml")
REQUIREMENTS = Path(os.environ.get("ENV_PINS_REQUIREMENTS") or ROOT / "requirements.txt")
DESIGN_DOC = ROOT / "docs" / "DESIGN-python-versions.md"

# Import name -> distribution, for every third-party import guardrail/ may make.
CORE = {"pydantic": "pydantic", "pydantic_core": "pydantic", "yaml": "pyyaml",
        "shapely": "shapely", "numpy": "numpy", "jinja2": "jinja2"}
EXTRA = {"cryptography": "signing", "fastapi": "api", "uvicorn": "api",
         "tokenizers": "tokens"}
# Whole modules that exist only for an extra; importing them needs the extra.
EXTRA_ONLY_MODULES = {"api"}
# VLA-slot occupants: run from the source tree in the VLA env, not packaged
# promises (pyproject.toml [tool.setuptools] comment).
SOURCE_TREE_ONLY = {"torch": {"vla_bc"}, "training": {"vla_bc"}}

SKIPPED = []
_TMP = []


def _mkdtemp(prefix):
    """A temp dir that is removed when the run ends (git marks objects read-only,
    so a plain rmtree fails on Windows; clear the bit and retry)."""
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def _cleanup():
    def force(func, path, _exc):
        os.chmod(path, stat.S_IWRITE)
        func(path)
    for d in _TMP:
        shutil.rmtree(d, onerror=force) if d.exists() else None


atexit.register(_cleanup)


def _skip(why):
    SKIPPED.append(why)
    print(f"      SKIP: {why}")


def _toml():
    for name in ("tomllib", "tomli", "pip._vendor.tomli"):
        try:
            return __import__(name, fromlist=["loads"])
        except ImportError:
            continue
    return None


def _pyproject():
    assert PYPROJECT.is_file(), f"{PYPROJECT} does not exist"
    toml = _toml()
    assert toml is not None, "no TOML parser (tomllib/tomli) in this interpreter"
    return toml.loads(PYPROJECT.read_text(encoding="utf-8"))


def _norm_req(s):
    s = s.split("#", 1)[0].strip()
    if not s:
        return None
    m = re.match(r"([A-Za-z0-9_.\-]+)\s*(.*)$", s)
    name, spec = m.group(1).lower().replace("_", "-"), m.group(2).replace(" ", "")
    return name + ",".join(sorted(spec.split(","))) if spec else name


def _dist(req):
    """Distribution name of a requirement string, normalised."""
    n = _norm_req(req)
    return re.match(r"[a-z0-9.\-]+", n).group(0) if n else None


def _setup_text():
    assert SETUP_SITL.is_file(), f"{SETUP_SITL} does not exist"
    return SETUP_SITL.read_bytes().decode("utf-8")


def _pinned_sha(text):
    m = re.search(r'ARDUPILOT_SHA="\$\{ARDUPILOT_SHA:-([0-9a-f]+)\}"', text)
    return m.group(1) if m else None


def _guardrail_imports():
    """{module_stem: [(top_name, guarded, lazy)]} for every third-party import."""
    std = set(getattr(sys, "stdlib_module_names", ()))
    out = {}
    for p in sorted((ROOT / "guardrail").glob("*.py")):
        tree = ast.parse(p.read_text(encoding="utf-8"))
        parents = {}
        for node in ast.walk(tree):
            for child in ast.iter_child_nodes(node):
                parents[child] = node
        found = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [a.name.split(".")[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module.split(".")[0]]
            else:
                continue
            guarded = lazy = False
            up = parents.get(node)
            while up is not None:
                if isinstance(up, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    lazy = True
                if isinstance(up, ast.Try) and any(
                        isinstance(h.type, ast.Name) and h.type.id in ("ImportError", "ModuleNotFoundError")
                        for h in up.handlers if h.type is not None):
                    guarded = True
                up = parents.get(up)
            for n in names:
                if n not in std and n not in ("guardrail", "__future__"):
                    found.append((n, guarded, lazy))
        out[p.stem] = found
    return out


# --------------------------------------------------------------------------- #
# pyproject.toml
# --------------------------------------------------------------------------- #
def test_the_python_floor_is_the_grants_3_11():
    proj = _pyproject()["project"]
    assert proj.get("requires-python") == ">=3.11", (
        f"requires-python is {proj.get('requires-python')!r}; the grant locks 3.11+")


def test_core_dependencies_are_exactly_requirements_txt():
    """Two lists that drift apart are two answers to 'what does it need'."""
    proj = _pyproject()["project"]
    req = REQUIREMENTS.read_text(encoding="utf-8").splitlines()
    want = sorted(r for r in map(_norm_req, req) if r)
    got = sorted(r for r in map(_norm_req, proj.get("dependencies", [])) if r)
    assert want, "requirements.txt lists nothing - the comparison would be empty"
    assert got == want, f"pyproject {got} != requirements.txt {want}"


def test_the_version_is_read_from_VERSION_not_copied():
    doc = _pyproject()
    assert "version" not in doc["project"], "a literal version would drift from VERSION"
    assert "version" in doc["project"].get("dynamic", [])
    assert doc["tool"]["setuptools"]["dynamic"]["version"] == {"file": "VERSION"}
    ver = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
    assert re.fullmatch(r"\d+\.\d+\.\d+", ver), f"VERSION holds {ver!r}"


def test_signing_is_an_extra_named_signing_that_brings_cryptography():
    extras = _pyproject()["project"].get("optional-dependencies", {})
    assert any(_norm_req(r) == "cryptography" for r in extras.get("signing", [])), extras


def test_only_the_guardrail_package_is_packaged():
    """demo/, tools/, training/ and the OpenVLA code must never become installable."""
    pk = _pyproject()["tool"]["setuptools"].get("packages")
    assert pk == ["guardrail"], f"packages = {pk!r}"
    assert (ROOT / "guardrail" / "__init__.py").is_file()


def test_every_data_file_in_the_package_is_shipped():
    """The Jinja2 templates load from the package dir; a wheel without them
    imports cleanly and fails on the first build_prompt()."""
    pats = _pyproject()["tool"]["setuptools"].get("package-data", {}).get("guardrail", [])
    pkg = ROOT / "guardrail"
    data = {p for p in pkg.rglob("*") if p.is_file() and p.suffix not in (".py", ".pyc")
            and "__pycache__" not in p.parts}
    covered = {p for pat in pats for p in pkg.glob(pat)}
    missing = sorted(str(p.relative_to(pkg)) for p in data - covered)
    assert not missing, f"not in [tool.setuptools.package-data]: {missing[:8]}"


def test_every_import_guardrail_makes_is_declared():
    """A new third-party import must land in pyproject.toml, or this fails."""
    extras = _pyproject()["project"].get("optional-dependencies", {})
    problems = []
    for mod, imports in _guardrail_imports().items():
        for name, guarded, lazy in imports:
            if name in CORE:
                deps = {_dist(r) for r in _pyproject()["project"].get("dependencies", [])}
                if CORE[name] not in deps:
                    problems.append(f"{mod}: imports {name}, but {CORE[name]} is not a dependency")
                continue
            if name in EXTRA:
                extra = EXTRA[name]
                if name not in {_dist(r) for r in extras.get(extra, [])}:
                    problems.append(f"{mod}: {name} not in extra [{extra}]")
                elif not (guarded or lazy or mod in EXTRA_ONLY_MODULES):
                    problems.append(f"{mod}: imports optional {name} unguarded at module level")
                continue
            if mod in SOURCE_TREE_ONLY.get(name, set()):
                continue
            problems.append(f"{mod}: imports {name!r}, declared nowhere")
    assert not problems, "; ".join(problems)


def test_the_package_imports_without_any_optional_dependency():
    """vla-drone (3.11) has no cryptography; the Shield must still import there."""
    mods = ["guardrail"] + sorted(
        "guardrail." + p.stem for p in (ROOT / "guardrail").glob("*.py")
        if p.stem not in EXTRA_ONLY_MODULES | {"vla_bc", "__init__"})
    blocked = sorted(set(EXTRA) | {"torch", "transformers"})
    code = (
        "import sys, importlib\n"
        f"sys.path.insert(0, {str(ROOT)!r})\n"
        f"for b in {blocked!r}: sys.modules[b] = None   # import now raises ImportError\n"
        "bad = []\n"
        f"for m in {mods!r}:\n"
        "    try: importlib.import_module(m)\n"
        "    except Exception as e: bad.append(f'{m}: {type(e).__name__}: {e}')\n"
        "print('\\n'.join(bad)); sys.exit(1 if bad else 0)\n")
    r = subprocess.run([sys.executable, "-B", "-c", code], cwd=str(ROOT),
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, (r.stdout + r.stderr).strip()[-800:]


def test_guardrail_uses_nothing_newer_than_3_10():
    """vla-real (3.10.20) imports guardrail from source until the flight env moves."""
    banned = {("tomllib", None), ("enum", "StrEnum"), ("typing", "Self"),
              ("datetime", "UTC"), ("typing", "LiteralString"), ("typing", "Never")}
    hits = []
    for p in sorted((ROOT / "guardrail").glob("*.py")):
        src = p.read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                hits += [f"{p.name}: import {a.name}" for a in node.names
                         if (a.name, None) in banned]
            elif isinstance(node, ast.ImportFrom) and node.module:
                hits += [f"{p.name}: from {node.module} import {a.name}" for a in node.names
                         if (node.module, a.name) in banned or (node.module, None) in banned]
            elif type(node).__name__ == "TryStar":
                hits.append(f"{p.name}: except* (3.11)")
    assert not hits, "; ".join(hits)


# --------------------------------------------------------------------------- #
# sitl/setup_sitl.sh: the ArduPilot pin (static)
# --------------------------------------------------------------------------- #
def _code_lines(text):
    """The script minus its comments (the header quotes the old clone line)."""
    return "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))


def test_setup_sitl_pins_a_full_commit_and_a_ref():
    text = _setup_text()
    sha = _pinned_sha(text)
    assert sha and re.fullmatch(r"[0-9a-f]{40}", sha), f"no 40-hex ARDUPILOT_SHA pin (got {sha!r})"
    assert re.search(r'ARDUPILOT_REF="\$\{ARDUPILOT_REF:-[^}]+\}"', text), "no ARDUPILOT_REF pin"


def test_setup_sitl_checks_out_the_pin_and_refuses_anything_else():
    text = _code_lines(_setup_text())
    assert re.search(r'checkout --detach "\$ARDUPILOT_REF"', text), "never checks out the pin"
    assert re.search(r'\[ "\$actual" != "\$ARDUPILOT_SHA" \]', text), (
        "does not compare the resolved commit with the pinned SHA")
    assert "submodule update --init --recursive" in text, "submodules not set to the pin"


def test_setup_sitl_does_not_clone_master_submodules_first():
    """`git clone --recursive` then a checkout leaves master-only submodules behind."""
    hit = re.search(r"git clone[^\n]*--recursive", _code_lines(_setup_text()))
    assert not hit, f"still runs: {hit.group(0)!r}"


def test_setup_sitl_cannot_hide_a_failed_build_behind_tail():
    text = _code_lines(_setup_text())
    assert re.search(r"^\s*set -[a-z]*o pipefail", text, re.M), (
        "`./waf ... | tail` returns tail's status: a failed build would look fine")


def _venv_ap_packages():
    """Every package of the ~/venv-ap install, quoted or bare, across
    continuation lines (the `pip install -U pip` line excluded)."""
    logical = re.sub(r"\\\n\s*", " ", _code_lines(_setup_text())).splitlines()
    cmds = [ln for ln in logical if "venv-ap/bin/pip install" in ln and " -U pip" not in ln]
    assert cmds, "venv package install line not found"
    return [t for c in cmds for t in shlex.split(c.split(" install", 1)[1])
            if not t.startswith("-")]


def test_setup_sitl_pins_the_venv_packages():
    pkgs = _venv_ap_packages()
    loose = [p for p in pkgs if "==" not in p]
    assert pkgs and not loose, f"unpinned venv packages: {loose}"


def test_setup_sitl_installs_every_core_dependency_pinned_in_the_venv():
    """sitl/run_sitl_demo.py imports the Shield and calls build_prompt() in
    ~/venv-ap. A core dependency missing from that install fails the rail at
    run time (jinja2 was missing from the real ~/venv-ap on 2026-10-06), and
    one left to arrive unpinned (numpy) runs the Shield on an unnamed version."""
    deps = {_dist(r) for r in _pyproject()["project"].get("dependencies", [])}
    assert deps, "pyproject.toml declares no core dependencies - nothing to compare"
    pinned = {_dist(p.split("==", 1)[0]) for p in _venv_ap_packages() if "==" in p}
    missing = sorted(deps - pinned)
    assert not missing, f"core dependencies not pinned in the ~/venv-ap install: {missing}"


def test_setup_sitl_keeps_lf_line_endings():
    """bash in WSL reads a CRLF script as commands ending in \\r (commit d50035f)."""
    assert b"\r" not in SETUP_SITL.read_bytes()


def test_the_design_doc_records_the_same_pin():
    sha = _pinned_sha(_setup_text())
    assert sha, "no pin to compare"
    assert DESIGN_DOC.is_file(), f"{DESIGN_DOC} missing"
    assert sha in DESIGN_DOC.read_text(encoding="utf-8"), (
        f"docs/DESIGN-python-versions.md does not mention the pinned {sha}")


# --------------------------------------------------------------------------- #
# Behavioural: run the script against throwaway repositories
# --------------------------------------------------------------------------- #
_BASH = []


def _bash():
    """A POSIX bash whose $HOME is the fixture. On Windows that is Git Bash.
    A WSL launcher (System32\\bash.exe, WindowsApps\\bash.exe, or one under any
    other name) would act on WSL's real ~/ardupilot instead - never acceptable
    here. So the safety rests on what a candidate does, not on its name or on
    PATH order: it is accepted only if, handed a fresh HOME, it reads back a
    token from a file in that directory. WSL does not inherit HOME, so it reads
    /home/<user>/probe and fails. (Git Bash maps the Windows path to /tmp/...,
    which is why the token is compared, not the path.) The name filter only
    spares starting WSL for nothing."""
    if _BASH:
        return _BASH[0]
    cands = [shutil.which("bash")]
    if os.name == "nt":
        cands += [r"C:\Program Files\Git\bin\bash.exe", r"C:\Program Files\Git\usr\bin\bash.exe"]
    probe = _mkdtemp("envpins_probe_")
    token = probe.name
    (probe / "probe").write_text(token, encoding="utf-8")
    found = None
    for c in cands:
        if not c or not Path(c).is_file() or re.search(r"system32|windowsapps", c, re.I):
            continue
        try:
            r = subprocess.run([c, "-c", 'cat "$HOME/probe"'], env=dict(os.environ, HOME=str(probe)),
                               capture_output=True, text=True, timeout=60)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if r.returncode == 0 and r.stdout.strip() == token:
            found = c
            break
    _BASH.append(found)
    return found


def _need_bash():
    if _bash() is None or shutil.which("git") is None:
        _skip("no POSIX bash + git on this machine; setup_sitl.sh not exercised")
        return False
    if "--verify" not in _setup_text():
        raise AssertionError("setup_sitl.sh has no --verify mode")
    return True


def _g(cwd, *args):
    """git in a fixture repo, isolated from the user's identity and autocrlf."""
    r = subprocess.run(["git", "-C", str(cwd), "-c", "user.name=t", "-c", "user.email=t@t",
                        "-c", "commit.gpgsign=false", "-c", "core.autocrlf=false",
                        "-c", "protocol.file.allow=always", *args],
                       capture_output=True, text=True, timeout=120)
    if r.returncode:
        raise AssertionError(f"fixture git {' '.join(args)} failed: {r.stderr.strip()}")
    return r.stdout.strip()


def _stub_venv(home):
    """A stub ~/venv-ap: pip and activate do nothing (pip logs its arguments),
    so the script's network and compiler steps are replaced, its git steps not."""
    b = home / "venv-ap" / "bin"
    b.mkdir(parents=True)
    (b / "pip").write_bytes(b'#!/bin/sh\necho "$*" >> "$HOME/pip_ran"\n')
    (b / "activate").write_bytes(b"# stub\n")
    os.chmod(b / "pip", 0o755)


def _write_binary(ap, tag=None):
    """The fake build output; tag=None writes a binary with no version banner."""
    b = ap / "build" / "sitl" / "bin" / "arducopter"
    b.parent.mkdir(parents=True, exist_ok=True)
    banner = b"ArduCopter V4.5.7 (" + tag.encode() + b")" if tag else b"no banner here"
    b.write_bytes(b"#!/bin/sh\n\x00\x7fELF junk\x00" + banner + b"\x00tail")
    os.chmod(b, 0o755)


def _fixture(banner_sha8=None, binary=True, banner=True):
    """A one-commit checkout with a fake binary: enough for --verify. The home
    gets a stub ~/venv-ap too, so that if a regression ever broke the --verify
    dispatch, the fall-through into the full setup would fail offline at once
    (no ./waf here) instead of reaching PyPI."""
    home = _mkdtemp("envpins_")
    _stub_venv(home)
    ap = home / "ardupilot"
    ap.mkdir()
    _g(ap, "init", "-q")
    (ap / "version.h").write_bytes(b"#define THISFIRMWARE \"ArduCopter V4.5.7\"\n")
    _g(ap, "add", "version.h")
    _g(ap, "commit", "-q", "-m", "fixture")
    sha = _g(ap, "rev-parse", "HEAD")
    if binary:
        _write_binary(ap, (banner_sha8 or sha[:8]) if banner else None)
    return home, ap, sha


def _verify(home, sha, ref="fixture-tag", **extra):
    # ARDUPILOT_URL points at nothing: a broken --verify dispatch that fell
    # through to `git clone` must fail in milliseconds, never fetch GitHub.
    env = dict(os.environ, HOME=str(home), ARDUPILOT_SHA=sha, ARDUPILOT_REF=ref,
               ARDUPILOT_URL=str(home / "no-upstream"), **extra)
    r = subprocess.run([_bash(), str(SETUP_SITL), "--verify"], env=env,
                       capture_output=True, text=True, timeout=120)
    return r.returncode, r.stdout + r.stderr


# Stand-in for ArduPilot's waf. It "builds" a binary whose version banner carries
# the first 8 hex digits of HEAD, as waf's build/sitl/ap_version.h does, and logs
# every build to $HOME/waf_ran so a test can tell whether a build was attempted.
FAKE_WAF = r"""#!/bin/sh
if [ "$1" = configure ]; then echo "fake waf: configured"; exit 0; fi
echo "$*" >> "$HOME/waf_ran"
if [ -n "$FAKE_WAF_FAIL" ]; then echo "fake waf: build error" >&2; exit 2; fi
sha="${FAKE_WAF_SHA8:-$(git rev-parse HEAD | cut -c1-8)}"
mkdir -p build/sitl/bin
printf '#!/bin/sh\nArduCopter V4.5.7 (%s)\n' "$sha" > build/sitl/bin/arducopter
chmod +x build/sitl/bin/arducopter
"""

_UP = {}


def _upstream():
    """Upstream stand-in, built once: commit `rc1`, then the release `pin`
    (annotated tag `pin-tag`), then master one commit later with a different
    version.h and a newer mavlink.
    modules/mavlink nests pymavlink, as ArduPilot's does; the release records
    pymavlink gen 1, master gen 2 - the drift found in the real ~/ardupilot."""
    if _UP:
        return _UP
    up = _mkdtemp("envpins_up_")

    def repo(name):
        d = up / name
        d.mkdir()
        _g(d, "init", "-q")
        (d / ".gitattributes").write_bytes(b"* -text\n")   # no CRLF in the fake waf
        return d

    pym = repo("pymavlink")
    (pym / "gen.py").write_bytes(b"GEN = 1\n")
    _g(pym, "add", "-A")
    _g(pym, "commit", "-qm", "generator 1")
    p1 = _g(pym, "rev-parse", "HEAD")
    (pym / "gen.py").write_bytes(b"GEN = 2\n")
    _g(pym, "commit", "-qam", "generator 2")
    p2 = _g(pym, "rev-parse", "HEAD")

    mav = repo("mavlink")
    _g(mav, "submodule", "add", "-q", "../pymavlink", "pymavlink")
    _g(mav / "pymavlink", "checkout", "-q", p1)
    _g(mav, "add", "-A")
    _g(mav, "commit", "-qm", "mavlink 1 (generator 1)")
    m1 = _g(mav, "rev-parse", "HEAD")
    _g(mav / "pymavlink", "checkout", "-q", p2)
    _g(mav, "add", "-A")
    _g(mav, "commit", "-qm", "mavlink 2 (generator 2)")
    m2 = _g(mav, "rev-parse", "HEAD")

    ap = repo("ardupilot")
    (ap / "version.h").write_bytes(b'#define THISFIRMWARE "ArduCopter V4.5.7-rc1"\n')
    (ap / "waf").write_bytes(FAKE_WAF.encode())
    _g(ap, "add", "-A")
    _g(ap, "update-index", "--chmod=+x", "waf")
    _g(ap, "commit", "-qm", "release candidate")
    rc1 = _g(ap, "rev-parse", "HEAD")
    (ap / "version.h").write_bytes(b'#define THISFIRMWARE "ArduCopter V4.5.7"\n')
    _g(ap, "submodule", "add", "-q", "../mavlink", "modules/mavlink")
    _g(ap / "modules" / "mavlink", "checkout", "-q", m1)
    _g(ap, "add", "-A")
    _g(ap, "commit", "-qm", "release stand-in")
    _g(ap, "tag", "-a", "pin-tag", "-m", "release stand-in")
    pin = _g(ap, "rev-parse", "HEAD")
    (ap / "version.h").write_bytes(b'#define THISFIRMWARE "ArduCopter V9.9.0-dev"\n')
    _g(ap / "modules" / "mavlink", "checkout", "-q", m2)
    _g(ap, "add", "-A")
    _g(ap, "commit", "-qm", "master moves on")
    master = _g(ap, "rev-parse", "HEAD")
    _UP.update(dir=up, url=ap.as_posix(), rc1=rc1, pin=pin, master=master, p1=p1, p2=p2)
    return _UP


def _home():
    """A fresh $HOME with a stub ~/venv-ap (see _stub_venv)."""
    home = _mkdtemp("envpins_home_")
    _stub_venv(home)
    return home


def _setup(home, sha, ref="pin-tag", **extra):
    up = _upstream()
    env = dict(os.environ, HOME=str(home), ARDUPILOT_SHA=sha, ARDUPILOT_REF=ref,
               ARDUPILOT_URL=up["url"],
               # Local-path submodules are refused by default since git 2.38.1.
               GIT_CONFIG_COUNT="1", GIT_CONFIG_KEY_0="protocol.file.allow",
               GIT_CONFIG_VALUE_0="always", **extra)
    r = subprocess.run([_bash(), str(SETUP_SITL)], env=env, capture_output=True,
                       text=True, timeout=300)
    return r.returncode, r.stdout + r.stderr


def _bad_submodules(ap):
    return [ln for ln in _g(ap, "submodule", "status", "--recursive").splitlines()
            if ln[:1] in "-+U"]


def test_verify_passes_only_when_commit_and_binary_match_and_writes_nothing():
    if not _need_bash():
        return
    home, ap, sha = _fixture()
    # Make the stat cache stale, as it is on any real tree touched since its
    # last `git status`: a git command that may refresh the index now will.
    t = time.time() - 100
    os.utime(ap / "version.h", (t, t))
    idx = hashlib.sha256((ap / ".git" / "index").read_bytes()).hexdigest()
    rc, out = _verify(home, sha)
    assert rc == 0 and "PIN OK" in out, out
    assert f"ArduCopter V4.5.7 ({sha[:8]})" in out, "the measured banner is not printed"
    assert hashlib.sha256((ap / ".git" / "index").read_bytes()).hexdigest() == idx, (
        "--verify rewrote the git index; it is meant to be read-only")


def test_verify_refuses_a_checkout_at_another_commit():
    if not _need_bash():
        return
    home, _, sha = _fixture()
    other = ("0" * 40) if sha != "0" * 40 else ("1" * 40)
    rc, out = _verify(home, other)
    assert rc == 1 and "HEAD is not the pinned commit" in out and "PIN OK" not in out, out


def test_verify_refuses_a_binary_built_from_another_commit():
    if not _need_bash():
        return
    home, _, sha = _fixture(banner_sha8="deadbeef")
    rc, out = _verify(home, sha)
    assert rc == 1 and "binary was not built from the pinned commit" in out, out


def test_verify_refuses_edited_tracked_sources():
    if not _need_bash():
        return
    home, ap, sha = _fixture()
    (ap / "version.h").write_bytes(b"#define THISFIRMWARE \"ArduCopter V9.9.9\"\n")
    rc, out = _verify(home, sha)
    assert rc == 1 and "tracked ArduPilot files modified" in out, out


def test_verify_refuses_a_missing_build_and_a_missing_checkout():
    if not _need_bash():
        return
    home, _, sha = _fixture(binary=False)
    rc, out = _verify(home, sha)
    assert rc == 1 and "missing (not built)" in out, out
    empty = _home()                                   # no ~/ardupilot at all
    rc, out = _verify(empty, sha)
    assert rc == 1 and "not a git checkout" in out, out


def test_verify_refuses_a_binary_with_no_version_banner():
    """No banner proves nothing about the commit; it must not pass as a match."""
    if not _need_bash():
        return
    home, _, sha = _fixture(banner=False)
    rc, out = _verify(home, sha)
    assert rc == 1 and "<no version banner found>" in out, out
    assert "binary was not built from the pinned commit" in out and "PIN OK" not in out, out


def test_verify_fails_when_git_status_itself_fails():
    """`git status` crashing prints nothing, and nothing would read as "no
    tracked file edited". The fault is injected through git's own config:
    a bad boolean for status.renames makes only `git status` exit 128
    (rev-parse, describe and submodule status do not read that key)."""
    if not _need_bash():
        return
    home, _, sha = _fixture()
    rc, out = _verify(home, sha)
    assert rc == 0 and "PIN OK" in out, "control run failed: " + out
    rc, out = _verify(home, sha, GIT_CONFIG_COUNT="1", GIT_CONFIG_KEY_0="status.renames",
                      GIT_CONFIG_VALUE_0="bogus")
    assert rc == 1 and "git status failed" in out and "PIN OK" not in out, out


def test_verify_fails_when_git_cannot_list_the_submodules():
    """A gitlink with no .gitmodules entry makes `git submodule status` exit
    128 with an empty listing. Read through `| grep || true`, that was "no bad
    submodules" and PIN OK. The gitlink's directory exists and is empty, as an
    uninitialised submodule's is, so `git status` sees nothing wrong and only
    the submodule check can catch it."""
    if not _need_bash():
        return
    home, ap, _ = _fixture()
    _g(ap, "update-index", "--add", "--cacheinfo", f"160000,{'1' * 40},orphan_sub")
    _g(ap, "commit", "-qm", "gitlink without a .gitmodules entry")
    (ap / "orphan_sub").mkdir()
    sha = _g(ap, "rev-parse", "HEAD")
    _write_binary(ap, sha[:8])
    rc, out = _verify(home, sha)
    assert rc == 1 and "git submodule status failed" in out and "PIN OK" not in out, out


def test_setup_builds_the_pin_with_nested_submodules_and_reruns_cleanly():
    """Fresh clone of a newer master -> pinned release, every submodule level at
    the release's commits, built, verified. A second run changes nothing."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["pin"])
    ap = home / "ardupilot"
    assert rc == 0, out[-1500:]
    assert "PIN OK" in out and "=== DONE ===" in out, "setup did not end in the pin check"
    assert _g(ap, "rev-parse", "HEAD") == up["pin"], "built tree is not the pinned commit"
    assert not _bad_submodules(ap), _bad_submodules(ap)
    nested = _g(ap / "modules" / "mavlink" / "pymavlink", "rev-parse", "HEAD")
    assert nested == up["p1"], "nested pymavlink is not at the commit the release records"
    assert (home / "waf_ran").is_file(), "the build step never ran"
    rc2, out2 = _setup(home, up["pin"])
    assert rc2 == 0 and "already cloned" in out2 and "PIN OK" in out2, out2[-1500:]


def test_setup_refuses_a_ref_that_resolves_to_another_commit_before_building():
    """A moved tag or another remote: the pinned SHA is the authority, not the
    name. Here the pin says rc1 while `pin-tag` names the release commit."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["rc1"])             # clone lands on master, not rc1
    assert rc != 0, out[-1500:]
    assert "refusing to build an unpinned tree" in out, out[-1500:]
    assert not (home / "waf_ran").exists(), "it built a tree it had just refused"


def test_setup_fails_when_the_built_binary_is_not_the_pin():
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["pin"], FAKE_WAF_SHA8="deadbeef")
    assert rc != 0 and "binary was not built from the pinned commit" in out, out[-1500:]
    assert "=== DONE ===" not in out


def test_setup_stops_on_a_failed_rebuild_even_with_an_old_binary_on_disk():
    """Without pipefail, `./waf copter | tail` reports tail's success, and the
    good binary from the last build would then pass the final check."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["pin"])
    assert rc == 0, out[-1500:]
    rc2, out2 = _setup(home, up["pin"], FAKE_WAF_FAIL="1")
    assert rc2 != 0 and "=== DONE ===" not in out2, (
        "a failed rebuild ended in DONE: " + out2[-800:])


def test_setup_refuses_rather_than_discards_local_edits():
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    _g(home, "clone", "-q", up["url"], "ardupilot")             # at master
    edited = b"/* local experiment */\n"
    (home / "ardupilot" / "version.h").write_bytes(edited)
    rc, out = _setup(home, up["pin"])
    assert rc != 0, out[-1500:]
    assert (home / "ardupilot" / "version.h").read_bytes() == edited, "local edit was discarded"
    assert not (home / "waf_ran").exists(), "it built over an unresolved checkout"
    # It must stop at the refused checkout with git's own reason. Carrying on
    # to the SHA comparison would blame a "moved tag" that never moved.
    assert "moved tag" not in out, "a refused checkout was reported as a moved tag: " + out[-800:]


def test_verify_refuses_a_nested_submodule_at_another_commit():
    """The state found in the real ~/ardupilot on 2026-10-06: HEAD and binary
    right, modules/mavlink/pymavlink left at a newer commit."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["pin"])
    assert rc == 0, out[-1500:]
    _g(home / "ardupilot" / "modules" / "mavlink" / "pymavlink", "checkout", "-q", up["p2"])
    rc, out = _verify(home, up["pin"], ref="pin-tag")
    assert rc == 1 and "submodules differ" in out and "modules/mavlink/pymavlink" in out, out
    assert "PIN OK" not in out


def test_setup_fetches_the_pin_tag_when_the_checkout_lacks_it():
    """An existing checkout made without tags (or before the release was
    tagged) still reaches the pin: the script fetches tags when the ref does
    not resolve locally, instead of failing on an unknown name."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    ap = home / "ardupilot"
    _g(home, "clone", "-q", "--no-tags", up["url"], "ardupilot")
    _g(ap, "-c", "advice.detachedHead=false", "checkout", "-q", up["rc1"])
    assert not _g(ap, "tag", "--list", "pin-tag"), "fixture clone already has the tag"
    rc, out = _setup(home, up["pin"])
    assert rc == 0 and "PIN OK" in out, out[-1500:]
    assert _g(ap, "rev-parse", "HEAD") == up["pin"]


def test_verify_refuses_modified_content_inside_a_nested_submodule():
    """Right commits everywhere, but an edited pymavlink/gen.py generates
    different MAVLink headers: not the pinned firmware. Same for a first-level
    submodule. Untracked leftovers inside a submodule (crash dumps) are not
    edits and must not fail the check - shown first, as the control."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["pin"])
    assert rc == 0, out[-1500:]
    ap = home / "ardupilot"
    mav = ap / "modules" / "mavlink"
    pym = mav / "pymavlink"
    (ap / "dumpcore.sh_arducopter.1234.out").write_bytes(b"core")
    (pym / "dumpstack.sh_arducopter.1234.out").write_bytes(b"stack")
    rc, out = _verify(home, up["pin"], ref="pin-tag")
    assert rc == 0 and "PIN OK" in out, "untracked leftovers were counted as edits: " + out
    (pym / "gen.py").write_bytes(b"GEN = 'edited generator'\n")
    rc, out = _verify(home, up["pin"], ref="pin-tag")
    assert rc == 1 and "tracked ArduPilot files modified" in out and "PIN OK" not in out, (
        "edited nested pymavlink passed: " + out)
    _g(pym, "checkout", "--", "gen.py")
    (mav / ".gitmodules").write_bytes((mav / ".gitmodules").read_bytes() + b"# edited\n")
    rc, out = _verify(home, up["pin"], ref="pin-tag")
    assert rc == 1 and "tracked ArduPilot files modified" in out and "PIN OK" not in out, (
        "edited modules/mavlink passed: " + out)


def test_verify_writes_no_index_inside_submodules_either():
    """The dirty check now runs `git status` inside every submodule. With
    stale stat caches at all three levels, a status that may refresh would
    rewrite those indexes; --verify must leave every one byte-identical."""
    if not _need_bash():
        return
    up, home = _upstream(), _home()
    rc, out = _setup(home, up["pin"])
    assert rc == 0, out[-1500:]
    ap = home / "ardupilot"
    mav = ap / "modules" / "mavlink"
    t = time.time() - 100
    for f in (ap / "version.h", mav / ".gitmodules", mav / "pymavlink" / "gen.py"):
        os.utime(f, (t, t))
    idx = sorted((ap / ".git").rglob("index"))
    assert len(idx) == 3, f"expected superproject + 2 submodule indexes, found {idx}"
    before = [hashlib.sha256(p.read_bytes()).hexdigest() for p in idx]
    rc, out = _verify(home, up["pin"], ref="pin-tag")
    assert rc == 0 and "PIN OK" in out, out
    after = [hashlib.sha256(p.read_bytes()).hexdigest() for p in idx]
    changed = [str(p.relative_to(ap)) for p, b, a in zip(idx, before, after) if b != a]
    assert not changed, f"--verify rewrote {changed}; it is meant to be read-only"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        n_skip = len(SKIPPED)
        try:
            fn()
            print(f"{'SKIP' if len(SKIPPED) > n_skip else 'PASS'}  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    passed = len(fns) - failed - len(SKIPPED)
    print(f"\n{passed}/{len(fns)} passed" + (f", {len(SKIPPED)} skipped (not passed)" if SKIPPED else ""))
    sys.exit(1 if failed else 0)
