"""Time the Shield against the grant's tick budgets, on real flight logs, on any host.

    python tools/profile_shield_tick.py pack                    # once, where demo/out exists
    python tools/profile_shield_tick.py profile --topology dev  # the desktop baseline
    python tools/profile_shield_tick.py profile --topology hil  # on the Jetson Orin (refused elsewhere)
    python tools/profile_shield_tick.py facts                   # what this host is, as JSON
    python tools/profile_shield_tick.py attach PROFILE.json     # the summary a run's metrics.json carries

WHY THIS EXISTS

The grant states two budgets for the Safety Shield, both at the monitor's 10 Hz:

  * Safety Shield page, acceptance KPIs: "Monitor tick budget 100 ms (10 Hz)
    end-to-end".
  * Policy DSL page, acceptance checks: "Monitor query latency at 10 Hz with 50
    active rules <= 5 ms (10% of monitor budget)", and the Safety Shield page
    states the load: "~50 active rules and 50 future-pose checks per tick".

and the PI's reference design says where they are checked: "10 Hz Shield
monitor budget will be validated by Orin profiling" (reference
docs/02-implementation/overview.md). experiments/bench_shield_50rules.py
already times the Shield on SYNTHETIC samples (a fence grid and random
headings). This tool does the other half the audit asked for (cards ARCH-26,
X-07): the same budgets on the states and actions of flights that actually
flew, through the same code, on whatever host runs it - the desktop now, the
Jetson Orin in the hil topology, unchanged.

THE TICK PACK (portable input)

Flight logs live under demo/out/, which is gitignored and is not on the Orin.
`pack` therefore extracts, once, everything the Shield needs to replay a
flight into one JSON file that is committed with this tool
(deploy/evidence/shield_tick_pack.json):

  * per tick: position, heading, the action the Shield was given (the rate-
    limited `smooth` action where the rail logged one, else `raw`), the subject
    position the rail passed to Shield.set_subject (or none), and the rule ids
    the rail logged as violated;
  * the policy the flight flew under, resolved from its manifest's
    policy_hash through guardrail.manifest.resolve_run_policy (never "the
    nearest policy"), stored inline so the Orin needs no policy files;
  * the obstacle map the rail used for ObstacleClearance rules, selected the
    way demo/follow_vlm.py selects it (demo/occ_bands.select_for_band), stored
    inline (zlib + base64).

`profile` refuses a pack whose stored policy document no longer matches the
SHA-256 it was packed with (an edited pack is not the flight).

Each flight's `rail` is its manifest topology read under today's names
(guardrail.manifest.normalize_topology): the three dev-rail flights recorded
the retired label "canonical-hil", which means `dev`, and the pack says `dev`.
The label as recorded is kept beside it as `rail_recorded`. None of these
flights is KPI-bearing: the grant reserves KPI figures for hil.

WHAT IS TIMED

For every flight, three policy arms x two horizons:

  arms      as_flown     the flight's own policy
            design_load  the flight's own rules plus square no-fly zones placed
                         next to the flown path until there are exactly 50
                         rules - the grant's stated load on real dynamics. The
                         zones never contain a logged position, so the vehicle
                         is never "already inside"; the forecasts run into them.
            floor        the flight's own policy with every polygon fence
                         removed: the cost of everything that is not fence
                         geometry (the null for the fence index)
  horizons  flown        3.0 s at 0.5 s, what every rail in this repo flies
            grant        5.0 s at 0.1 s, the grant's "50 future-pose checks"

and per tick two calls: `Shield._check` (the monitor query the 5 ms budget is
about) and `Shield.filter` (the whole Shield step: check, repair, re-check,
rescue). The headline is design_load at the grant horizon, because that is the
condition the grant states its budgets at.

THE NULLS (a fast number alone says nothing)

  * floor: the same ticks with no fences. A design_load time close to the
    floor is only credible if the fences were actually hit, so...
  * the design_load arm must raise a geofence violation on at least one tick
    of every flight, or the run FAILS: a Shield that skipped its fences would
    post the best time in the table;
  * the floor arm must raise none, or it is not a floor;
  * as_flown at the flown horizon is compared with what the rail logged: the
    number of ticks where the replayed violated-rule set equals the logged
    one. A replay that never flags anything also "agrees" on every tick whose
    logged set is empty, so the count is split: agreement on the ticks that
    logged a violation (the informative part) and the never-flagging null (the
    empty ticks). A flight that logged no violation at all verifies nothing by
    agreement - not its obstacle map, not its subject - and says so. The code
    has changed since some of these flights, so agreement is reported, not
    gated.

HOST CONDITIONS

A fixed CPU workload is timed before and after every profile (`host_speed`):
the reference that makes the desktop's and the Orin's numbers comparable, and
that shows when a host changed speed mid-run. A run whose reference moved by
more than 1.5x either way FAILS: its timings mix two host speeds, so its
verdict is undetermined (`attach` reports `within_budget: null`). On the lab's
hybrid-core desktop (an i7-14700, 8 performance cores with two hardware
threads each, logical CPUs 0-15, and 12 efficiency cores, 16-27) unpinned runs
moved to the efficiency cores mid-run and measured 2-3x slower, so `--cpus`
pins the process and records which CPUs it ran on. Pin one logical CPU per
physical core (`--cpus 0,2,4,6,8,10,12,14` for the performance cores): two
threads of one core share it, and a co-scheduled process on the sibling
changes the timing.

WHAT THIS DOES NOT MEASURE

The ROS 2 and MAVROS hops of the end-to-end tick: subscription callbacks, DDS,
the MAVLink round trip. Those exist only on a live rail. Where a rail logged
its own tick period (`t` per row) or its own Shield time (`ms.shield`), the
profile carries them as `rail_logged`, labelled as measured on that rail's
host, under that rail's load.

HONESTY RULES

  * `--topology hil` and `--topology flight` are refused unless this host is
    the grant's Jetson Orin by the SAME rule that labels a run
    (guardrail.manifest.collect_host_evidence, judged by guardrail.manifest's
    Orin check: aarch64, an Orin named by the device tree, the L4T release or
    the JetPack version read). A Xavier, a Nano or a bare aarch64 board is
    refused, so a profile can never say hil on a host whose runs would say dev.
    A desktop number labelled hil is exactly the failure the manifest gate
    exists for.
  * `--topology` selects which budget file this is; it is a claim the host
    must support, never a label the tool trusts.
  * The verdict is p99 against each budget. The grant states a budget, not a
    percentile; p99 is this project's choice, and the maximum and the count of
    ticks over budget are reported beside it.
  * Nothing here guesses a value. A field that cannot be read is None and says
    why.

OUTPUT LOCATION

`profile` writes `<evidence>/<topology>/shield_tick_profile.json`, where
<evidence> is $VLAGUARD_EVIDENCE_OUT or deploy/evidence. The Orin compose file
points it at deploy/evidence/incoming/ (git-ignored), so a run on the Orin
never modifies a tracked file - a modified checkout would stamp every later
manifest on that host `-dirty`. The desktop moves reviewed files into
deploy/evidence/<topology>/ and commits them (docs/RUNBOOK-orin-hil.md).
"""
from __future__ import annotations

import argparse
import base64
import gc
import hashlib
import json
import math
import os
import platform
import shlex
import statistics
import subprocess
import sys
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# /2: host facts judged by guardrail.manifest's Orin rule (`jetson.is_orin`),
# agreement split into its informative part and the never-flagging null, a
# changed host speed fails the run, headline carries max and over-budget count.
PROFILE_SCHEMA = "vlaguard.shield_tick_profile/2"
# /2: `rail` normalised (guardrail.manifest.normalize_topology), `rail_recorded`
# keeps the label the flight wrote.
PACK_SCHEMA = "vlaguard.shield_tick_pack/2"

# The grant's numbers (see the module docstring for the page each comes from).
# tests/test_profile_shield_tick.py pins each one to the grant's text.
TICK_BUDGET_MS = 100.0
QUERY_BUDGET_MS = 5.0
DESIGN_LOAD_RULES = 50
# A run whose host-speed reference moved by more than this factor either way
# mixed two host speeds.
HOST_SPEED_TOLERANCE = 1.5
VERDICT_BASIS = ("p99 of every tick against the budget. The grant states a budget, "
                 "not a percentile: p99 is this project's choice, and the maximum and "
                 "the count of ticks over budget are reported beside it")

EVIDENCE_OUT_ENV = "VLAGUARD_EVIDENCE_OUT"

# (lookahead_s, dt). "flown" is what sitl/ros2_shield_node.py,
# sitl/run_sitl_demo.py, demo/run_demo.py, demo/real_vla_demo.py and
# demo/follow_vlm.py all construct; "grant" is the Safety Shield page's
# "5 s of predicted trajectory at 10 Hz = 50 future poses".
HORIZONS = {"flown": (3.0, 0.5), "grant": (5.0, 0.1)}
ARMS = ("as_flown", "design_load", "floor")
HEADLINE = ("design_load", "grant")

TOPOLOGIES = ("dev", "hil", "flight")
JETSON_TOPOLOGIES = ("hil", "flight")

DEFAULT_PACK = ROOT / "deploy" / "evidence" / "shield_tick_pack.json"
DEFAULT_DEMO_OUT = ROOT / "demo" / "out"
# Two rails, four kinds of rule load: the dev rail (ArduPilot SITL + MAVROS 2)
# with a static no-fly zone, with a zone hot-applied mid-flight, and with a
# subject stand-off; and a Project AirSim city flight with a no-fly zone, a
# per-tick subject and an obstacle-clearance map.
DEFAULT_FLIGHTS = ("ros2_shield_on", "ros2_shield_on_dynamic", "ros2_ped_on",
                   "citylife_redcar_nfz1")
DEFAULT_MAX_TICKS = 600

COLUMNS = ("t", "x", "y", "up", "yaw_deg", "vx", "vy", "vz_up", "yaw_rate",
           "sx", "sy", "s_class", "logged_rules", "logged_shield_ms")

# Synthetic-fence geometry for the design-load arm. Same square and margin as
# experiments/bench_shield_50rules.py. The placement differs on purpose: the
# bench samples states around a fixed grid, while here the states are fixed (a
# real flight) and the zones are placed along the path, on a fine grid of
# candidate corners, closest first, never overlapping each other, and never
# with a logged position inside a margin ring. A first version reused the
# bench's 40 m grid: on the two slow flights no zone came within forecast reach
# at the 3 s horizon, and the sanity check below refused the run.
LOAD_SIDE_M, LOAD_MARGIN_M = 12.0, 2.0
LOAD_CANDIDATE_STEP_M = 4.0
LOAD_MIN_GAP_M = 1.0
LOAD_CEILING_M = 500.0


class PackRefused(ValueError):
    """A flight that cannot be packed honestly (missing log, unresolved policy)."""


# --------------------------------------------------------------------------- #
# Host facts
# --------------------------------------------------------------------------- #
def _read_text(path: str) -> str | None:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").replace("\x00", "").strip()
    except OSError:
        return None


def _nvpmodel() -> str | None:
    """The Orin's power mode, when nvpmodel is on PATH (on the host; not inside
    the containers, which is why the runbook asks for it in --note)."""
    try:
        r = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, timeout=10)
        if r.returncode == 0:
            return " | ".join(line.strip() for line in r.stdout.splitlines() if line.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def _orin_rule():
    """guardrail.manifest's Orin check: the rule that decides whether a run on
    this host may be labelled hil or flight. A public name is used when the
    manifest module offers one; until then its private helper, which is the
    same function the manifest gate calls."""
    from guardrail import manifest as M
    return getattr(M, "orin_missing", None) or M._orin_missing


def jetson_facts(read=None, machine=None, jetpack=None) -> dict:
    """Is this host the grant's Jetson Orin? Read, never declared, and judged
    by the SAME rule as a run's topology label.

    guardrail.manifest.collect_host_evidence reads the device tree's board name
    (/proc/device-tree/model, which Docker masks inside a default container:
    the Orin compose file unmasks it with security_opt systempaths=unconfined),
    /etc/nv_tegra_release (mounted read-only into the Orin's containers) and
    the JetPack package; guardrail.manifest's Orin check then requires
    aarch64, an Orin named by the device tree, and L4T or JetPack. A Jetson
    Xavier or Nano, or any aarch64 board, is not the hil machine. The readers
    are parameters so tests can describe any host without being one.

    The hostname collect_host_evidence also returns is dropped: a profile is
    committed and published, and the hardware class is what matters."""
    from guardrail.manifest import collect_host_evidence
    kw = {k: v for k, v in (("read", read), ("machine", machine), ("jetpack", jetpack))
          if v is not None}
    ev = collect_host_evidence(**kw)
    ev.pop("hostname", None)
    missing = list(_orin_rule()(ev))
    return {"is_orin": not missing, "missing": missing, **ev, "nvpmodel": _nvpmodel()}


def _cpu_model() -> str | None:
    info = _read_text("/proc/cpuinfo")
    if info:
        for line in info.splitlines():
            key = line.split(":", 1)[0].strip().lower()
            if key in ("model name", "hardware", "cpu model"):
                return line.split(":", 1)[1].strip()
        parts = sorted({line.split(":", 1)[1].strip() for line in info.splitlines()
                        if line.split(":", 1)[0].strip().lower() == "cpu part"})
        if parts:
            return "ARM CPU part " + ",".join(parts)
    return platform.processor() or None


def _lib_versions() -> dict:
    out = {}
    for name in ("numpy", "shapely", "pydantic", "yaml"):
        try:
            mod = __import__(name)
            out[name] = getattr(mod, "__version__", None)
        except Exception:                                    # noqa: BLE001
            out[name] = None
    try:
        import shapely
        out["geos"] = ".".join(map(str, shapely.geos_version))
    except Exception:                                        # noqa: BLE001
        out["geos"] = None
    return out


def host_facts(jetson: dict | None = None) -> dict:
    """The facts a measurement needs beside it: where it ran, on what.

    No hostname: the profile is meant to be committed and shared, and the
    hardware class is what matters, not the machine's name."""
    clock = time.get_clock_info("perf_counter")
    return {
        "os": platform.system(),
        "os_release": platform.release(),
        "machine": platform.machine(),
        "cpu_model": _cpu_model(),
        "cpu_count": os.cpu_count(),
        "python": platform.python_version(),
        "in_container": Path("/.dockerenv").exists(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "topology_env": os.environ.get("VLAGUARD_TOPOLOGY"),
        "libs": _lib_versions(),
        "perf_counter_resolution_s": clock.resolution,
        "jetson": jetson if jetson is not None else jetson_facts(),
    }


def calibrate(repeats: int = 5, clock=time.perf_counter) -> dict:
    """Time a fixed CPU workload (pure-Python arithmetic plus shapely point
    tests, the two things a Shield tick spends its time on). Median of
    `repeats`, in ms.

    Run before and after a profile. It is the host-speed reference beside the
    Shield numbers: on a shared desktop the same code measured 2-3x apart an
    hour apart (2026-10-06), and without a reference that difference reads as
    a property of the Shield. A large before/after change means the host's
    speed changed during the run."""
    from shapely.geometry import Point, Polygon
    poly = Polygon([(0, 0), (12, 0), (12, 12), (0, 12)]).buffer(2.0)
    pts = [Point((k * 0.37) % 20 - 4, (k * 0.61) % 20 - 4) for k in range(2000)]
    times = []
    for _ in range(repeats):
        t0 = clock()
        acc = 0.0
        for k in range(200_000):
            acc += math.sqrt(k * 0.5 + 1.0) * 1e-6
        hits = sum(1 for p in pts if poly.contains(p))
        times.append((clock() - t0) * 1e3)
    return {"workload": "200k sqrt-adds + 2000 shapely contains",
            "median_ms": statistics.median(times), "min_ms": min(times),
            "max_ms": max(times), "repeats": repeats, "_check": (round(acc, 3), hits)}


def parse_cpus(spec: str) -> list[int]:
    """'0-7,12' -> [0, 1, ..., 7, 12]."""
    out: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = (int(v) for v in part.split("-", 1))
            if hi < lo:
                raise ValueError(f"bad CPU range {part!r}")
            out.extend(range(lo, hi + 1))
        elif part:
            out.append(int(part))
    if not out:
        raise ValueError(f"no CPUs in {spec!r}")
    return sorted(set(out))


def pin_cpus(cpus: list[int]) -> list[int]:
    """Restrict this process to `cpus` and return the set now in force.

    On a hybrid-core desktop (this lab's i7-14700: 8 performance cores, 12
    efficiency cores) the OS may move a background process between the two
    kinds mid-run, and the Shield then measures 2-3x apart. Pinning makes the
    condition part of the record. Linux uses os.sched_setaffinity; elsewhere
    psutil, if installed. Refuses rather than pretending to pin."""
    if hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, set(cpus))
        return sorted(os.sched_getaffinity(0))
    try:
        import psutil
    except ImportError as exc:
        raise RuntimeError("cannot pin CPUs here: no os.sched_setaffinity and no psutil") from exc
    p = psutil.Process()
    p.cpu_affinity(cpus)
    return sorted(p.cpu_affinity())


def host_speed_record(before: dict, after: dict,
                      tolerance: float = HOST_SPEED_TOLERANCE) -> dict:
    """The calibration before and after a run, and whether the host kept its
    speed: the after/before ratio within `tolerance` either way."""
    ratio = after["median_ms"] / before["median_ms"]
    strip = [{k: v for k, v in c.items() if k != "_check"} for c in (before, after)]
    return {"before": strip[0], "after": strip[1], "after_over_before": ratio,
            "tolerance": tolerance, "steady": 1 / tolerance <= ratio <= tolerance}


def check_topology_claim(topology: str | None, facts: dict) -> list[str]:
    """Why this host may NOT label a measurement with `topology`. Empty = fine.

    hil and flight put the Shield on a Jetson Orin; a number taken anywhere
    else must not carry either label. "An Orin" is guardrail.manifest's rule
    (see jetson_facts), so a profile and the runs on the same host can never
    disagree about it."""
    if topology is None:
        return ["no topology given (--topology or VLAGUARD_TOPOLOGY)"]
    if topology not in TOPOLOGIES:
        return [f"{topology!r} is not one of the grant's topologies {TOPOLOGIES}"]
    reasons = []
    j = facts.get("jetson") or {}
    if topology in JETSON_TOPOLOGIES and not j.get("is_orin"):
        why = "; ".join(j.get("missing") or ["no Orin evidence was read"])
        reasons.append(f"{topology!r} runs the Shield on a Jetson Orin, and this host "
                       f"is not one by guardrail.manifest's rule: {why}")
    return reasons


def evidence_root() -> Path:
    """Where measurements are written: $VLAGUARD_EVIDENCE_OUT, else
    deploy/evidence. The Orin compose file sets deploy/evidence/incoming."""
    env = os.environ.get(EVIDENCE_OUT_ENV)
    return Path(env) if env else ROOT / "deploy" / "evidence"


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #
def write_lf(path: Path, text: str) -> None:
    """Write UTF-8 with LF line endings on every OS, so a file produced on the
    Windows desktop and one produced on the Orin hash the same way
    (Path.write_text writes CRLF on Windows)."""
    with Path(path).open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _code_revision() -> str:
    try:
        from guardrail.manifest import code_revision
        return code_revision()
    except Exception as exc:                                 # noqa: BLE001
        return f"unresolved ({type(exc).__name__})"


def _rel(path: Path) -> str:
    try:
        return Path(path).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return Path(path).as_posix()


def stats(ms: list[float], budget_ms: float | None = None) -> dict:
    """Median, p95, p99, max and mean; and against a budget, how many ticks
    went over. Percentiles are nearest-rank, so p99 of 100 samples is the
    99th smallest, never an interpolation that no tick actually took."""
    if not ms:
        return {"n": 0}
    s = sorted(ms)

    def rank(q: float) -> float:
        return s[min(len(s) - 1, max(0, int(math.ceil(q * len(s))) - 1))]

    out = {"n": len(s), "median_ms": statistics.median(s), "p95_ms": rank(0.95),
           "p99_ms": rank(0.99), "max_ms": s[-1], "mean_ms": statistics.fmean(s)}
    if budget_ms is not None:
        out["budget_ms"] = budget_ms
        out["over_budget"] = sum(1 for v in s if v > budget_ms)
        # p99 is the project's choice (VERDICT_BASIS); max and over_budget
        # stay beside it so no reader has to trust the percentile.
        out["verdict"] = "within" if out["p99_ms"] <= budget_ms else "over"
    return out


# --------------------------------------------------------------------------- #
# Policies for the three arms
# --------------------------------------------------------------------------- #
def design_load_policy(policy, xy: list[tuple[float, float]],
                       total: int = DESIGN_LOAD_RULES):
    """The flight's own rules plus square no-fly zones along the flown path,
    up to `total` rules. Returns (policy, n_added).

    Candidate corners lie on a fixed grid (multiples of the step, so the result
    is deterministic). A candidate qualifies when no logged position is within
    LOAD_MIN_GAP_M of its margin ring; qualifying squares are taken nearest to
    the path first, skipping any whose ring would overlap one already taken.
    A policy that already has `total` rules or more is returned unchanged with
    n_added = 0."""
    import numpy as np
    from guardrail.models import PolygonFence

    own = list(policy.constraints)
    need = total - len(own)
    if need <= 0:
        return policy.model_copy(deep=True), 0
    pts = np.asarray(xy, dtype=float)
    if pts.size == 0:
        raise ValueError("no positions to place the design-load zones around")
    step = LOAD_CANDIDATE_STEP_M
    ring = LOAD_SIDE_M + 2 * LOAD_MARGIN_M
    chosen: list[tuple[float, float]] = []
    # A short path has room for fewer zones beside it than a long one, so the
    # search area grows until `need` fit. Closest-first selection means the
    # growth only ever adds zones farther out; the near ones do not move.
    for grow in range(1, 9):
        pad = 2 * grow * ring
        gxs = np.arange(math.floor((pts[:, 0].min() - pad) / step) * step,
                        pts[:, 0].max() + pad, step)
        gys = np.arange(math.floor((pts[:, 1].min() - pad) / step) * step,
                        pts[:, 1].max() + pad, step)
        cands = []
        for gx in gxs:
            lo_x, hi_x = gx - LOAD_MARGIN_M, gx + LOAD_SIDE_M + LOAD_MARGIN_M
            dx = np.maximum(np.maximum(lo_x - pts[:, 0], 0.0), pts[:, 0] - hi_x)
            for gy in gys:
                lo_y, hi_y = gy - LOAD_MARGIN_M, gy + LOAD_SIDE_M + LOAD_MARGIN_M
                dy = np.maximum(np.maximum(lo_y - pts[:, 1], 0.0), pts[:, 1] - hi_y)
                d = float(np.min(np.hypot(dx, dy)))   # ring-to-path distance
                if d >= LOAD_MIN_GAP_M:
                    cands.append((round(d, 6), round(float(gx), 6), round(float(gy), 6)))
        cands.sort()
        chosen = []
        for _d, gx, gy in cands:
            if all(abs(gx - cx) >= ring or abs(gy - cy) >= ring for cx, cy in chosen):
                chosen.append((gx, gy))
                if len(chosen) == need:
                    break
        if len(chosen) == need:
            break
    if len(chosen) < need:
        raise ValueError(f"only {len(chosen)} non-overlapping zones fit near the "
                         f"path, {need} needed")
    added = []
    for k, (gx, gy) in enumerate(chosen):
        added.append(PolygonFence(
            id=f"design-load-{k:02d}", type="polygon_fence",
            vertices=[{"x": gx, "y": gy}, {"x": gx + LOAD_SIDE_M, "y": gy},
                      {"x": gx + LOAD_SIDE_M, "y": gy + LOAD_SIDE_M},
                      {"x": gx, "y": gy + LOAD_SIDE_M}],
            altitude_floor_m=0.0, altitude_ceiling_m=LOAD_CEILING_M,
            margin_m=LOAD_MARGIN_M))
    out = policy.model_copy(deep=True)
    out.constraints = list(out.constraints) + added
    out.policy_id = f"{policy.policy_id}+design-load-{total}"
    return out, len(added)


def floor_policy(policy):
    """The flight's own policy with every polygon fence removed."""
    from guardrail.models import PolygonFence
    out = policy.model_copy(deep=True)
    out.constraints = [c for c in out.constraints if not isinstance(c, PolygonFence)]
    out.policy_id = f"{policy.policy_id}+no-fences"
    return out


# --------------------------------------------------------------------------- #
# Packing real flights
# --------------------------------------------------------------------------- #
def _encode_map(m: dict) -> dict:
    import numpy as np
    occ = np.ascontiguousarray(np.asarray(m["occ"], dtype=np.uint8))
    return {"shape": list(occ.shape), "dtype": "uint8",
            "zlib_b64": base64.b64encode(zlib.compress(occ.tobytes(), 9)).decode("ascii"),
            "res": float(m["res"]), "ox": float(m["ox"]), "oy": float(m["oy"])}


def decode_map(enc: dict | None) -> dict | None:
    if not enc:
        return None
    import numpy as np
    raw = zlib.decompress(base64.b64decode(enc["zlib_b64"]))
    occ = np.frombuffer(raw, dtype=np.uint8).reshape(enc["shape"]).copy()
    return {"occ": occ, "res": enc["res"], "ox": enc["ox"], "oy": enc["oy"]}


def _local_path(p: str) -> Path:
    """A path recorded on this machine (often absolute Windows) as one that
    resolves under this checkout."""
    cand = Path(p)
    if cand.exists():
        return cand
    parts = p.replace("\\", "/").split("/")
    if "demo" in parts:
        return ROOT.joinpath(*parts[parts.index("demo"):])
    return cand


def _obstacle_map_for(policy, metrics: dict) -> tuple[dict | None, dict]:
    """The map the rail's Shield had, selected the way demo/follow_vlm.py
    selects it, or (None, why)."""
    from guardrail.models import AltitudeEnvelope, ObstacleClearance
    if not policy.by_type(ObstacleClearance):
        return None, {"used": False, "why": "policy has no obstacle_clearance rule"}
    citymap = (metrics.get("params") or {}).get("citymap")
    if not citymap:
        return None, {"used": False, "why": "obstacle_clearance rule present but the "
                      "run recorded no citymap: the rule was inert in flight"}
    path = _local_path(citymap)
    alts = policy.by_type(AltitudeEnvelope)
    sys.path.insert(0, str(ROOT / "demo"))
    if alts:
        import occ_bands
        sel = occ_bands.select_for_band(path.parent, alts[0].alt_min_m, alts[0].alt_max_m)
        if sel["map"] is None:
            return None, {"used": False, "why": "no band map covers the envelope",
                          "report": sel["report"]}
        return sel["map"], {"used": True, "dir": _rel(path.parent),
                            "selected_for_band_m": [alts[0].alt_min_m, alts[0].alt_max_m],
                            "report": sel["report"]}
    import city_planner
    m = city_planner.load_occ(str(path))
    if m is None:
        return None, {"used": False, "why": f"{_rel(path)} not found"}
    return m, {"used": True, "file": _rel(path), "sha256": sha256_file(path)}


_CLASS_WORDS = {"car": "car", "vehicle": "car", "truck": "car", "pedestrian": "pedestrian",
                "person": "pedestrian", "man": "pedestrian", "woman": "pedestrian"}


def _num(v):
    return v if isinstance(v, (int, float)) and math.isfinite(v) else None


def pack_flight(d: Path, max_ticks: int = DEFAULT_MAX_TICKS) -> dict:
    """One flight directory -> one self-contained pack entry."""
    from guardrail.bundle import policy_candidates
    from guardrail.manifest import normalize_topology, resolve_run_policy

    d = Path(d)
    log = d / "flight_log.jsonl"
    if not log.is_file():
        raise PackRefused(f"{d.name}: no flight_log.jsonl")
    try:
        manifest = json.loads((d / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PackRefused(f"{d.name}: manifest.json unreadable ({exc})") from exc
    metrics = {}
    if (d / "metrics.json").is_file():
        metrics = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
    policy, form, label = resolve_run_policy(manifest, policy_candidates())
    if policy is None:
        raise PackRefused(f"{d.name}: recorded policy_hash {manifest.get('policy_hash')!r} "
                          f"matches no policy on disk; refusing to pick the nearest")
    rows = [json.loads(line) for line in log.open(encoding="utf-8") if line.strip()]
    if not rows:
        raise PackRefused(f"{d.name}: flight_log.jsonl is empty")
    stride = max(1, math.ceil(len(rows) / max_ticks)) if max_ticks else 1
    picked = rows[::stride]

    declared = metrics.get("subject") if isinstance(metrics.get("subject"), dict) else None
    obj = str(metrics.get("object") or "").lower()
    word_class = next((c for w, c in _CLASS_WORDS.items() if w in obj.split()), None)
    action_field = "smooth" if any("smooth" in r for r in picked) else "raw"
    out_rows = []
    for r in picked:
        act = r.get(action_field) or r.get("raw") or {}
        if declared is not None:
            sx, sy, sc = declared.get("x"), declared.get("y"), declared.get("class")
        elif r.get("est_xy"):
            sx, sy = r["est_xy"][0], r["est_xy"][1]
            sc = ((r.get("truth") or {}).get("class")) or word_class
        else:
            sx = sy = sc = None
        psi = r.get("psi")
        yaw_deg = math.degrees(psi) if _num(psi) is not None else (_num(r.get("yaw_deg")) or 0.0)
        logged = sorted({v.get("rule_id") for v in (r.get("violations") or [])
                         if v.get("rule_id")})
        sh_ms = (r.get("ms") or {}).get("shield") if isinstance(r.get("ms"), dict) else None
        out_rows.append([
            _num(r.get("t")), round(float(r["x"]), 4), round(float(r["y"]), 4),
            round(float(r["up"]), 4), round(float(yaw_deg), 4),
            round(float(act.get("vx", 0.0)), 5), round(float(act.get("vy", 0.0)), 5),
            round(float(act.get("vz_up", 0.0)), 5), round(float(act.get("yaw_rate", 0.0)), 5),
            _num(sx), _num(sy), sc, logged, _num(sh_ms)])

    omap, omap_note = _obstacle_map_for(policy, metrics)
    dump = policy.model_dump(mode="json", exclude_none=True)
    from guardrail.models import Policy
    if Policy.model_validate(dump).policy_hash != policy.policy_hash:
        dump = policy.model_dump(mode="json")
        if Policy.model_validate(dump).policy_hash != policy.policy_hash:
            raise PackRefused(f"{d.name}: the resolved policy does not survive a "
                              f"JSON round trip with its hash intact")
    return {
        "tag": d.name,
        "source": _rel(log),
        "source_sha256": sha256_file(log),
        "rows_in_source": len(rows),
        "stride": stride,
        "manifest": manifest,
        # Today's name for the rail ("canonical-hil" was the retired name of
        # dev); the label the flight wrote is kept, never rewritten.
        "rail": normalize_topology(manifest.get("topology")),
        "rail_recorded": manifest.get("topology"),
        "policy_label": label,
        "policy_hash_recorded": manifest.get("policy_hash"),
        "policy_hash_form": form,
        "policy_hash": policy.policy_hash,
        "policy_sha256": policy_digest(dump),
        "policy": dump,
        "action_field": action_field,
        "subject_source": ("declared in metrics.json" if declared is not None
                           else "est_xy per row (the estimator position the rail "
                                "passed to Shield.set_subject)" if any(r[9] is not None for r in out_rows)
                           else "none"),
        "obstacle_map": _encode_map(omap) if omap is not None else None,
        "obstacle_map_note": omap_note,
        "columns": list(COLUMNS),
        "rows": out_rows,
    }


def build_pack(flights, demo_out: Path = DEFAULT_DEMO_OUT,
               max_ticks: int = DEFAULT_MAX_TICKS) -> dict:
    packed, skipped = [], []
    for name in flights:
        try:
            packed.append(pack_flight(Path(demo_out) / name, max_ticks))
        except PackRefused as exc:
            skipped.append({"tag": name, "why": str(exc)})
    return {"schema": PACK_SCHEMA,
            "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "code_revision": _code_revision(),
            "max_ticks_per_flight": max_ticks,
            "flights": packed, "skipped": skipped}


def policy_digest(dump: dict) -> str:
    """SHA-256 of the stored policy document itself. Deliberately independent
    of Policy.policy_hash: the pack must stay checkable even if the project
    changes how it fingerprints policies (it did once, on 2026-10-06)."""
    return hashlib.sha256(json.dumps(dump, sort_keys=True, separators=(",", ":"))
                          .encode("utf-8")).hexdigest()


def load_pack(path: Path) -> dict:
    """Read a pack and refuse it if any stored policy is not the document it
    was packed as (an edited pack is not the flight).

    A policy whose document is intact but whose Policy.policy_hash now differs
    from the packed value is loaded with a warning in `pack["warnings"]`: that
    means the fingerprint function changed, not the policy."""
    from guardrail.manifest import normalize_topology
    from guardrail.models import Policy
    pack = json.loads(Path(path).read_text(encoding="utf-8"))
    if pack.get("schema") != PACK_SCHEMA:
        raise ValueError(f"{path}: schema {pack.get('schema')!r}, expected {PACK_SCHEMA!r}")
    if not pack.get("flights"):
        raise ValueError(f"{path}: the pack holds no flights")
    pack["warnings"] = []
    for f in pack["flights"]:
        if list(f.get("columns") or []) != list(COLUMNS):
            raise ValueError(f"{path}: flight {f['tag']}: unexpected columns")
        if normalize_topology(f.get("rail")) != f.get("rail"):
            raise ValueError(f"{path}: flight {f['tag']}: rail {f.get('rail')!r} is a "
                             f"retired label; re-pack (it reads as "
                             f"{normalize_topology(f.get('rail'))!r})")
        want = f.get("policy_sha256")
        got = policy_digest(f["policy"])
        if want is None or got != want:
            raise ValueError(f"{path}: flight {f['tag']}: stored policy hashes to "
                             f"{got[:16]}, packed as {str(want)[:16]}: the pack was edited")
        pol = Policy.model_validate(f["policy"])
        if pol.policy_hash != f["policy_hash"]:
            pack["warnings"].append(
                f"{f['tag']}: Policy.policy_hash is now {pol.policy_hash}, packed as "
                f"{f['policy_hash']}; the document is unchanged, so the fingerprint "
                f"function changed")
        f["_policy"] = pol
    return pack


# --------------------------------------------------------------------------- #
# Replay
# --------------------------------------------------------------------------- #
def replay(flight: dict, policy, horizon: tuple[float, float], obstacle_map=None,
           clock=time.perf_counter, compare_logged: bool = False) -> dict:
    """Run every packed tick through a fresh Shield, timing _check and filter.

    With `compare_logged`, the replayed violated-rule set is compared with the
    one the rail logged, and the agreement is split: ticks whose log names a
    violation (where agreement shows the replay reproduces the rule), and
    ticks whose log is empty (where a replay that never flags anything agrees
    too - the null)."""
    from guardrail import shield as shield_mod
    from guardrail.models import Action4D, State

    lookahead, dt = horizon
    # Looked up at call time, so a test can swap in a broken Shield.
    shield = shield_mod.Shield(policy, lookahead_s=lookahead, dt=dt,
                               obstacle_map=obstacle_map)
    check_ms, filter_ms = [], []
    geofence_ticks = braked = escaped = 0
    agree = compared = nonempty = agree_nonempty = flagged_on_empty = 0
    for row in flight["rows"]:
        (_t, x, y, up, yaw, vx, vy, vz, yr, sx, sy, sc, logged, _sh) = row
        state = State(x=x, y=y, up=up, yaw_deg=yaw)
        act = Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yr)
        if sx is None or sy is None:
            shield.set_subject(None)
        else:
            shield.set_subject(sx, sy, sc)
        t0 = clock()
        shield._check(state, act)
        t1 = clock()
        dec = shield.filter(state, act)
        t2 = clock()
        check_ms.append((t1 - t0) * 1e3)
        filter_ms.append((t2 - t1) * 1e3)
        if any(v.category == "geofence" for v in dec.violations):
            geofence_ticks += 1
        braked += bool(dec.braked)
        escaped += bool(dec.emitted_violations)
        if compare_logged:
            compared += 1
            got = sorted({v.rule_id for v in dec.violations})
            same = got == list(logged)
            agree += same
            if logged:
                nonempty += 1
                agree_nonempty += same
            elif got:
                flagged_on_empty += 1
    agreement = None
    if compare_logged:
        agreement = {
            "compared": compared,
            "identical_rule_sets": agree,
            # The informative part: ticks whose log names at least one rule.
            "logged_nonempty": nonempty,
            "identical_nonempty": agree_nonempty,
            # The null: a replay that never flags anything matches exactly the
            # ticks whose log is empty, and nothing else.
            "never_flagging_null": compared - nonempty,
            "replay_flagged_where_log_was_empty": flagged_on_empty,
            "informative": nonempty > 0,
        }
    return {"check_ms": check_ms, "filter_ms": filter_ms,
            "ticks": len(flight["rows"]), "geofence_ticks": geofence_ticks,
            "braked_ticks": braked, "escaped_ticks": escaped,
            "logged_agreement": agreement}


def _rail_logged(flight: dict) -> dict:
    ts = [r[0] for r in flight["rows"] if r[0] is not None]
    out = {"note": "measured on the rail that flew, on its host and under its load"}
    if flight.get("stride", 1) == 1 and len(ts) > 2:
        # The period of a 10 Hz loop is ~100 ms by design, so it is not held
        # against the 100 ms budget (a 100.4 ms period is jitter, not an
        # overrun). What matters is a tick that arrived far too late.
        periods = [(b - a) * 1e3 for a, b in zip(ts, ts[1:])]
        out["tick_period"] = stats(periods)
        out["tick_period"]["late_ticks_over_150ms"] = sum(1 for p in periods if p > 150.0)
    elif len(ts) > 2:
        out["tick_period"] = None
        out["tick_period_why"] = (f"rows were packed every {flight['stride']}th tick, "
                                  f"so consecutive packed rows are not one tick apart")
    sh = [r[13] for r in flight["rows"] if r[13] is not None]
    out["shield_ms"] = stats(sh, TICK_BUDGET_MS) if sh else None
    return out


def profile(pack: dict, topology: str, horizons=("flown", "grant"), arms=ARMS,
            clock=time.perf_counter, warmup: int = 10, keep_gc: bool = True,
            facts: dict | None = None, command: str | None = None,
            calibrate_host: bool = True) -> dict:
    """Replay every packed flight under every arm and horizon; budgets,
    sanity checks, the host-speed reference and the headline verdict."""
    facts = facts if facts is not None else host_facts()
    cal_before = calibrate() if calibrate_host else None
    flights_out, failures = [], []
    pooled: dict[tuple[str, str], dict[str, list[float]]] = {}
    for f in pack["flights"]:
        pol = f["_policy"]
        omap = decode_map(f.get("obstacle_map"))
        xy = [(r[1], r[2]) for r in f["rows"]]
        built = {"as_flown": (pol, 0)}
        if "design_load" in arms:
            built["design_load"] = design_load_policy(pol, xy)
        if "floor" in arms:
            built["floor"] = (floor_policy(pol), 0)
        entry = {"tag": f["tag"], "rail": f.get("rail"),
                 "rail_recorded": f.get("rail_recorded"), "ticks": len(f["rows"]),
                 "stride": f.get("stride"), "source_sha256": f.get("source_sha256"),
                 "policy_hash": f["policy_hash"], "policy_label": f.get("policy_label"),
                 "rules": {a: len(built[a][0].constraints) for a in arms},
                 "design_load_zones_added": built.get("design_load", (None, None))[1],
                 "obstacle_map": bool(omap), "arms": {},
                 "rail_logged": _rail_logged(f)}
        for arm in arms:
            arm_pol = built[arm][0]
            entry["arms"][arm] = {}
            for hz in horizons:
                if warmup:
                    replay({"rows": f["rows"][:warmup]}, arm_pol, HORIZONS[hz], omap)
                if not keep_gc:
                    gc.disable()
                try:
                    res = replay(f, arm_pol, HORIZONS[hz], omap, clock=clock,
                                 compare_logged=(arm == "as_flown" and hz == "flown"))
                finally:
                    if not keep_gc:
                        gc.enable()
                cell = {"check": stats(res["check_ms"], QUERY_BUDGET_MS),
                        "filter": stats(res["filter_ms"], TICK_BUDGET_MS),
                        "geofence_ticks": res["geofence_ticks"],
                        "braked_ticks": res["braked_ticks"],
                        "escaped_ticks": res["escaped_ticks"]}
                if res["logged_agreement"] is not None:
                    cell["logged_agreement"] = res["logged_agreement"]
                    if not res["logged_agreement"]["informative"]:
                        entry["agreement_note"] = (
                            "this flight logged no violation on any packed tick, so "
                            "agreement verifies nothing beyond the empty sets: not its "
                            "obstacle map, not its subject, not its fences")
                entry["arms"][arm][hz] = cell
                pool = pooled.setdefault((arm, hz), {"check": [], "filter": []})
                pool["check"] += res["check_ms"]
                pool["filter"] += res["filter_ms"]
                if not res["check_ms"]:
                    failures.append(f"{f['tag']}/{arm}/{hz}: no tick was timed")
                if arm == "design_load" and res["geofence_ticks"] == 0:
                    failures.append(
                        f"{f['tag']}/design_load/{hz}: no tick raised a geofence "
                        f"violation, so its time says nothing about fence cost")
                if arm == "floor" and res["geofence_ticks"] != 0:
                    failures.append(f"{f['tag']}/floor/{hz}: {res['geofence_ticks']} "
                                    f"geofence violations with every fence removed")
        if "design_load" in arms:
            n_dl = entry["rules"]["design_load"]
            if entry["design_load_zones_added"] and n_dl != DESIGN_LOAD_RULES:
                failures.append(f"{f['tag']}: design_load has {n_dl} rules, "
                                f"not {DESIGN_LOAD_RULES}")
        flights_out.append(entry)

    pooled_out = {f"{a}/{h}": {"check": stats(v["check"], QUERY_BUDGET_MS),
                               "filter": stats(v["filter"], TICK_BUDGET_MS)}
                  for (a, h), v in pooled.items()}
    host_speed = host_speed_record(cal_before, calibrate()) if cal_before is not None else None
    if host_speed is not None and not host_speed["steady"]:
        failures.append(
            f"the host-speed reference moved from {host_speed['before']['median_ms']:.1f} "
            f"to {host_speed['after']['median_ms']:.1f} ms during the run (more than "
            f"{host_speed['tolerance']:g}x): the timings mix two host speeds, so the "
            f"verdict is undetermined. Re-run pinned to one logical CPU per core, with "
            f"nothing else running")
    agreement = None
    cells = [fl["arms"].get("as_flown", {}).get("flown", {}).get("logged_agreement")
             for fl in flights_out]
    cells = [c for c in cells if c]
    if cells:
        agreement = {k: sum(c[k] for c in cells)
                     for k in ("compared", "identical_rule_sets", "logged_nonempty",
                               "identical_nonempty", "never_flagging_null",
                               "replay_flagged_where_log_was_empty")}
        agreement["flights_with_no_logged_violation"] = [
            fl["tag"] for fl in flights_out
            if not ((fl["arms"].get("as_flown", {}).get("flown", {})
                     .get("logged_agreement") or {}).get("informative", True))]
    head_key = f"{HEADLINE[0]}/{HEADLINE[1]}"
    headline = None
    if head_key in pooled_out:
        h = pooled_out[head_key]
        headline = {
            "condition": (f"{DESIGN_LOAD_RULES} active rules, {HORIZONS['grant'][0]:g} s "
                          f"horizon at {HORIZONS['grant'][1]:g} s "
                          f"({int(round(HORIZONS['grant'][0] / HORIZONS['grant'][1])) + 1} poses "
                          f"including t = 0), real flight states"),
            "verdict_basis": VERDICT_BASIS,
            "check_p99_ms": h["check"].get("p99_ms"),
            "check_max_ms": h["check"].get("max_ms"),
            "check_over_budget": h["check"].get("over_budget"),
            "check_budget_ms": QUERY_BUDGET_MS,
            "check_verdict": h["check"].get("verdict"),
            "filter_p99_ms": h["filter"].get("p99_ms"),
            "filter_max_ms": h["filter"].get("max_ms"),
            "filter_over_budget": h["filter"].get("over_budget"),
            "filter_budget_ms": TICK_BUDGET_MS,
            "filter_verdict": h["filter"].get("verdict"),
            "ticks": h["check"].get("n"),
            "host_steady": None if host_speed is None else host_speed["steady"],
        }
    return {
        "schema": PROFILE_SCHEMA,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "topology": topology,
        "command": command,
        "code_revision": _code_revision(),
        "host": facts,
        "pack": {"path": pack.get("_path"), "sha256": pack.get("_sha256"),
                 "created_utc": pack.get("created_utc"),
                 "code_revision_at_pack": pack.get("code_revision")},
        "budgets_ms": {"check": QUERY_BUDGET_MS, "filter": TICK_BUDGET_MS},
        "horizons": {h: {"lookahead_s": HORIZONS[h][0], "dt_s": HORIZONS[h][1]}
                     for h in horizons},
        "gc_enabled": keep_gc,
        "host_speed": host_speed,
        "flights": flights_out,
        "pooled": pooled_out,
        "logged_agreement": agreement,
        "headline": headline,
        "failures": failures,
        "not_measured": ("ROS 2 / DDS / MAVROS / MAVLink hops of the end-to-end tick; "
                         "see rail_logged per flight for what the rails recorded"),
    }


def attach_summary(prof: dict, path: Path | None = None) -> dict:
    """The compact record a run's metrics.json carries beside its manifest.

    The manifest stays the grant's six fields; evidence goes beside it, as
    hil_evidence and policy_source already do.

    `within_budget` is None (undetermined) when the host changed speed during
    the run or recorded no host-speed reference: such timings mix two host
    speeds, and neither yes nor no would be true. Otherwise it is False when
    the run failed any of its own checks, else both p99 verdicts."""
    head = prof.get("headline") or {}
    hs = prof.get("host_speed") or {}
    if not hs or not hs.get("steady"):
        within = None
    elif prof.get("failures"):
        within = False
    else:
        within = (head.get("check_verdict") == "within"
                  and head.get("filter_verdict") == "within")
    out = {
        "schema": prof.get("schema"),
        "topology": prof.get("topology"),
        "host_is_orin": bool(((prof.get("host") or {}).get("jetson") or {}).get("is_orin")),
        "host_machine": (prof.get("host") or {}).get("machine"),
        "code_revision": prof.get("code_revision"),
        "check_p99_ms": head.get("check_p99_ms"),
        "check_max_ms": head.get("check_max_ms"),
        "check_budget_ms": head.get("check_budget_ms"),
        "filter_p99_ms": head.get("filter_p99_ms"),
        "filter_max_ms": head.get("filter_max_ms"),
        "filter_budget_ms": head.get("filter_budget_ms"),
        "verdict_basis": head.get("verdict_basis"),
        "host_steady": hs.get("steady") if hs else None,
        "within_budget": within,
        "failures": len(prof.get("failures") or []),
    }
    if path is not None:
        out["path"] = _rel(Path(path))
        out["sha256"] = sha256_file(Path(path))
    return out


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _print_profile(prof: dict) -> None:
    print(f"topology {prof['topology']}  host {prof['host']['machine']} "
          f"{prof['host'].get('cpu_model')}  orin={prof['host']['jetson']['is_orin']}  "
          f"python {prof['host']['python']}")
    hdr = f"{'arm/horizon':<20}{'call':<8}{'n':>7}{'median':>10}{'p99':>10}{'max':>10}  budget"
    print(hdr)
    print("-" * len(hdr))
    for key, cell in prof["pooled"].items():
        for call in ("check", "filter"):
            s = cell[call]
            if not s.get("n"):
                continue
            print(f"{key:<20}{call:<8}{s['n']:>7}{s['median_ms']:>9.3f}m{s['p99_ms']:>9.3f}m"
                  f"{s['max_ms']:>9.3f}m  {s['budget_ms']:g} ms p99 {s['verdict']}"
                  f" ({s['over_budget']} ticks over)")
    for f in prof["flights"]:
        dl = f["arms"].get("design_load", {})
        geo = {h: c["geofence_ticks"] for h, c in dl.items()}
        ag = (f["arms"].get("as_flown", {}).get("flown", {}) or {}).get("logged_agreement")
        agree = (f"{ag['identical_nonempty']}/{ag['logged_nonempty']} ticks with a logged "
                 f"violation (+{ag['never_flagging_null']} empty, which a never-flagging "
                 f"replay also matches)") if ag else "-"
        print(f"flight {f['tag']:<24} ticks {f['ticks']:>5}  rules {f['rules']}  "
              f"design-load geofence ticks {geo}  replay = logged rule set on {agree}")
        if f.get("agreement_note"):
            print(f"       note: {f['agreement_note']}")
    hs = prof.get("host_speed")
    if hs:
        print(f"host-speed reference: {hs['before']['median_ms']:.1f} ms before, "
              f"{hs['after']['median_ms']:.1f} ms after "
              f"({'steady' if hs['steady'] else 'CHANGED during the run'})")
    h = prof.get("headline")
    if h:
        print(f"\nHEADLINE ({h['condition']}): _check p99 {h['check_p99_ms']:.3f} ms "
              f"(max {h['check_max_ms']:.3f}, {h['check_over_budget']} of {h['ticks']} ticks "
              f"over) vs {h['check_budget_ms']:g} ms -> {h['check_verdict']}; filter p99 "
              f"{h['filter_p99_ms']:.3f} ms (max {h['filter_max_ms']:.3f}) vs "
              f"{h['filter_budget_ms']:g} ms -> {h['filter_verdict']}")
        print(f"verdict basis: {h['verdict_basis']}")
    for msg in prof.get("failures") or []:
        print(f"FAIL  {msg}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_pack = sub.add_parser("pack", help="extract a portable tick pack from demo/out flights")
    p_pack.add_argument("--flights", nargs="+", default=list(DEFAULT_FLIGHTS))
    p_pack.add_argument("--demo-out", type=Path, default=DEFAULT_DEMO_OUT)
    p_pack.add_argument("--max-ticks", type=int, default=DEFAULT_MAX_TICKS,
                        help="per flight; longer flights are sampled evenly (0 = all)")
    p_pack.add_argument("--out", type=Path, default=DEFAULT_PACK)

    p_prof = sub.add_parser("profile", help="replay the pack through the Shield, time it")
    p_prof.add_argument("--pack", type=Path, default=DEFAULT_PACK)
    p_prof.add_argument("--topology", default=os.environ.get("VLAGUARD_TOPOLOGY"),
                        help="dev | hil | flight (default: $VLAGUARD_TOPOLOGY)")
    p_prof.add_argument("--horizons", nargs="+", default=list(HORIZONS), choices=list(HORIZONS))
    p_prof.add_argument("--no-gc", action="store_true",
                        help="disable the garbage collector while timing (default: on, "
                             "as in a running node)")
    p_prof.add_argument("--cpus", default=None,
                        help="pin to these logical CPUs, one per physical core, e.g. "
                             "0,2,4,6,8,10,12,14 (recorded in the result)")
    p_prof.add_argument("--note", default=None,
                        help="free text stored with the result, e.g. 'shared desktop, "
                             "other agents running'")
    p_prof.add_argument("--out", type=Path, default=None,
                        help="default: <evidence>/<topology>/shield_tick_profile.json, "
                             "<evidence> = $VLAGUARD_EVIDENCE_OUT or deploy/evidence")

    sub.add_parser("facts", help="print this host's facts as JSON")
    p_att = sub.add_parser("attach", help="print the summary a run's metrics.json carries")
    p_att.add_argument("profile_json", type=Path)

    args = ap.parse_args(argv)
    cmd_line = "python tools/profile_shield_tick.py " + shlex.join(
        argv if argv is not None else sys.argv[1:])

    if args.cmd == "facts":
        print(json.dumps(host_facts(), indent=2))
        return 0
    if args.cmd == "attach":
        prof = json.loads(args.profile_json.read_text(encoding="utf-8"))
        print(json.dumps(attach_summary(prof, args.profile_json), indent=2))
        return 0
    if args.cmd == "pack":
        pack = build_pack(args.flights, args.demo_out, args.max_ticks)
        pack["command"] = cmd_line.strip()
        for s in pack["skipped"]:
            print(f"SKIP  {s['tag']}: {s['why']}")
        if not pack["flights"]:
            print("FAIL  no flight could be packed")
            return 1
        args.out.parent.mkdir(parents=True, exist_ok=True)
        write_lf(args.out, json.dumps(pack, separators=(",", ":")) + "\n")
        for f in pack["flights"]:
            print(f"packed {f['tag']:<24} {len(f['rows']):>5} ticks (every {f['stride']}) "
                  f"policy {f['policy_label']} map={'yes' if f['obstacle_map'] else 'no'} "
                  f"subject: {f['subject_source']}")
        print(f"wrote {_rel(args.out)} ({args.out.stat().st_size // 1024} KB)")
        return 0

    # profile
    facts = host_facts()
    refused = check_topology_claim(args.topology, facts)
    if refused:
        for r in refused:
            print(f"REFUSED  {r}")
        return 2
    try:
        pack = load_pack(args.pack)
    except (OSError, ValueError) as exc:
        print(f"REFUSED  {exc}")
        return 2
    pack["_path"] = _rel(args.pack)
    pack["_sha256"] = sha256_file(args.pack)
    pinned = None
    if args.cpus:
        try:
            pinned = pin_cpus(parse_cpus(args.cpus))
        except (RuntimeError, ValueError, OSError) as exc:
            print(f"REFUSED  --cpus {args.cpus}: {exc}")
            return 2
    prof = profile(pack, args.topology, horizons=tuple(args.horizons),
                   keep_gc=not args.no_gc, facts=facts, command=cmd_line.strip())
    prof["cpu_affinity"] = pinned
    prof["note"] = args.note
    out = args.out or (evidence_root() / args.topology / "shield_tick_profile.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    write_lf(out, json.dumps(prof, indent=1) + "\n")
    _print_profile(prof)
    print(f"\nwrote {_rel(out)}")
    print(f"reproduce: {cmd_line.strip()}")
    return 1 if prof["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
