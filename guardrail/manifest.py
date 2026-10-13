"""The six-field determinism manifest, and whether a run may be called KPI-grade.

WHY THIS EXISTS

The grant's WP4 spec is unambiguous about where contractual numbers may come from:

    "The Stress Testing harness is the only way the project produces the
     contractual KPI numbers"

    "every episode emits a six-field manifest (code_revision, vla_model_hash,
     policy_hash, random_seed, sim_speedup, topology); sim_speedup=1.0 is
     mandatory for any KPI-bearing run"

Our flights emitted none of those six. `metrics.json` carried six controller gains
and the policy hash was computed, printed, and then thrown away. So on the grant's
own terms not one number this repo has produced was reportable.

It is also the cheapest available fix for a failure class that has bitten
repeatedly, each time because an artefact looked valid and was not:

  * the takeoff heading was never commanded, and swung the traffic hit rate
    0.740 -> 0.331 on IDENTICAL configuration. Nothing in the output said which
    run had which heading;
  * a detector stall to 0.52 Hz produced `det_hit_rate 1.000` on a flight that
    actually tracked for 13.6% of ticks;
  * two A/B claims were made from one flight per arm before the spread was known.

HONESTY RULE FOR THIS MODULE

Every field either resolves to something real or records the string
`"unresolved"`. A plausible-looking placeholder is worse than an absent value,
because the whole point of the manifest is that a number can be traced. Nothing
here may guess.
"""
from __future__ import annotations

import hashlib
import ipaddress
import json
import os
import platform
import re
import socket
import subprocess
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlparse

UNRESOLVED = "unresolved"

# THE GRANT'S THREE TOPOLOGIES, BY THE GRANT'S NAMES (Architecture constraints;
# reference vlaguard_common/manifest.py `Topology`: dev | hil | flight):
#
#   dev     one desktop: simulator + ArduPilot SITL + MAVROS 2 + VLA + Shield.
#           "Not used for reported KPI numbers" (reference
#           docs/03-simulation/topologies.md).
#   hil     the VLA and the Shield on a Jetson Orin, talking to the simulator
#           host over the network - the grant's "canonical KPI configuration".
#   flight  the Orin on a real ArduPilot airframe.
#
# Until 2026-10-06 this file called our desktop SITL + MAVROS 2 rail
# "canonical-hil" and is_kpi_grade() treated it as the KPI topology. It is the
# grant's `dev`: one PC, no Orin. Five runs carry the old label; they are read
# as `dev` (LEGACY_TOPOLOGY_LABELS), never rewritten.
#
# Since the PI's 2026-10-06 decision a Jetson Orin is the `hil` machine, and
# the same Shield node is meant to move onto the real drone (`flight`) with no
# code change. So `hil` and `flight` are no longer refused outright: they are
# accepted on EVIDENCE the node gathers from the machine it runs on and the
# link it flies over (`collect_host_evidence`, `network_link_evidence`,
# `check_topology_evidence`), and `detect_topology` picks the label from that
# evidence, never from a command-line flag.
TOPOLOGY_DEV = "dev"
TOPOLOGY_HIL = "hil"
TOPOLOGY_FLIGHT = "flight"
# The old constant name, kept so importers do not break. It now MEANS dev, and
# a manifest built with it says "dev".
TOPOLOGY_CANONICAL_HIL = TOPOLOGY_DEV
LEGACY_TOPOLOGY_LABELS = {"canonical-hil": TOPOLOGY_DEV}

# Our functional rails, which are none of the three.
TOPOLOGY_PROJECTAIRSIM = "projectairsim-single-host"
# ArduPilot SITL driven straight over pymavlink (sitl/run_sitl_demo.py). Real
# flight code and a real MAVLink path, but not even the grant's dev topology,
# which runs MAVROS 2. Naming it separately keeps the difference from being
# blurred in either direction.
TOPOLOGY_ARDUPILOT_SITL = "ardupilot-sitl-pymavlink"

# The PI's written decision that `dev` runs may carry contractual KPI figures
# (question PQ1 in the 2026-10-06 plan), or None while there is none. When the
# answer arrives, set this to a citation ("PI email 2026-10-xx: ...") and
# is_kpi_grade() will accept evidence-backed dev runs, naming the waiver in the
# reasons. Until then a dev run is never KPI-grade: the grant reserves KPI
# figures for hil, and a constant that says otherwise is the drift this replaced.
DEV_KPI_WAIVER: str | None = None


def normalize_topology(label: str | None) -> str | None:
    """Read a stored topology label under today's names (old manifests too)."""
    return LEGACY_TOPOLOGY_LABELS.get(label, label) if label else label

MIN_DET_HZ = 2.0
MAX_HEADING_ERR_DEG = 10.0


# A repository only counts as ours if it tracks this file. Resolving to SOME
# revision is not enough: the field has to name a commit a reader could check
# out to reproduce the run.
CODE_SENTINEL = "guardrail/shield.py"


def _tracks_our_code(cand: Path) -> bool:
    """Does this repository actually contain the Shield?"""
    try:
        r = subprocess.run(
            ["git", "-C", str(cand), "ls-files", "--error-unmatch", CODE_SENTINEL],
            capture_output=True, text=True, timeout=15)
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def code_revision(repo: str | Path | None = None) -> str:
    """Git revision of the repository holding the code that runs, with a dirty flag.

    Two rules, both learned from getting this wrong.

    FIRST, the working directory wins. It used to be searched AFTER
    `kuanting-vla-uav-guardrail`, so on a machine where only the fork was a git
    repo this returned the fork's HEAD - `fe0dc0ffd3a6-dirty` - even though the
    fork does not contain `guardrail/shield.py`. Every determinism manifest
    written before 2026-08-20 names a commit that cannot reproduce the flight it
    describes.

    SECOND, a candidate must TRACK the code. Order alone would not have caught
    it: the failure was not "the wrong repo was checked first", it was "a repo
    that resolves was accepted without asking whether it holds the code". A
    reader who checks out the reported revision must find the Shield there.

    Returns "unversioned" when nothing qualifies, which is a fact rather than a
    placeholder, and which `is_kpi_grade()` correctly refuses.
    """
    here = Path(__file__).resolve().parents[1]
    for cand in ([Path(repo)] if repo else []) + [
            here,
            here / "kuanting-vla-uav-guardrail"]:
        try:
            head = subprocess.run(["git", "-C", str(cand), "rev-parse", "HEAD"],
                                  capture_output=True, text=True, timeout=15)
            if head.returncode != 0:
                continue
            if not _tracks_our_code(cand):
                continue
            rev = head.stdout.strip()[:12]
            st = subprocess.run(["git", "-C", str(cand), "status", "--porcelain"],
                                capture_output=True, text=True, timeout=20)
            dirty = bool(st.stdout.strip()) if st.returncode == 0 else None
            suffix = "-dirty" if dirty else ("" if dirty is False else "-unknown")
            return f"{rev}{suffix}"
        except (OSError, subprocess.SubprocessError):
            continue
    return "unversioned"


def hf_snapshot_revision(model_id: str) -> str | None:
    """The commit the HuggingFace cache actually holds for this model id.

    Far better than hashing file stats: it is the upstream revision, so two
    machines that resolve the same value really did run the same weights. A bare
    model id pins nothing, which is why an id alone is never accepted as the hash.
    """
    root = os.environ.get("HF_HOME")
    bases = [Path(root) / "hub"] if root else []
    bases.append(Path.home() / ".cache" / "huggingface" / "hub")
    folder = "models--" + model_id.replace("/", "--")
    for b in bases:
        snaps = b / folder / "snapshots"
        if not snaps.is_dir():
            continue
        revs = sorted((d.name for d in snaps.iterdir() if d.is_dir()))
        if len(revs) == 1:
            return revs[0]
        if revs:
            # More than one snapshot cached means the id is genuinely ambiguous
            # on this machine, and saying so is the honest answer.
            return "ambiguous:" + ",".join(r[:8] for r in revs)
    return None


# The pilot of a run with no VLA at all - the hand-written servo controllers
# of the follow demos. A fact, not a placeholder: `model_hash(NO_VLA)` returns
# it as resolved, so such a run stops filing its DETECTOR under
# vla_model_hash (audit card X-16) without being refused for "unresolved".
NO_VLA = "none:hand-written-controller"

# Where full weight digests are cached. Hashing the bytes of a 15 GB checkpoint
# takes tens of seconds; a flight should pay that once per file version, not
# once per launch. A central file rather than one beside the weights: a
# HuggingFace cache or a read-only model mount must not be written into by a
# flight script. The key is the file's VERSION, not only its name, size and
# date (see `_file_version_key`).
DIGEST_CACHE_ENV = "GUARDRAIL_DIGEST_CACHE"
# Bytes read from each end of a file for the cache key's content sample.
_KEY_SAMPLE_BYTES = 1 << 16


def _digest_cache_path() -> Path:
    env = os.environ.get(DIGEST_CACHE_ENV)
    if env:
        return Path(env)
    return Path.home() / ".cache" / "guardrail" / "weight_sha256.json"


def _load_digest_cache(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def _file_version_key(p: Path, st: os.stat_result) -> str:
    """The cache key for one version of one file.

    Path + size + mtime_ns alone (the 2026-10-06 key) served the old digest to
    a different checkpoint copied over the same path with its mtime preserved
    (`cp -p`, `rsync -a`, LoRA adapters of the same shape): identity by
    metadata again, the defect X-16 removed from the hash itself. So the key
    also carries
      * the inode / NTFS file index: a file REPLACED by a copy is a new file,
        whatever date it was given;
      * on POSIX, ctime: user space cannot set it, and every rewrite in place
        changes it (Windows has no change time in os.stat; there st_ctime is
        the creation time, which NTFS may even carry over to a replacement);
      * a SHA-256 of the first and last 64 KiB: a rewrite in place that
        restores the old mtime still changes the header (safetensors' JSON
        header) or the tail of a real checkpoint.
    """
    parts = [str(p), str(st.st_size), str(st.st_mtime_ns)]
    if getattr(st, "st_ino", 0):
        parts.append(f"ino{st.st_ino}")
    if os.name == "posix":
        parts.append(f"ctime{st.st_ctime_ns}")
    h = hashlib.sha256()
    with open(p, "rb") as fh:
        h.update(fh.read(_KEY_SAMPLE_BYTES))
        if st.st_size > 2 * _KEY_SAMPLE_BYTES:
            fh.seek(st.st_size - _KEY_SAMPLE_BYTES)
            h.update(fh.read(_KEY_SAMPLE_BYTES))
    parts.append(f"ends{h.hexdigest()[:16]}")
    return "|".join(parts)


def file_sha256(path: str | Path, cache: dict | None = None) -> str:
    """Full SHA-256 of a file's BYTES (64 hex), served from `cache` only when
    the same version of the file was hashed before (`_file_version_key`)."""
    p = Path(path).resolve()
    st = p.stat()
    key = _file_version_key(p, st)
    if cache is not None and isinstance(cache.get(key), str) \
            and len(cache[key]) == 64:
        return cache[key]
    h = hashlib.sha256()
    with open(p, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    digest = h.hexdigest()
    if cache is not None:
        cache[key] = digest
    return digest


def weights_digest(weight_paths: list[str | Path],
                   cache_path: str | Path | None = None) -> str | None:
    """One SHA-256 over every weight file's own SHA-256, or None if no file.

    The grant's manifest field is "vla_model_hash: <SHA-256 of model weights>"
    (reference data-flow.md). Until 2026-10-06 this module hashed each file's
    NAME, SIZE and MTIME instead of its bytes and cut the result to 16 hex, so
    two different checkpoints of the same size written in the same second
    shared a hash, and a byte-identical copy did not (audit card X-16).

    Each file is listed by its path RELATIVE to the directory it was found in
    plus its full digest, sorted, so the result does not depend on where the
    weights live on disk. Logs and JSONL files are skipped, as before: they
    are written beside adapters during training and are not weights.
    """
    cpath = Path(cache_path) if cache_path else _digest_cache_path()
    cache = _load_digest_cache(cpath)
    before = dict(cache)
    lines: list[str] = []
    for p in weight_paths:
        p = Path(p)
        if p.is_dir():
            files = [(f.relative_to(p).as_posix(), f) for f in sorted(p.rglob("*"))]
        elif p.is_file():
            files = [(p.name, p)]
        else:
            files = []
        for rel, f in files:
            if not f.is_file() or f.suffix in (".log", ".jsonl"):
                continue
            try:
                lines.append(f"{rel}  {file_sha256(f, cache)}")
            except OSError:
                continue
    if not lines:
        return None
    if cache != before:
        try:
            cpath.parent.mkdir(parents=True, exist_ok=True)
            tmp = cpath.with_suffix(".tmp")
            tmp.write_text(json.dumps(cache, indent=0, sort_keys=True),
                           encoding="utf-8")
            os.replace(tmp, cpath)
        except OSError:
            pass        # a cache that cannot be written only costs time
    return hashlib.sha256("\n".join(sorted(lines)).encode()).hexdigest()


def model_hash(model_id: str, weight_paths: list[str | Path] | None = None) -> str:
    """Identify the model that actually ran.

    Hashes the weight FILES' BYTES when they are on disk (full SHA-256, cached
    per file version - see `weights_digest`), because a HuggingFace id alone
    does not pin a revision and an adapter directory can be edited in place.
    Falls back to the id plus "unresolved" rather than pretending the id is a
    hash.
    """
    if model_id == NO_VLA:
        return NO_VLA
    rev = hf_snapshot_revision(model_id)
    if rev and not weight_paths:
        return f"{model_id}@{rev[:16]}"

    digest = weights_digest(list(weight_paths)) if weight_paths else None
    if digest is None:
        # A code-only pilot has no weights and no HuggingFace revision, but it is
        # not unresolved: it is fully determined by its source. Hashing that file
        # pins it exactly, where "unresolved" would wrongly suggest the run
        # cannot be reproduced. Applies to ids like
        # "guardrail.vla_stub.StubVLA", which the SITL rails use. Full 64 hex
        # since 2026-10-06 (16 before).
        src = _source_of(model_id)
        if src is not None:
            return f"{model_id}@src:{hashlib.sha256(src).hexdigest()}"
        return f"{model_id}@{UNRESOLVED}"
    if rev:
        # Pin the base model as well as the adapter weights hashed here.
        digest = hashlib.sha256(f"{rev}\n{digest}".encode()).hexdigest()
    return f"{model_id}@sha256:{digest}"


def pilot_record(model_id: str, kind: str,
                 weight_paths: list[str | Path] | None = None,
                 **extra) -> dict[str, Any]:
    """What flew the aircraft, beside the six-field manifest (audit card X-16).

    `kind` is one of "vla" (a learned model in the VLA slot), "stub" (a
    code-only stand-in such as guardrail.vla_stub.StubVLA) or "none" (no VLA:
    a hand-written controller, model_id NO_VLA). The manifest keeps exactly
    the grant's six fields; this record says in words what its
    `vla_model_hash` is a hash OF, and carries anything else that steered the
    run (a detector, a tokenizer) under its own key, so a detector is never
    filed as the VLA again.
    """
    if kind not in ("vla", "stub", "none"):
        raise ValueError(f"pilot kind {kind!r} is not one of vla / stub / none")
    if kind == "none" and model_id != NO_VLA:
        raise ValueError(f"a run with no VLA records model_id {NO_VLA!r}, "
                         f"not {model_id!r}")
    rec = {"kind": kind, "model_id": model_id,
           "hash": model_hash(model_id, weight_paths)}
    rec.update(extra)
    return rec


def _source_of(model_id: str) -> bytes | None:
    """Source bytes of a dotted `package.module.Symbol` id inside this repo."""
    parts = model_id.split(".")
    root = Path(__file__).resolve().parents[1]
    # Drop trailing symbol names until a module file is found, so both
    # "pkg.mod.Class" and "pkg.mod" resolve.
    for cut in range(len(parts), 0, -1):
        cand = root.joinpath(*parts[:cut]).with_suffix(".py")
        if cand.is_file():
            try:
                return cand.read_bytes()
            except OSError:
                return None
    return None


def sim_speedup_from_scene(scene_path: str | Path) -> float | None:
    """DERIVE the speed-up from the scene clock rather than asserting it.

    `sim_speedup=1.0` is mandatory for a KPI-bearing run, so a hardcoded 1.0
    would defeat the requirement entirely. The scene declares a steppable clock
    with `step-ns` and `real-time-update-rate`; equal means real time. None means
    it could not be read, which `is_kpi_grade` treats as a failure.
    """
    try:
        raw = Path(scene_path).read_text(encoding="utf-8")
    except OSError:
        return None
    # jsonc: strip // comments before parsing
    stripped = re.sub(r"^\s*//.*$", "", raw, flags=re.M)
    try:
        cfg = json.loads(stripped)
    except json.JSONDecodeError:
        return None
    clock = cfg.get("clock") or {}
    step = clock.get("step-ns")
    rate = clock.get("real-time-update-rate")
    if not step or not rate:
        return None
    return round(float(rate) / float(step), 4)


# Project AirSim's ArduCopterApi: the robot config's controller when ArduPilot
# SITL, not the simulator's own simple_flight, flies the simulated vehicle
# (demo/pas_ardupilot/robot_citylife_ardupilot.jsonc).
ARDUPILOT_CONTROLLER = "ardupilot-api"


def _strip_jsonc(text: str) -> str:
    """// and /* */ comments OUTSIDE string literals removed (a robot config
    has comments inside objects, and a mesh path such as "/Drone/Quad" must
    survive). demo/pas_ardupilot/rail.strip_jsonc, which guardrail/ cannot
    import."""
    out, i, n, in_str = [], 0, len(text), False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            in_str = c != '"'
            i += 1
        elif c == '"':
            in_str = True
            out.append(c)
            i += 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def scene_controllers(scene_path: str | Path) -> tuple[list[dict], list[str]]:
    """Whose controller flew each robot of a Project AirSim scene, READ from
    the scene file and the robot config it names (the files the simulator was
    given, kept with the run): `([{"robot", "robot_config", "controller"}],
    problems)`. A file that cannot be read, a scene with no robot, or a robot
    config that cannot be found is a problem, never a guess."""
    problems: list[str] = []
    scene_path = Path(scene_path)
    try:
        scene = json.loads(_strip_jsonc(scene_path.read_text(encoding="utf-8")))
    except (OSError, ValueError) as exc:
        return [], [f"the scene file {str(scene_path)!r} could not be read "
                    f"({type(exc).__name__})"]
    robots = [a for a in scene.get("actors") or []
              if isinstance(a, dict) and a.get("type") == "robot"]
    if not robots:
        return [], [f"the scene {str(scene_path)!r} names no robot actor"]
    found = []
    for a in robots:
        name = a.get("robot-config")
        cfg_path = scene_path.parent / str(name or "")
        try:
            cfg = json.loads(_strip_jsonc(cfg_path.read_text(encoding="utf-8")))
            ctl = (cfg.get("controller") or {}).get("type")
        except (OSError, ValueError, AttributeError) as exc:
            problems.append(f"robot {a.get('name')!r}: its robot config "
                            f"{str(cfg_path)!r} could not be read "
                            f"({type(exc).__name__})")
            continue
        found.append({"robot": a.get("name"), "robot_config": str(name),
                      "controller": ctl})
    return found, problems


def _scene_not_ardupilot(scene_path: str | Path) -> list[str]:
    """Why ArduPilot is not shown to have flown this scene's robot. Empty =
    every robot config names the ArduCopterApi controller."""
    found, problems = scene_controllers(scene_path)
    for r in found:
        if r["controller"] != ARDUPILOT_CONTROLLER:
            problems.append(
                f"robot {r['robot']!r} uses controller {r['controller']!r} "
                f"({r['robot_config']}), not {ARDUPILOT_CONTROLLER!r}: the "
                f"simulator's own controller flew it, and a MAVLink link to an "
                f"ArduPilot that flies nothing is not an ArduPilot run")
    return problems


def _mavros_link_missing(ev: dict | None) -> list[str]:
    """dev or hil beside a scene: the link evidence must be MAVROS's own. A
    run whose evidence names the pymavlink link, or names no MAVLink URL
    MAVROS flew over, does not show the grant's MAVROS 2 safety path."""
    ev = ev or {}
    link = ev.get("link")
    if link is not None and link != "mavros":
        return [f"the autopilot link is {link!r}, not MAVROS 2 (the grant's "
                f"topologies fly the Shield's setpoints through MAVROS)"]
    nl = ev.get("network_link")
    if not isinstance(nl, dict) or not nl.get("fcu_url"):
        return ["no MAVROS link evidence: network_link names no fcu_url, so "
                "nothing shows which MAVLink link MAVROS flew over"]
    return []


def check_hil_evidence(ev: dict | None) -> list[str]:
    """What is missing before a run may call itself `dev`. Empty = nothing.

    (The name predates the relabel; the evidence is the MAVROS chain.) The
    grant's dev topology is ArduPilot SITL driven through MAVROS 2 on one
    desktop, so the evidence has to show all three links of that chain were
    live: a ROS 2 distribution, the MAVROS node itself, and a flight controller
    reporting connected. A run that merely imported rclpy has not demonstrated
    any of it.

    `fcu_connected` is the load-bearing one. MAVROS starts happily with nothing
    on the other end of the serial or UDP link and publishes `connected: false`
    forever, so a node can come up, run a whole mission into the void, and look
    healthy from the outside.

    `ardupilot_version` (audit card ARCH-31) is required since 2026-10-06: the
    autopilot that flew must be named, as the autopilot reports itself
    (AUTOPILOT_VERSION), or a run cannot be matched to a firmware build. The
    five stored ros2_* runs predate the field; nothing re-checks a stored run
    through this function (is_kpi_grade does not), so they are not demoted by
    it. Their firmware is on record separately: every dataflash log of that
    period reports "ArduCopter V4.8.0-dev (5f119834)" (sitl/setup_sitl.sh).
    """
    if not ev:
        return ["no evidence supplied"]
    missing = []
    if not ev.get("ros_distro"):
        missing.append("ros_distro not reported")
    if not ev.get("mavros_node"):
        missing.append("no MAVROS node seen on the graph")
    if ev.get("fcu_connected") is not True:
        missing.append(f"MAVROS reports fcu_connected={ev.get('fcu_connected')!r}, "
                       f"so nothing was flying")
    if not ev.get("ardupilot_version"):
        missing.append("ardupilot_version not reported: the autopilot that flew "
                       "is not named")
    return missing


# --------------------------------------------------------------------------- #
# The autopilot's own name for itself (ARCH-31)
# --------------------------------------------------------------------------- #

# MAVLink FIRMWARE_VERSION_TYPE, the low byte of flight_sw_version.
_FW_TYPE_SUFFIX = ((0, "-dev"), (64, "-alpha"), (128, "-beta"), (192, "-rc"),
                   (255, ""))
# MAV_TYPE -> ArduPilot vehicle name, for the types this project can fly.
_COPTER_TYPES = {2, 3, 4, 13, 14, 15, 29}
MAV_AUTOPILOT_ARDUPILOTMEGA = 3


def decode_custom_version(raw) -> str | None:
    """The 8-character git hash in AUTOPILOT_VERSION.flight_custom_version.

    ArduPilot sends the first 8 hex digits of its commit as 8 ASCII bytes.
    pymavlink hands them over as a list of ints; MAVROS's vehicle_info_get
    may hand them over as 16 hex digits of the little-endian uint64 those
    bytes form. Both are decoded back to the ASCII hash. Anything that does
    not decode to 8 hex digits is returned as given (or None when empty),
    never "repaired" into a plausible-looking hash.
    """
    if raw is None:
        return None
    hexdig = set("0123456789abcdef")
    as_given: str | None = None
    cands: list[bytes] = []
    if isinstance(raw, (list, tuple, bytes, bytearray)):
        cands.append(bytes(int(b) & 0xFF for b in raw))
    else:
        s = str(raw).strip().lower()
        if len(s) == 8 and set(s) <= hexdig:
            return s
        as_given = s or None
        if len(s) == 16 and set(s) <= hexdig:
            b = bytes.fromhex(s)
            cands += [b[::-1], b]
    for b in cands:
        t = b.rstrip(b"\x00").decode("ascii", "replace").lower()
        if len(t) == 8 and set(t) <= hexdig:
            return t
    return as_given


def describe_autopilot_version(flight_sw_version: int, custom: str | None,
                               mav_type: int | None = None,
                               autopilot: int | None = None) -> str:
    """"ArduCopter V4.5.7 (2a3dc4b7)" - the banner ArduPilot prints at boot -
    rebuilt from AUTOPILOT_VERSION, so the record reads like the firmware's
    own string (sitl/setup_sitl.sh --verify greps the same banner out of the
    binary)."""
    v = int(flight_sw_version)
    major, minor, patch, fw_type = (v >> 24) & 0xFF, (v >> 16) & 0xFF, \
        (v >> 8) & 0xFF, v & 0xFF
    suffix = ""
    for lo, s in _FW_TYPE_SUFFIX:
        if fw_type >= lo:
            suffix = s
    if autopilot not in (None, MAV_AUTOPILOT_ARDUPILOTMEGA):
        name = f"autopilot{autopilot}"
    elif mav_type is None or mav_type in _COPTER_TYPES:
        name = "ArduCopter"
    else:
        name = f"ArduPilot(MAV_TYPE {mav_type})"
    tail = f" ({custom})" if custom else ""
    return f"{name} V{major}.{minor}.{patch}{suffix}{tail}"


def ardupilot_version_from_mavlink(master, timeout: float = 3.0) -> dict | None:
    """Ask a running ArduPilot for AUTOPILOT_VERSION over pymavlink.

    Returns {"ardupilot_version", "flight_sw_version", "git_hash",
    "ardupilot_version_source"} or None. A READ from the autopilot, like
    `sim_speedup_from_mavlink`: None means it could not be read, and the
    caller must record that rather than fill in what the setup script pins.
    """
    try:
        mav = master.mav
        # MAV_CMD_REQUEST_MESSAGE (512) for AUTOPILOT_VERSION (148); the older
        # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (520) as the second try.
        for cmd, p1 in ((512, 148), (520, 1)):
            mav.command_long_send(master.target_system, master.target_component,
                                  cmd, 0, p1, 0, 0, 0, 0, 0, 0)
            msg = master.recv_match(type="AUTOPILOT_VERSION", blocking=True,
                                    timeout=timeout)
            if msg is not None:
                break
        else:
            return None
        custom = decode_custom_version(list(msg.flight_custom_version))
        mav_type = getattr(master, "mav_type", None)
        return {
            "ardupilot_version": describe_autopilot_version(
                msg.flight_sw_version, custom, mav_type,
                MAV_AUTOPILOT_ARDUPILOTMEGA),
            "flight_sw_version": f"{int(msg.flight_sw_version):08x}",
            "git_hash": custom,
            "ardupilot_version_source": "AUTOPILOT_VERSION (pymavlink)",
        }
    except Exception:                                     # noqa: BLE001
        return None


# --------------------------------------------------------------------------- #
# hil and flight: evidence from the machine and the link, not from a flag
# --------------------------------------------------------------------------- #

_DT_MODEL_PATHS = ("/proc/device-tree/model",
                   "/sys/firmware/devicetree/base/model")
_TEGRA_RELEASE = "/etc/nv_tegra_release"
_ARM64 = ("aarch64", "arm64")


def _read_text(path: str) -> str | None:
    try:
        with open(path, "rb") as fh:
            return fh.read().decode("utf-8", "replace").replace("\x00", "").strip()
    except OSError:
        return None


def _jetpack_version() -> str | None:
    try:
        r = subprocess.run(["dpkg-query", "-W", "-f=${Version}", "nvidia-jetpack"],
                           capture_output=True, text=True, timeout=10)
        return (r.stdout.strip() or None) if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def parse_l4t_release(text: str | None) -> str | None:
    """"# R36 (release), REVISION: 3.0, ..." -> "R36.3.0"."""
    if not text:
        return None
    m = re.search(r"R(\d+)\s*\(release\),\s*REVISION:\s*([\d.]+)", text)
    return f"R{m.group(1)}.{m.group(2)}" if m else None


def collect_host_evidence(read: Callable[[str], str | None] = _read_text,
                          machine: Callable[[], str] = platform.machine,
                          jetpack: Callable[[], str | None] = _jetpack_version
                          ) -> dict[str, Any]:
    """What machine is this node running on? Read, never declared.

    The grant puts the VLA and the Shield on a Jetson Orin for `hil` and
    `flight`. The device tree names the board ("NVIDIA Jetson AGX Orin
    Developer Kit"), /etc/nv_tegra_release names the L4T release that
    JetPack ships, and dpkg names the JetPack meta-package when it is
    installed. On a desktop all three are absent, which is the point.
    The readers are parameters so tests can describe an Orin without one.
    """
    model = None
    for p in _DT_MODEL_PATHS:
        # The device tree string is NUL-terminated; strip it whoever read it.
        model = (read(p) or "").replace("\x00", "").strip()
        if model:
            break
    return {
        "host_arch": machine(),
        "device_model": model or None,
        "l4t_release": parse_l4t_release(read(_TEGRA_RELEASE)),
        "jetpack": jetpack(),
        "hostname": socket.gethostname(),
    }


def _is_loopback_or_unset(host: str | None) -> bool:
    if not host or host in ("localhost", "0.0.0.0", "::"):
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host.endswith(".localhost")


def fcu_peer(fcu_url: str | None) -> str | None:
    """The REMOTE host a MAVROS fcu_url talks to, or None.

    MAVROS URLs: tcp://HOST:PORT, udp://[BIND]:PORT@[REMOTE]:PORT,
    serial:///dev/ttyX:BAUD. For udp the part after '@' is the remote; an
    empty remote ("udp://:14555@") means "whoever sends first", which proves
    nothing about where the autopilot is.
    """
    if not fcu_url:
        return None
    u = urlparse(fcu_url)
    if u.scheme in ("udp", "udp-b", "udp-pb"):
        rest = fcu_url.split("://", 1)[1]
        remote = rest.split("@", 1)[1] if "@" in rest else ""
        host = remote.rsplit(":", 1)[0] if ":" in remote else remote
        return host.strip("[]") or None
    if u.scheme in ("tcp", "tcp-l"):
        return u.hostname
    return None


def _route_iface(peer: str) -> str | None:
    try:
        r = subprocess.run(["ip", "route", "get", peer], capture_output=True,
                           text=True, timeout=5)
        m = re.search(r"\bdev\s+(\S+)", r.stdout)
        return m.group(1) if m else None
    except (OSError, subprocess.SubprocessError):
        return None


def network_link_evidence(fcu_url: str | None,
                          route_iface: Callable[[str], str | None] = _route_iface
                          ) -> dict[str, Any]:
    """The MAVLink link MAVROS flies over, as its fcu_url names it.

    `peer` is the remote the URL names (None for serial, and for udp with an
    empty remote, "whoever sends first"); `iface` / `link_kind` the route to
    it when it is not loopback. A Wi-Fi route is recorded as such, not
    refused: the grant asks for wired Ethernet between desktop and Orin, and
    the record lets a reader see it was not. Whether the link makes a run
    `hil` is decided in `check_topology_evidence`, together with where
    MAVROS and the autopilot run (`scan_local_processes`).
    """
    peer = fcu_peer(fcu_url)
    iface = route_iface(peer) if peer and not _is_loopback_or_unset(peer) else None
    kind = None
    if iface:
        kind = ("ethernet" if re.match(r"^(eth|en|eno|enp|enx|mgbe)", iface)
                else "wifi" if re.match(r"^(wl|wlan|wlp)", iface) else "other")
    return {"fcu_url": fcu_url, "peer": peer,
            "peer_is_loopback": _is_loopback_or_unset(peer),
            "iface": iface, "link_kind": kind}


# ArduPilot SITL binaries and launchers, MAVROS, and MAVLink routers, by the
# basename of an argv entry.
_SITL_PROCS = {"arducopter", "arducopter-heli", "arduplane", "ardurover",
               "ardusub", "blimp", "antennatracker", "sim_vehicle.py",
               "start_sitl.sh"}
_MAVROS_PROCS = {"mavros_node"}
_ROUTER_PROCS = {"mavlink-routerd", "mavproxy.py"}


def classify_process(argv: list[str]) -> str | None:
    """"sitl" | "mavros" | "router" | None for one process's argv. Only the
    first few entries are looked at (the executable, or an interpreter and
    its script), so a recorder whose ARGUMENTS name /mavros topics is not
    taken for MAVROS."""
    names = {os.path.basename(a) for a in argv[:6] if a and not a.startswith("-")}
    if names & _SITL_PROCS:
        return "sitl"
    if names & _MAVROS_PROCS:
        return "mavros"
    if names & _ROUTER_PROCS:
        return "router"
    return None


def scan_local_processes(proc_root: str | Path = "/proc") -> dict[str, Any] | None:
    """Which MAVLink actors run on THIS host, read from /proc; None where
    there is no /proc to read (Windows), which is "not scanned", never
    "nothing found".

    The grant's hil puts ArduPilot SITL (and MAVROS, and the GCS) on the
    desktop and the VLA + Shield on the Orin; this repository's Orin runbook
    puts MAVROS on the Orin beside the Shield. Either way the SITL autopilot
    must not run on the Shield's host, and only the host can say which
    processes it runs. A container sees its own PID namespace only, so a
    Shield node in a container without `pid: host` reports MAVROS in a sibling
    container as remote; the record says what was seen, nothing more.
    """
    root = Path(proc_root)
    try:
        entries = [d for d in root.iterdir() if d.name.isdigit()]
    except OSError:
        return None
    if not entries:
        return None
    me = str(os.getpid())
    found: dict[str, set] = {"sitl": set(), "mavros": set(), "router": set()}
    for d in entries:
        if d.name == me:
            continue
        try:
            raw = (d / "cmdline").read_bytes()
        except OSError:
            continue
        argv = [a for a in raw.decode("utf-8", "replace").split("\x00") if a]
        kind = classify_process(argv)
        if kind:
            found[kind].add(next(os.path.basename(a) for a in argv[:6]
                                 if os.path.basename(a) in
                                 _SITL_PROCS | _MAVROS_PROCS | _ROUTER_PROCS))
    return {"scanned": str(root),
            "local_sitl": sorted(found["sitl"]),
            "local_mavros": bool(found["mavros"]),
            "local_router": sorted(found["router"])}


def mavros_location(ev: dict | None) -> str | None:
    """Where MAVROS ran, relative to the Shield node: "shield_host" (this
    repository's Orin runbook: MAVROS beside the Shield), "remote" (the
    grant's hil table: MAVROS on the desktop with the simulators and the
    GCS), or None when this host's processes were not scanned."""
    procs = (ev or {}).get("local_processes")
    if not isinstance(procs, dict):
        return None
    return "shield_host" if procs.get("local_mavros") else "remote"


def _sitl_elsewhere_missing(ev: dict) -> list[str]:
    """hil: is the ArduPilot SITL autopilot shown to run off the Shield's host?

    Layout-independent since 2026-10-07. The rule before it asked for a
    non-loopback fcu_url remote, which only the PI reference's layout gives:
    the grant's layout (MAVROS on the desktop; its fcu_url is loopback
    there) and this repository's runbook (MAVROS on the Orin with
    `udp://:14555@`, the URL that works behind mavlink-router) were both
    refused, so no Orin run made by the runbook could ever be `hil`.
    """
    procs = ev.get("local_processes")
    scanned = isinstance(procs, dict)
    if scanned and procs.get("local_sitl"):
        return [f"ArduPilot SITL runs on the Shield's host "
                f"({', '.join(procs['local_sitl'])}): the autopilot is local, "
                f"which is dev, not hil"]
    if mavros_location(ev) == "remote":
        # The grant's layout. MAVROS's fcu_url points at SITL from the
        # desktop, so loopback there says nothing about this host.
        return []
    link = ev.get("network_link") or {}
    url, peer = link.get("fcu_url"), link.get("peer")
    if peer and _is_loopback_or_unset(peer):
        return [f"MAVROS fcu_url {url!r} names a loopback autopilot on this "
                f"host, not a remote desktop: the autopilot link is local, "
                f"which is dev, not hil"]
    if peer:
        return []          # MAVROS here names the desktop's address
    if scanned:
        return []          # no SITL here; MAVLink arrives from elsewhere
    return [f"MAVROS fcu_url {url!r} names no remote desktop, and this host's "
            f"processes were not scanned, so nothing shows the SITL autopilot "
            f"runs elsewhere"]


def _vla_host_missing(ev: dict) -> list[str]:
    """The grant puts the VLA on the Orin with the Shield (hil and flight).

    `vla_host` absent means the VLA runs in the Shield's own process (the
    in-process rails), so the Shield host's evidence covers it. A separate
    VLA node reports its host on /vla/identity; the ROS rail records that
    report, or None when the node sent none - which is refused."""
    if "vla_host" not in ev:
        return []
    vh = ev.get("vla_host")
    if not isinstance(vh, dict) or not vh:
        return ["the VLA node did not report its host (no 'host' in "
                "/vla/identity): the grant puts the VLA on the Orin too"]
    if "orin" not in str(vh.get("device_model") or "").lower():
        return [f"the VLA node runs on {vh.get('device_model')!r} "
                f"({vh.get('host_arch')}), not on a Jetson Orin"]
    return []


def orin_missing(ev: dict) -> list[str]:
    """Why `ev` does not show a Jetson Orin host. Empty = it does.

    The one rule for "this host is an Orin": the manifest's hil/flight checks
    and tools/profile_shield_tick.py both call it."""
    missing = []
    if str(ev.get("host_arch") or "").lower() not in _ARM64:
        missing.append(f"host_arch is {ev.get('host_arch')!r}, not aarch64: the "
                       f"Shield is not running on a Jetson")
    if "orin" not in str(ev.get("device_model") or "").lower():
        missing.append(f"device_model is {ev.get('device_model')!r}: no Jetson "
                       f"Orin named by the device tree")
    if not (ev.get("l4t_release") or ev.get("jetpack")):
        missing.append("neither the L4T release nor the JetPack version was read")
    return missing


# The earlier private name, kept for importers that still use it.
_orin_missing = orin_missing


def check_topology_evidence(topology: str, ev: dict | None) -> list[str]:
    """What is missing before a run may call itself `topology`. Empty = ok.

      dev     the MAVROS chain (check_hil_evidence).
      hil     dev's evidence, plus: the Shield runs on an Orin (aarch64, the
              device tree names an Orin, L4T/JetPack read); a separate VLA
              node, if any, reports an Orin too (`vla_host`); the autopilot
              is SITL (its SIM_SPEEDUP parameter was readable); and that SITL
              runs OFF the Shield's host (`_sitl_elsewhere_missing`), in
              either layout: MAVROS on the desktop (the grant's table) or on
              the Orin (this repository's runbook, `udp://:14555@`).
      flight  dev's evidence, plus the Orin (Shield and VLA), plus a hardware
              autopilot (no SIM_ parameter answered:
              `autopilot_kind == "hardware"`).
    """
    topology = normalize_topology(topology)
    missing = check_hil_evidence(ev)
    if topology == TOPOLOGY_DEV or not ev:
        return missing
    missing += orin_missing(ev)
    missing += _vla_host_missing(ev)
    if topology == TOPOLOGY_HIL:
        if ev.get("autopilot_kind") != "sitl":
            missing.append(f"autopilot_kind is {ev.get('autopilot_kind')!r}: hil "
                           f"flies ArduPilot SITL on the desktop")
        missing += _sitl_elsewhere_missing(ev)
    elif topology == TOPOLOGY_FLIGHT:
        if ev.get("autopilot_kind") != "hardware":
            missing.append(f"autopilot_kind is {ev.get('autopilot_kind')!r}: "
                           f"flight needs a hardware autopilot")
    else:
        missing.append(f"{topology!r} is not one of the grant's topologies")
    return missing


def detect_topology(ev: dict | None) -> str:
    """The label the evidence supports: flight, hil or dev (in that order).

    The node calls this at the end of a run and hands the answer to
    build_manifest, which checks the evidence again. A run that supports none
    of the three gets `dev` here and is then refused by build_manifest, so
    the caller's fallback label applies - never a silent upgrade.
    """
    for topo in (TOPOLOGY_FLIGHT, TOPOLOGY_HIL):
        if not check_topology_evidence(topo, ev):
            return topo
    return TOPOLOGY_DEV


def sim_speedup_from_mavlink(master, timeout: float = 3.0) -> float | None:
    """Read SIM_SPEEDUP from a running ArduPilot SITL, or None.

    The second derivation source, for the rail that has no scene file. It is a
    READ, like sim_speedup_from_scene: the value comes from the thing being
    measured. Returning None when the parameter cannot be fetched is the point -
    an unread speedup must surface as "unresolved", never as an assumed 1.0.
    """
    try:
        master.mav.param_request_read_send(
            master.target_system, master.target_component, b"SIM_SPEEDUP", -1)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is None:
            return None
        if msg.param_id.strip("\x00") != "SIM_SPEEDUP":
            return None
        return round(float(msg.param_value), 4)
    except Exception:                                     # noqa: BLE001
        return None


def build_manifest(policy_hash: str, model_id: str, seed: int,
                   scene_path: str | Path | None = None,
                   weight_paths: list[str | Path] | None = None,
                   topology: str = TOPOLOGY_PROJECTAIRSIM,
                   repo: str | Path | None = None,
                   sim_speedup: float | None = None,
                   hil_evidence: dict | None = None) -> dict[str, Any]:
    """The six fields, in the grant's order and under the grant's names.

    `sim_speedup` may be passed only for a rail with no scene file, and only
    with a value produced by a derivation helper such as
    `sim_speedup_from_mavlink()`. Never pass a literal: the whole reason this
    field is derived rather than declared is that a hand-written 1.0 is exactly
    the number a broken run would also carry.

    `topology` is stored under today's name: the legacy "canonical-hil" is
    accepted as input and written as "dev".

    `hil` and `flight` are the grant's Jetson Orin topologies. They used to be
    refused outright, because no rail ran on an Orin. Since 2026-10-06 the
    ROS 2 Shield node gathers the Orin's own evidence (device tree, L4T /
    JetPack, the MAVLink link it flies over; see `check_topology_evidence`),
    so the labels are accepted on that evidence and refused without it.

    A SCENE FILE BESIDE A GRANT LABEL (2026-10-12). A Project AirSim scene
    used to refuse dev, hil and flight outright: every scene then meant
    simple_flight, the simulator's own controller. The Project AirSim +
    ArduPilot rail (demo/pas_ardupilot) gives the scene's robot Project
    AirSim's ArduCopterApi controller, so ArduPilot SITL flies the simulated
    vehicle. `dev` and `hil` are now accepted beside a scene when the scene
    file and the robot config it names say `ardupilot-api` (READ from the
    files, `scene_controllers`), AND the evidence holds as without a scene:
    the live MAVROS chain for dev (check_hil_evidence), the Orin's evidence
    for hil, and for both MAVROS's own link (an fcu_url, and no pymavlink
    link). A simple_flight scene is refused as before, so is a pymavlink
    link, and `flight` (a real airframe) never has a simulator scene.
    """
    topology = normalize_topology(topology)
    if topology in (TOPOLOGY_HIL, TOPOLOGY_FLIGHT):
        missing = check_topology_evidence(topology, hil_evidence)
        if missing:
            raise ValueError(
                f"{topology!r} is the grant's Jetson Orin topology and may not "
                f"be claimed without the Orin's own evidence: "
                + "; ".join(missing)
                + f". A desktop ArduPilot SITL + MAVROS 2 run is "
                  f"{TOPOLOGY_DEV!r}.")
        if scene_path is not None and topology == TOPOLOGY_FLIGHT:
            raise ValueError(
                f"{topology!r} flies a real ArduPilot airframe; a Project "
                f"AirSim scene file ({str(scene_path)!r}) contradicts that")
        if scene_path is not None:
            why = _scene_not_ardupilot(scene_path) + _mavros_link_missing(
                hil_evidence)
            if why:
                raise ValueError(
                    f"{topology!r} runs the Shield on an Orin against "
                    f"ArduPilot; beside the Project AirSim scene "
                    f"{str(scene_path)!r} that needs ArduPilot to have flown "
                    f"the scene's robot through MAVROS: " + "; ".join(why))
    if topology == TOPOLOGY_DEV:
        # The guard opens on EVIDENCE, never on the caller's word. `dev` is the
        # one label that becomes KPI-eligible if the PI grants the waiver
        # (DEV_KPI_WAIVER), so the caller must show it was actually talking to
        # a flight controller through MAVROS 2 - a fact only a live connection
        # can produce.
        #
        # The evidence is checked here and recorded in the run's metrics.json,
        # not in the manifest: the manifest is exactly six fields by the grant's
        # definition and stays that way.
        missing = check_hil_evidence(hil_evidence)
        if missing:
            raise ValueError(
                "dev is the grant's desktop ArduPilot SITL + MAVROS 2 topology "
                "and may not be claimed without evidence of it: "
                + "; ".join(missing))
        # A scene file is itself evidence, and it contradicts the claim.
        #
        # The dev rail here is ArduPilot SITL: there is no Project AirSim scene
        # in it, which is precisely why `sim_speedup` may be passed in for a
        # rail with no scene file (see below). So a caller handing over BOTH a
        # scene path and the MAVROS topology is describing two different
        # stacks at once, and the honest answer is to refuse rather than to
        # believe the half that flatters the run.
        #
        # Without this the evidence check was the only gate, and evidence is a
        # dict the caller assembles. Our own Project AirSim scene passed
        # straight through and got stamped with the MAVROS label, the one this
        # module treats as waiver-eligible.
        #
        # Since 2026-10-12 the one exception is a scene whose robot ArduPilot
        # flies (the docstring, "A scene file beside a grant label"): the
        # controller is READ from the scene's own files, and the MAVROS link
        # evidence must hold too. A simple_flight scene is still refused,
        # whatever evidence accompanies it.
        if scene_path is not None:
            why = _scene_not_ardupilot(scene_path) + _mavros_link_missing(
                hil_evidence)
            if why:
                raise ValueError(
                    "dev here is the ArduPilot SITL + MAVROS 2 topology; beside "
                    f"a simulator scene (scene_path={str(scene_path)!r}) it "
                    f"needs ArduPilot to have flown the scene's robot through "
                    f"MAVROS: " + "; ".join(why) + ". A run whose scene the "
                    f"simulator's own controller flew is "
                    f"{TOPOLOGY_PROJECTAIRSIM!r}, whatever evidence "
                    f"accompanies it.")
    speedup = (sim_speedup if scene_path is None
               else sim_speedup_from_scene(scene_path))
    return {
        "code_revision": code_revision(repo),
        "vla_model_hash": model_hash(model_id, weight_paths),
        "policy_hash": policy_hash or UNRESOLVED,
        "random_seed": int(seed),
        "sim_speedup": UNRESOLVED if speedup is None else speedup,
        "topology": topology,
    }


def is_kpi_grade(manifest: dict, metrics: dict,
                 dev_waiver: str | None = None) -> tuple[bool, list[str]]:
    """May this run's numbers be quoted as contractual KPIs?

    Every check below exists because the corresponding failure already happened
    and produced a number that looked fine.

    Topology: the grant's Stress Testing page says every reported KPI comes
    from a run "executed in the canonical HIL topology", and its Architecture
    constraints make that `hil` (Jetson Orin). `dev` - our desktop SITL +
    MAVROS 2 rail, including runs stored as "canonical-hil" - qualifies only
    under a written PI waiver, passed here or recorded in DEV_KPI_WAIVER.
    """
    reasons: list[str] = []
    waiver = dev_waiver if dev_waiver is not None else DEV_KPI_WAIVER

    if manifest.get("sim_speedup") != 1.0:
        reasons.append(
            f"sim_speedup is {manifest.get('sim_speedup')}, and 1.0 is mandatory "
            f"for a KPI-bearing run")

    stored = manifest.get("topology")
    topo = normalize_topology(stored)
    shown = repr(stored) if stored == topo else f"{stored!r} (read as {topo!r})"
    if topo == TOPOLOGY_HIL:
        # The label alone is a string anyone can type into a manifest. A hil
        # run carries the Orin's evidence beside it (metrics.json
        # "hil_evidence"), and the gate re-checks it here, so a hand-edited
        # "hil" is refused rather than quoted.
        ev = metrics.get("hil_evidence")
        missing = check_topology_evidence(TOPOLOGY_HIL, ev) if ev else [
            "no hil_evidence recorded beside the manifest"]
        if missing:
            reasons.append(f"topology is {shown} but its evidence does not "
                           f"hold: " + "; ".join(missing))
    elif topo == TOPOLOGY_DEV:
        if not waiver:
            reasons.append(
                f"topology is {shown}: the grant's desktop configuration, which "
                f"it does not use for reported KPI numbers - those come from "
                f"{TOPOLOGY_HIL!r} (Jetson Orin). A dev run counts only under a "
                f"written PI waiver, and none is recorded (PQ1)")
    else:
        reasons.append(
            f"topology is {shown}, not the grant's {TOPOLOGY_HIL!r} (nor a "
            f"waivable {TOPOLOGY_DEV!r}): functional-rail evidence, not a "
            f"contractual KPI number")

    for field in ("code_revision", "vla_model_hash", "policy_hash"):
        v = str(manifest.get(field, ""))
        if UNRESOLVED in v or v in ("", "unversioned"):
            reasons.append(f"{field} did not resolve ({v!r})")

    # A dirty tree means the named commit is not the code that flew: checking it
    # out would give you something else. "-unknown" is the same problem wearing a
    # politer word - git could not say whether the tree was clean.
    rev = str(manifest.get("code_revision", ""))
    if rev.endswith("-dirty"):
        reasons.append(
            f"code_revision is {rev!r}: the working tree had uncommitted changes, "
            f"so checking out that commit does not reproduce this run")
    elif rev.endswith("-unknown"):
        reasons.append(
            f"code_revision is {rev!r}: git could not report whether the tree was "
            f"clean, so the revision cannot be trusted to describe the run")

    hz = metrics.get("det_hz")
    if hz is not None and hz < MIN_DET_HZ:
        reasons.append(
            f"detector managed only {hz} Hz; hit rate is measured over the "
            f"inferences that ran and is not a tracking result below {MIN_DET_HZ}")

    err = metrics.get("start_heading_err_deg")
    if err is not None and abs(err) > MAX_HEADING_ERR_DEG:
        reasons.append(
            f"start heading was {err:.1f} deg off; whether the subject was in "
            f"frame at all is then luck, and the run is not comparable")

    return (not reasons), reasons


def policy_source_record(kind: str, path: str | None, policy_hash: str,
                         **extra) -> dict[str, Any]:
    """Where a flight's policy came from, in one shape for every entry point.

    Written beside the manifest (metrics.json, kpi.json, the replay index), not
    into it: the manifest is the grant's six fields and stays six. `kind` is
    "bundle" (then `signature` says whether it verified) or "yaml" - which is
    always `signature: "unsigned"`, because a YAML file carries none.
    """
    rec = {"kind": kind, "path": path, "loaded_policy_hash": policy_hash}
    if kind == "yaml":
        rec["signature"] = "unsigned"
        rec["signature_detail"] = ("loaded from YAML, not from a signed bundle; "
                                   "the policy hash identifies it but nothing "
                                   "attests who issued it")
    rec.update(extra)
    return rec


# A flight rail flies a YAML policy only when told this (2026-10-12). Until
# then both SITL rails flew policies/sim_demo_policy.yaml, unsigned, whenever
# no --bundle was given, so a run's policy was "whatever the YAML said that
# day", identified by its hash and attested by nobody.
DEV_YAML_FLAG = "--dev-yaml"
BUNDLES_DIR = Path(__file__).resolve().parents[1] / "bundles"


def flight_bundle_path(policy, bundles_dir: str | Path | None = None) -> Path:
    """Where the signed bundle of `policy` (a Policy or a policy file) lives:
    bundles/<policy_id>-v<version>.tar.gz, the name `python -m
    guardrail.bundle` and tools/build_flight_bundles.py write."""
    if not hasattr(policy, "policy_id"):
        from .models import load_policy
        policy = load_policy(policy, runtime=False)
    base = Path(bundles_dir) if bundles_dir is not None else BUNDLES_DIR
    return base / f"{policy.policy_id}-v{policy.version}.tar.gz"


def flight_bundle_status(yaml_path, bundle=None,
                         bundles_dir: str | Path | None = None) -> dict[str, Any]:
    """Is the signed bundle of `yaml_path` on disk, and is it STILL that YAML's
    policy? `{bundle, present, bundle_hash, yaml_hash, matches, why}`.

    A bundle is named by policy id and version alone (`flight_bundle_path`),
    and bundles/ is a gitignored build product: a YAML edited without a
    version bump keeps its bundle's name while the bundle flies the old
    rules (review 2026-10-13: an edited follow_car_citylife.yaml flew
    sha256:4f7ebad6... from its stale bundle against the YAML's
    sha256:59883... with no word said). `matches` is None when the bundle is
    absent or cannot be read (the flight loader then refuses it with its own
    reason); the signature is not judged here."""
    from .models import load_policy
    pol = load_policy(yaml_path, runtime=False)
    path = Path(bundle) if bundle is not None else flight_bundle_path(pol, bundles_dir)
    out: dict[str, Any] = {"bundle": str(path), "present": path.is_file(),
                           "bundle_hash": None, "yaml_hash": pol.policy_hash,
                           "matches": None, "why": None}
    if not out["present"]:
        out["why"] = "no bundle on disk"
        return out
    try:
        from .bundle import check_bundle
        chk = check_bundle(path)
        out["bundle_hash"] = chk.source_record()["loaded_policy_hash"]
    except Exception as exc:                                  # noqa: BLE001
        out["why"] = f"bundle unreadable here: {type(exc).__name__}: {exc}"
        return out
    out["matches"] = out["bundle_hash"] == out["yaml_hash"]
    if not out["matches"]:
        out["why"] = (f"{path.name} flies {out['bundle_hash']}, but {yaml_path} is "
                      f"now {out['yaml_hash']} (edited without a version bump?): "
                      f"rebuild it (python tools/build_flight_bundles.py)")
    return out


def flight_policy_choice(bundle, yaml_path, *, dev_yaml: bool,
                         default_yaml, default_bundle=None
                         ) -> tuple[str | None, str | None, dict[str, Any]]:
    """Which policy a flight rail loads: `(bundle, yaml, record)`, the first
    two for guardrail.bundle.load_for_flight, `record` for its policy source.

      --bundle B             B (signature-verified by load_for_flight);
                             with --dev-yaml too, refused: two answers.
      --policy P             refused unless --dev-yaml: P is unsigned.
      --policy P --dev-yaml  P, recorded `dev_yaml: true`.
      neither                the signed bundle of the rail's default policy
                             (`default_bundle`) when it is on disk, recorded
                             `default_bundle: true`; with --dev-yaml the
                             default YAML instead; with neither, refused,
                             naming both ways out. A default bundle that no
                             longer flies the default YAML's policy (edited
                             without a version bump: flight_bundle_status) is
                             refused, naming both hashes.

    Both --bundle and --policy are handed on unchanged, so load_for_flight
    refuses them with its own message ("not both")."""
    if bundle and yaml_path:
        return bundle, yaml_path, {}
    if bundle:
        if dev_yaml:
            raise ValueError(f"{DEV_YAML_FLAG} is for flying a YAML policy, and "
                             f"--bundle flies the signed artefact: give one")
        return str(bundle), None, {}
    if yaml_path or dev_yaml:
        if not dev_yaml:
            raise ValueError(
                f"{yaml_path} is a YAML policy: unsigned, so nothing attests "
                f"who issued it. Fly its signed bundle (python "
                f"tools/build_flight_bundles.py builds them into bundles/, "
                f"then --bundle bundles/<policy_id>-v<version>.tar.gz), or "
                f"pass {DEV_YAML_FLAG} to fly the YAML as a development run, "
                f"recorded as such")
        return None, str(yaml_path or default_yaml), {
            "dev_yaml": True,
            "dev_yaml_note": f"flown from YAML under {DEV_YAML_FLAG}: a "
                             f"development run, not the issued artefact"}
    if default_bundle is not None and Path(default_bundle).is_file():
        try:
            st = flight_bundle_status(default_yaml, bundle=default_bundle)
        except Exception:                                     # noqa: BLE001
            st = {"matches": None}          # the YAML unreadable: the bundle decides
        if st["matches"] is False:
            raise ValueError(
                f"the default policy's signed bundle is stale: {st['why']}; or "
                f"pass {DEV_YAML_FLAG} to fly {default_yaml} unsigned, recorded "
                f"as such")
        return str(default_bundle), None, {"default_bundle": True}
    raise ValueError(
        f"no policy given, and the signed bundle of the default policy is not "
        f"on disk ({default_bundle}). Build it (python "
        f"tools/build_flight_bundles.py), give --bundle, or pass "
        f"{DEV_YAML_FLAG} to fly {default_yaml} unsigned, recorded as such")


def resolve_run_policy(manifest: dict, candidates) -> tuple[Any, str | None,
                                                            str | None]:
    """The policy a stored run flew under: `(policy, hash_form, label)`.

    `candidates` is `[(label, Policy), ...]`, normally
    `guardrail.bundle.policy_candidates()` - every policy file plus the
    hot-applied derivations the lock records. Every identity form is tried:
    the 64-hex hash and each 16-hex legacy form (`Policy.hash_form`). Returns
    `(None, None, None)` when nothing matches; the caller must report that run
    as unverifiable rather than pick the nearest policy.
    """
    recorded = (manifest or {}).get("policy_hash")
    if not recorded or UNRESOLVED in str(recorded):
        return None, None, None
    for label, pol in candidates:
        form = pol.hash_form(recorded)
        if form is not None:
            return pol, form, label
    return None, None, None
