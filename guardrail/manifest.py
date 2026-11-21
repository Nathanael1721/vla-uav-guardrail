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

from guardrail.delivery import grade_reasons as delivery_grade_reasons

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
# The same Project AirSim rail with the simulator on ANOTHER host: the Shield,
# the controller and the pilot run where this process runs (the Jetson Orin
# against the desktop's simulator, --sim-address), the simulator elsewhere.
# Until 2026-10-28 these runs were stamped single-host. Not the grant's hil:
# no ArduPilot, no MAVROS (hil needs both, check_topology_evidence).
TOPOLOGY_PROJECTAIRSIM_REMOTE = "projectairsim-remote-sim"
LOOPBACK_ADDRESSES = ("127.0.0.1", "localhost", "::1")


def projectairsim_topology(sim_address: str | None) -> str:
    """The Project AirSim rail's label for a simulator at `sim_address`:
    single-host on loopback (or none given), remote-sim otherwise."""
    if sim_address is None or str(sim_address).strip().lower() in LOOPBACK_ADDRESSES:
        return TOPOLOGY_PROJECTAIRSIM
    return TOPOLOGY_PROJECTAIRSIM_REMOTE
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


def _code_candidates(repo: str | Path | None) -> list[Path]:
    """Where code_revision (and git_describe) look, in order: the caller's
    repository, then this checkout, then the reference fork beside it."""
    here = Path(__file__).resolve().parents[1]
    return ([Path(repo)] if repo else []) + [here, here / "kuanting-vla-uav-guardrail"]


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
    for cand in _code_candidates(repo):
        try:
            head = subprocess.run(["git", "-C", str(cand), "rev-parse", "HEAD"],
                                  capture_output=True, text=True, timeout=15)
            if head.returncode != 0:
                continue
            if not _tracks_our_code(cand):
                continue
            rev = head.stdout.strip()[:12]
            # Its own try: once HEAD resolved in a repo that tracks the code,
            # a status that times out or fails says only "could not tell
            # whether the tree is clean" (-unknown). Until 2026-10-19 the
            # timeout fell through to the outer except, the candidate was
            # dropped, and the known HEAD came back as "unversioned".
            # A second, longer try (2026-11-02): from WSL, the first status
            # on the OneDrive tree refreshes the whole index over /mnt/d and
            # outlasted 20 s, so the first Gazebo rail run recorded a clean
            # tree as "-unknown" while the next status took 2.4 s.
            dirty = None
            for status_timeout_s in (20, 90):
                try:
                    st = subprocess.run(["git", "-C", str(cand), "status", "--porcelain"],
                                        capture_output=True, text=True,
                                        timeout=status_timeout_s)
                except (OSError, subprocess.SubprocessError):
                    continue
                if st.returncode == 0:
                    dirty = bool(st.stdout.strip())
                    break
            suffix = "-dirty" if dirty else ("" if dirty is False else "-unknown")
            return f"{rev}{suffix}"
        except (OSError, subprocess.SubprocessError):
            # rev-parse or the ownership check failed: this candidate cannot
            # be shown to hold the code, so it is not reported.
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
    """One SHA-256 over every weight file's own SHA-256, or None if no file -
    or if any named weight could not be read.

    The grant's manifest field is "vla_model_hash: <SHA-256 of model weights>"
    (reference data-flow.md). Until 2026-10-06 this module hashed each file's
    NAME, SIZE and MTIME instead of its bytes and cut the result to 16 hex, so
    two different checkpoints of the same size written in the same second
    shared a hash, and a byte-identical copy did not (audit card X-16).

    Each file is listed by its path RELATIVE to the directory it was found in
    plus its full digest, sorted, so the result does not depend on where the
    weights live on disk. Logs and JSONL files are skipped, as before: they
    are written beside adapters during training and are not weights.

    A named path that does not exist, a dangling link (a missing blob in an
    interrupted HuggingFace download) and a file that cannot be read make the
    result None. Until 2026-10-19 each was skipped without a word and the
    digest of the REST came back as the weights hash - two checkpoints that
    differed only in an unreadable shard shared it, and the KPI gate passed it.
    """
    cpath = Path(cache_path) if cache_path else _digest_cache_path()
    cache = _load_digest_cache(cpath)
    before = dict(cache)
    lines: list[str] = []
    missing: list[str] = []
    for p in weight_paths:
        p = Path(p)
        if p.is_dir():
            files = [(f.relative_to(p).as_posix(), f) for f in sorted(p.rglob("*"))]
        elif p.is_file():
            files = [(p.name, p)]
        else:
            missing.append(str(p))
            continue
        for rel, f in files:
            if f.suffix in (".log", ".jsonl") or f.is_dir():
                continue
            if not f.is_file():                 # dangling link, special file
                missing.append(rel)
                continue
            try:
                lines.append(f"{rel}  {file_sha256(f, cache)}")
            except OSError:
                missing.append(rel)
    if missing or not lines:
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
    if rev and rev.startswith("ambiguous:"):
        # Several snapshots cached: the id does not say which weights ran.
        # Until 2026-10-19 this was cut to 16 characters ("org/m@ambiguous:
        # aaaaaa") BEFORE anything looked at it, and is_kpi_grade, which
        # refuses only "unresolved", passed it as a resolved weights hash.
        if not weight_paths:
            return f"{model_id}@{UNRESOLVED}:{rev}"        # whole text, not cut
        # The bytes hashed below pin the weights; mixing cache-dependent text
        # into that digest would hide the ambiguity and move the hash whenever
        # an unrelated snapshot is added or removed.
        rev = None
    if rev and not weight_paths:
        return f"{model_id}@{rev[:16]}"

    digest = weights_digest(list(weight_paths)) if weight_paths else None
    if digest is None and weight_paths:
        # Weights were NAMED but could not all be read (weights_digest says
        # None for a missing path, a dangling link or an unreadable file):
        # never an @src: or @sha256: hash over whatever was left.
        return f"{model_id}@{UNRESOLVED}"
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


REPORTED_DIGEST_MISMATCH = "reported-digest-mismatch"
_HEX64 = re.compile(r"[0-9a-f]{64}")


def resolve_vla_hash(model_id: str, weight_paths: list[str | Path] | None = None,
                     reported_digest: str | None = None) -> tuple[str, str]:
    """(vla_model_hash, where it came from) - `model_hash` plus the REPORTED
    path (wave V1, 2026-10-21).

    AerialVLA runs in its own server process (demo/vla_server_v2.py), which
    holds the weights, hashes the base shards + adapter with
    `weights_digest` and returns that `weights_sha256` with every action. A
    flight process that cannot read those files - the ROS companion
    container, a client on another host - got "<id>@unresolved" from
    `model_hash`, and is_kpi_grade refused the run for a model that was in
    fact identified (integration discovery report, section 7). So:

      * no report: `model_hash(model_id, weight_paths)`, source "local";
      * a report, and the named weights cannot all be read here (or none
        are named): "<id>@sha256:<reported>", source "reported";
      * a report, and the weights CAN be read here: their local digest must
        equal the report - then `model_hash`'s own answer, source "local;
        reported agrees" - and if it differs, "<id>@unresolved:
        reported-digest-mismatch", source "mismatch": the server ran other
        bytes than the ones this host holds under those names, and the KPI
        gate (which refuses "unresolved") must not quote the run.

    The digest compared is `weights_digest` over the SAME list the server
    hashes (vla_server_v2.weight_paths: the base model-*.safetensors shards
    and the adapter directory), so the two agree byte for byte. A model id
    that resolves a HuggingFace snapshot has that revision mixed into the
    local hash (`model_hash`); the server's ids ("aerialvla-lora/aero_vla@
    196f2f32") do not, which is why the agreeing case can return the local
    form unchanged.

    Only a 64-character lowercase hex digest is accepted, and only for a
    learned VLA: the no-VLA id, an id `model_hash` pins by its SOURCE file
    (a code-only pilot such as "guardrail.vla_stub.StubVLA") and a "fake:*"
    stand-in (vla_server_v2's scripted model) have no weights to report.
    The guard is here, not only in pilot_record's kind check, because
    build_manifest calls this directly with no kind: until the review of V1a
    (P8) a stub id with a reported digest got "StubVLA@sha256:<digest>"
    instead of "@src:...", and a code-only pilot looked weight-identified to
    anything that classifies runs by hash form."""
    if reported_digest is None:
        return model_hash(model_id, weight_paths), "local"
    if not (isinstance(reported_digest, str) and _HEX64.fullmatch(reported_digest)):
        raise ValueError(f"reported_digest must be 64 lowercase hex characters, "
                         f"got {reported_digest!r}")
    if model_id == NO_VLA:
        raise ValueError(f"{NO_VLA!r} has no weights to report a digest for")
    if model_id.startswith("fake:") or _source_of(model_id) is not None:
        what = ("a scripted stand-in" if model_id.startswith("fake:")
                else "a code-only pilot, hashed by its source file")
        raise ValueError(f"{model_id!r} has no weights to report a digest for: it is {what}; "
                         f"a reported weights digest belongs to a learned VLA only")
    local = weights_digest(list(weight_paths)) if weight_paths else None
    if local is None:
        return f"{model_id}@sha256:{reported_digest}", "reported"
    if local != reported_digest:
        return f"{model_id}@{UNRESOLVED}:{REPORTED_DIGEST_MISMATCH}", "mismatch"
    return model_hash(model_id, weight_paths), "local; reported agrees"


def pilot_record(model_id: str, kind: str,
                 weight_paths: list[str | Path] | None = None,
                 reported_digest: str | None = None,
                 reported_by: str | None = None,
                 **extra) -> dict[str, Any]:
    """What flew the aircraft, beside the six-field manifest (audit card X-16).

    `kind` is one of "vla" (a learned model in the VLA slot), "stub" (a
    code-only stand-in such as guardrail.vla_stub.StubVLA) or "none" (no VLA:
    a hand-written controller, model_id NO_VLA). The manifest keeps exactly
    the grant's six fields; this record says in words what its
    `vla_model_hash` is a hash OF, and carries anything else that steered the
    run (a detector, a tokenizer) under its own key, so a detector is never
    filed as the VLA again.

    `reported_digest` (kind "vla" only) is the weights_sha256 a VLA server
    returned, for a flight process that cannot see the weights; `reported_by`
    says which server (host:port). The record then also carries
    `hash_source`, the digest and its reporter (see `resolve_vla_hash`).
    Without a report the record keeps its three-key shape.
    """
    if kind not in ("vla", "stub", "none"):
        raise ValueError(f"pilot kind {kind!r} is not one of vla / stub / none")
    if kind == "none" and model_id != NO_VLA:
        raise ValueError(f"a run with no VLA records model_id {NO_VLA!r}, "
                         f"not {model_id!r}")
    if reported_digest is not None and kind != "vla":
        raise ValueError(f"a {kind!r} pilot has no weights; a reported weights digest "
                         f"belongs to kind 'vla' only")
    h, source = resolve_vla_hash(model_id, weight_paths, reported_digest)
    rec = {"kind": kind, "model_id": model_id, "hash": h}
    if reported_digest is not None:
        rec.update({"hash_source": source, "reported_digest": reported_digest,
                    "reported_by": reported_by})
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

    The ratio is step-ns / real-time-update-rate: every real-time-update-rate
    ns of wall time the clock advances step-ns of sim time, so this is sim
    seconds per wall second - the convention of ArduPilot's SIM_SPEEDUP, which
    `sim_speedup_from_mavlink` writes into the same manifest field. Project
    AirSim's own example settles the direction (client/python/
    airsimv1_scripts_migrated/multirotor/clock_speed.py: step 1.5 ms at a 3 ms
    update rate makes a 3 s move take 6 s of wall time, i.e. 0.5). Until
    2026-10-19 this returned the reciprocal; equal values gave 1.0 either way,
    so the KPI gate never decided differently, but a non-real-time scene was
    recorded upside down (a 0.25x clock as 4.0). Comments are stripped with
    _strip_jsonc (trailing // and /* */ too; only whole-line // were before).
    """
    try:
        raw = Path(scene_path).read_text(encoding="utf-8")
    except OSError:
        return None
    try:
        cfg = json.loads(_strip_jsonc(raw))
    except ValueError:                       # json.JSONDecodeError is one
        return None
    clock = cfg.get("clock") if isinstance(cfg, dict) else None
    if not isinstance(clock, dict):
        return None
    step = clock.get("step-ns")
    rate = clock.get("real-time-update-rate")
    if not step or not rate:
        return None
    try:
        return round(float(step) / float(rate), 4)
    except (TypeError, ValueError):
        return None


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

# The host's board-name file as the Orin compose file mounts it, read-only, into
# the containers that gather evidence. Docker's default AppArmor profile denies
# /sys/firmware/** inside a container (where /proc/device-tree points), and
# security_opt systempaths=unconfined does not lift that: on the AGX Orin
# (JetPack 7.2.1, AppArmor on) the read was "Permission denied" and the first
# facts run said is_orin false (2026-11-06). The file is still the device
# tree's own string, seen through a read-only bind mount.
_DT_MODEL_MOUNT = "/run/vlaguard/host/device-tree-model"
_DT_MODEL_PATHS = ("/proc/device-tree/model",
                   "/sys/firmware/devicetree/base/model",
                   _DT_MODEL_MOUNT)
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


def _pem_sha256_fingerprint(path: Path) -> str | None:
    """`openssl x509 -fingerprint -sha256` of a PEM certificate, in plain
    Python (base64 of the first CERTIFICATE block -> DER -> SHA-256)."""
    import base64
    try:
        text = path.read_text(encoding="ascii")
    except (OSError, UnicodeDecodeError):
        return None
    m = re.search(r"-----BEGIN CERTIFICATE-----(.+?)-----END CERTIFICATE-----", text, re.S)
    if not m:
        return None
    try:
        der = base64.b64decode("".join(m.group(1).split()), validate=True)
    except ValueError:
        return None
    return ":".join(f"{b:02X}" for b in hashlib.sha256(der).digest())


def ros2_security_evidence(environ=None) -> dict[str, Any] | None:
    """SROS 2 on this node's process, read from the environment and the
    keystore - None when security is not enabled (ROS_SECURITY_ENABLE is not
    "true"), so a run without SROS 2 records exactly what it recorded before
    2026-10-19.

    The grant's data-flow page asks for "SROS 2 turned on for the ROS 2
    graph using the same root CA" as the harness mTLS. A run flown with
    SROS2=1 (sitl/run_ros2_demo.sh) records here which strategy was in force,
    the enclave this process joined as, whether its security files are in
    the keystore, and the SHA-256 fingerprint of the identity CA - the
    number `python -m guardrail.harness_service gen-pki` printed for the lab
    CA, so a reader can check "same root CA" from the record alone. Under
    Enforce, rcl refuses a node whose files are missing before it exists
    (tools/sros2_check.sh, check 3), so a flown Enforce run that records
    files_present True is what was flown.

    Since 2026-11-03 (X-15) it also records the enclave's signed permissions:
    `permissions_sha256` (of permissions.p7s, the file the DDS security
    plugin loads) and `permissions_scope` - "allow_all" when they still grant
    every topic (sros2's create_enclave default, `<topic>rt/*</topic>`),
    "per_topic" when they are a policy's narrower grant
    (sitl/sros2_policy.xml through tools/sros2_setup.sh), None when the file
    cannot be read as permissions. The p7s is S/MIME clear-signed, so the
    granted topics are readable without verifying it; tools/sros2_check.sh
    check 8 verifies it against the CA."""
    env = os.environ if environ is None else environ
    if str(env.get("ROS_SECURITY_ENABLE", "")).lower() != "true":
        return None
    ks = env.get("ROS_SECURITY_KEYSTORE")
    enclave = env.get("ROS_SECURITY_ENCLAVE_OVERRIDE")
    out: dict[str, Any] = {"enabled": True,
                           "strategy": env.get("ROS_SECURITY_STRATEGY") or "Permissive",
                           "keystore": ks, "enclave": enclave,
                           "enclave_source": ("ROS_SECURITY_ENCLAVE_OVERRIDE" if enclave
                                              else "none (the node's --enclave, or /)")}
    files = ("cert.pem", "key.pem", "permissions.p7s", "governance.p7s",
             "identity_ca.cert.pem", "permissions_ca.cert.pem")
    if ks and enclave:
        d = Path(ks) / "enclaves" / enclave.lstrip("/")
        out["files_present"] = all((d / f).exists() for f in files)
        out["identity_ca_sha256"] = _pem_sha256_fingerprint(d / "identity_ca.cert.pem")
        out["permissions_ca_sha256"] = _pem_sha256_fingerprint(d / "permissions_ca.cert.pem")
        out.update(_permissions_scope(d / "permissions.p7s"))
    else:
        out["files_present"] = None
        out["identity_ca_sha256"] = out["permissions_ca_sha256"] = None
        out["permissions_sha256"] = out["permissions_scope"] = None
    return out


def _permissions_scope(p7s: Path) -> dict[str, Any]:
    """`permissions_sha256` and `permissions_scope` of an enclave's
    permissions.p7s (ros2_security_evidence)."""
    try:
        data = p7s.read_bytes()
    except OSError:
        return {"permissions_sha256": None, "permissions_scope": None}
    if b"<grant" not in data:
        scope = None
    elif b"<topic>rt/*</topic>" in data:
        scope = "allow_all"
    else:
        scope = "per_topic"
    return {"permissions_sha256": hashlib.sha256(data).hexdigest(),
            "permissions_scope": scope}


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
                   hil_evidence: dict | None = None,
                   reported_digest: str | None = None) -> dict[str, Any]:
    """The six fields, in the grant's order and under the grant's names.

    `reported_digest`: the weights_sha256 a VLA server returned, for a flight
    process that cannot see the weights; vla_model_hash then follows
    `resolve_vla_hash`, the rule pilot_record applies, so the two agree.

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
        "vla_model_hash": resolve_vla_hash(model_id, weight_paths, reported_digest)[0],
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
    # politer word - git could not say whether the tree was clean. "-dirty"
    # also covers an untracked file git does not ignore (`git status
    # --porcelain` lists it; an untracked .py really changes what runs), so the
    # reason says so: on 2026-10-18 a tool's private, unignored folder was the
    # only entry, and "uncommitted changes" sent the reader looking for code.
    rev = str(manifest.get("code_revision", ""))
    if rev.endswith("-dirty"):
        reasons.append(
            f"code_revision is {rev!r}: the working tree had uncommitted changes "
            f"or untracked files git does not ignore (`git status --porcelain` "
            f"names them), so checking out that commit does not reproduce this run")
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

    # A mission of a functional sweep (2026-11-17, WP4-04): the stress
    # harness's scenario record (metrics.json `scenario`) says it was flown on
    # the Gazebo functional rail. Refused whatever its manifest reads, a dev
    # waiver included: the lab's choice to take KPI-grade runs from hil only,
    # not the grant's wording - the grant's 03-simulation/dual-rail.md gives
    # the functional rail KPI runs at sim_speedup=1, and its "tracked but not
    # part of the final KPI numbers" covers the broad sweep at speed > 1 only.
    # No record before that date carries the key, so no earlier verdict moves.
    scen = metrics.get("scenario")
    if isinstance(scen, dict) and scen.get("functional_only"):
        reasons.append(
            f"flown on the functional rail ({scen.get('physics') or 'functional'}): "
            f"tracked as functional evidence, never KPI-grade - the lab's choice to "
            f"take KPI-grade runs from hil only (2026-11-17, WP4-04), not the grant's "
            f"wording")

    # Setpoints the Shield published that the MAVLink adapter did not hand to
    # MAVROS (2026-11-17; guardrail/delivery.py): the run's KPIs are computed
    # on what the Shield published, so a stall (a gap) or a shortfall would
    # count commands that never left the rail (hil ntFIN2_175: 245 of them,
    # graded KPI-grade until this date). The ROS 2 rail's metrics.json
    # `setpoint_delivery` carries it since 2026-11-17; for a stored run flown
    # before, the regrading tools fill the key from the run folder's logs
    # (guardrail.delivery.with_stored_delivery). No other rail has an
    # adapter, so its verdicts do not move.
    reasons.extend(delivery_grade_reasons(metrics.get("setpoint_delivery")))

    return (not reasons), reasons


# --------------------------------------------------------------------------- #
# The stack's revisions, host by host (ARCH-31)
# --------------------------------------------------------------------------- #
#
# The grant's determinism contract names ONE code_revision, a "single git SHA
# across the entire stack" (03-simulation/layer-requirements.md). A hil run
# spans hosts: the Shield and the VLA on the Orin (a git checkout synced by
# bundle, deploy/orin/sync_follow_controller.ps1), the harness, SITL's helper
# and the stressors on the desktop, and ArduPilot's own firmware. manifest.json
# carries one code_revision - the revision of the host that wrote it - and
# nothing in it said whether the other hosts ran the same commit, nor which
# firmware build flew.
#
# manifest.json stays the grant's six fields (the reference's
# DeterminismManifest, and every reader here, depend on exactly six). The
# per-host record goes BESIDE it, as the rest of a run's evidence does
# (metrics.json hil_evidence, manifest_extras.json):
#
#   stack_revisions.json   schema "vlaguard-stack-revisions/1"
#     code_revision    manifest.json's own field, repeated for the reader
#     topology         manifest.json's topology, which decides the hosts the
#                      stack needs (required_hosts: hil and flight need the
#                      companion AND the desktop, dev and the single-host
#                      rails the desktop)
#     hosts            {role: host_revision()}: "companion" (a Jetson, by its
#                      device tree) or "desktop", each with its code_revision
#                      (git rev-parse + the porcelain dirty flag, the rule
#                      code_revision() applies), `git describe`, hostname,
#                      the role the writer read and why (role_basis), and
#                      optional `components` (another checkout on that host,
#                      e.g. the WSL helper's) checked like hosts. A second
#                      host claiming a role already held is kept beside it
#                      ("desktop#2"), never written over it
#     autopilot        the firmware as the autopilot names itself
#                      (AUTOPILOT_VERSION: "ArduCopter V4.5.7 (2a3dc4b7)" on
#                      the pinned SITL), its version and 8-hex git hash, or
#                      None when it never answered
#     simulator        what is known of the simulator build (ArduPilot SITL:
#                      the autopilot's own binary; Project AirSim: the client
#                      package and the scene file's digest), or None
#     single_sha       True only when every host the topology needs has
#                      reported, no role is claimed twice, and every host
#                      and component runs the manifest's commit on a clean
#                      tree; False when one that reported does not; None
#                      (pending) while a needed host has not reported or
#                      the topology's hosts are not known. The Orin's own
#                      write on hil is therefore None until the desktop
#                      joins: one host never speaks for the stack
#     problems         why it is not True, in words (stack_problems)
#
# RUN MANIFEST SCHEMA (load_run_manifest)
#   1  manifest.json alone: every run written before 2026-11-17, and any run
#      whose writer predates this record. It loads unchanged; its only
#      revision is the manifest's, of the host that wrote it.
#   2  manifest.json (unchanged) + stack_revisions.json beside it.
#
# The record is EVIDENCE, not a gate: is_kpi_grade is unchanged, so no run's
# grade moves because of it. Since 2026-11-21 tools/kpi_report.py reads it
# (stack_summary) and reports single_sha true / false / pending / missing
# per campaign and over the KPI-grade runs - reported, never gated.

STACK_FILE = "stack_revisions.json"
STACK_SCHEMA = "vlaguard-stack-revisions/1"
RUN_MANIFEST_SCHEMA = 2
ROLE_DESKTOP = "desktop"
ROLE_COMPANION = "companion"
# The hosts each topology's stack runs our code on, so the hosts whose
# revisions one SHA across the stack must cover. hil and flight: the Shield
# and the VLA on the companion, the harness / SITL's helper / the stressors
# (or the ground side) on the desktop. dev and the single-host rails: the
# desktop alone. The remote-sim Project AirSim rail: the controller on the
# companion, the simulator and its scene on the desktop. A topology not
# listed here has no known host set, and single_sha stays None for it.
STACK_REQUIRED_ROLES: dict[str, tuple[str, ...]] = {
    TOPOLOGY_HIL: (ROLE_COMPANION, ROLE_DESKTOP),
    TOPOLOGY_FLIGHT: (ROLE_COMPANION, ROLE_DESKTOP),
    TOPOLOGY_DEV: (ROLE_DESKTOP,),
    TOPOLOGY_PROJECTAIRSIM: (ROLE_DESKTOP,),
    TOPOLOGY_PROJECTAIRSIM_REMOTE: (ROLE_COMPANION, ROLE_DESKTOP),
    TOPOLOGY_ARDUPILOT_SITL: (ROLE_DESKTOP,),
}
# "ArduCopter V4.5.7 (2a3dc4b7)" / "ArduCopter V4.8.0-dev (5f119834)"
_FIRMWARE_RE = re.compile(r"V(\d+\.\d+\.\d+[^\s(]*)(?:\s*\(([0-9a-f]{8})\))?")


def git_describe(repo: str | Path | None = None) -> str | None:
    """`git describe --always --tags --abbrev=12` of the repository that
    code_revision() names (the same candidates, the same ownership rule).
    No --dirty: the dirty flag of record is code_revision's, which also
    counts untracked files; describe would not. None when git cannot say."""
    for cand in _code_candidates(repo):
        try:
            r = subprocess.run(["git", "-C", str(cand), "describe", "--always",
                                "--tags", "--abbrev=12"],
                               capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.SubprocessError):
            continue
        if r.returncode != 0 or not _tracks_our_code(cand):
            continue
        return r.stdout.strip() or None
    return None


def _dirty_flag(rev: str | None) -> bool | None:
    """code_revision's suffix as a flag: True "-dirty", False clean, None
    when git could not tell ("-unknown") or nothing resolved."""
    rev = str(rev or "")
    if not rev or rev in ("unversioned", UNRESOLVED) or rev.endswith("-unknown"):
        return None
    return rev.endswith("-dirty")


def _revision_base(rev: str | None) -> str | None:
    rev = str(rev or "")
    if not rev or rev in ("unversioned", UNRESOLVED):
        return None
    for suffix in ("-dirty", "-unknown"):
        if rev.endswith(suffix):
            return rev[: -len(suffix)]
    return rev


def stack_required_roles(topology: str | None) -> tuple[str, ...] | None:
    """The host roles a run of `topology` must hear from before one SHA
    across the stack can be claimed (STACK_REQUIRED_ROLES), or None when
    the topology's hosts are not known. A legacy label reads as its
    current one (LEGACY_TOPOLOGY_LABELS)."""
    t = LEGACY_TOPOLOGY_LABELS.get(str(topology), topology)
    return STACK_REQUIRED_ROLES.get(t) if t else None


def host_role_basis(host: dict | None) -> tuple[str, str]:
    """(role, why): "companion" on a Jetson (its device tree or L4T release
    says so), else "desktop", with the evidence that decided it. Read from
    the host's evidence, never declared."""
    host = host or {}
    raw_model = host.get("device_model")
    model = str(raw_model or "").lower()
    if "jetson" in model:
        return ROLE_COMPANION, f"device tree model {raw_model!r}"
    if host.get("l4t_release"):
        return ROLE_COMPANION, f"L4T release {host.get('l4t_release')!r}"
    why = "no Jetson device tree model and no L4T release in the host evidence"
    if str(host.get("host_arch") or "").lower() in ("aarch64", "arm64"):
        # The Orin's container without the /run/vlaguard/host mount (which
        # docker-compose.orin.yml adds because AppArmor blocks /proc's path)
        # reads exactly like this.
        why += (f" (host_arch {host.get('host_arch')!r}: possibly a companion "
                f"whose device-tree mount is missing)")
    return ROLE_DESKTOP, why


def host_role(host: dict | None) -> str:
    """"companion" on a Jetson (its device tree or L4T release says so),
    else "desktop" (host_role_basis says why)."""
    return host_role_basis(host)[0]


def host_revision(*, host: dict | None = None, revision: str | None = None,
                  repo: str | Path | None = None,
                  describe: Callable[[str | Path | None], str | None] = git_describe
                  ) -> dict[str, Any]:
    """This host's own revision: code_revision (rev-parse + dirty flag; pass
    `revision` to reuse one already read, such as the manifest's), `git
    describe`, which machine it is (collect_host_evidence's fields), and
    the role it read itself as, with why (host_role_basis)."""
    host = collect_host_evidence() if host is None else host
    rev = code_revision(repo) if revision is None else revision
    role, why = host_role_basis(host)
    return {"hostname": host.get("hostname"),
            "device_model": host.get("device_model"),
            "host_arch": host.get("host_arch"),
            "role": role,
            "role_basis": why,
            "code_revision": rev,
            "dirty": _dirty_flag(rev),
            "git_describe": describe(repo),
            "source": "git in this host's own checkout (guardrail.manifest."
                      "code_revision, git describe)"}


def autopilot_revision(evidence: dict | None) -> dict[str, Any] | None:
    """The firmware that flew, as the autopilot named itself (the evidence's
    `ardupilot_version`, read from AUTOPILOT_VERSION or, failing that, the
    SITL binary's banner - `ardupilot_version_source` says which). None when
    nothing named it: never filled in from what setup_sitl.sh pins."""
    ev = evidence or {}
    fw = ev.get("ardupilot_version")
    if not fw:
        return None
    m = _FIRMWARE_RE.search(str(fw))
    git_hash = ((m.group(2) if m else None) or ev.get("git_hash")
                or decode_custom_version(ev.get("flight_custom_version_raw")))
    return {"firmware": str(fw),
            "version": m.group(1) if m else None,
            "git_hash": git_hash,
            "flight_sw_version": ev.get("flight_sw_version"),
            "kind": ev.get("autopilot_kind"),
            "source": ev.get("ardupilot_version_source"),
            **({"pin_ok": ev["ardupilot_pin_ok"]} if "ardupilot_pin_ok" in ev else {})}


def sitl_simulator(autopilot: dict | None) -> dict[str, Any] | None:
    """ArduPilot SITL as the simulator: one arducopter binary runs the
    firmware and the simulated vehicle, so its build is the autopilot's.
    Only when the autopilot showed it is SITL (`kind` "sitl": SIM_SPEEDUP
    answered); the physics backend (start_sitl.sh --model: the built-in
    quad, or JSON for Gazebo) is not reported over MAVLink."""
    if not autopilot or autopilot.get("kind") != "sitl":
        return None
    return {"name": "ArduPilot SITL",
            "build": autopilot.get("firmware"),
            "git_hash": autopilot.get("git_hash"),
            "build_source": "the autopilot's own report (the SITL binary is "
                            "the autopilot build)",
            "physics_model": UNRESOLVED,
            "physics_model_why": "SITL's --model is not reported over MAVLink"}


def projectairsim_simulator(scene_path: str | Path,
                            version: Callable[[str], str] | None = None
                            ) -> dict[str, Any]:
    """Project AirSim as the simulator: the client package's version (as
    installed in this interpreter) and the scene file's SHA-256. The
    simulator binary's own build is not asked for by any rail, so it is
    recorded as unresolved, never guessed."""
    if version is None:
        from importlib.metadata import version as installed_version
        version = installed_version
    try:
        client = version("projectairsim")
    except Exception:                                      # noqa: BLE001
        client = None
    p = Path(scene_path)
    try:
        scene_sha = hashlib.sha256(p.read_bytes()).hexdigest()
    except OSError:
        scene_sha = None
    return {"name": "Project AirSim",
            "client_version": client,
            "scene": p.name,
            "scene_sha256": scene_sha,
            "server_build": UNRESOLVED,
            "server_build_why": "the rail does not ask the simulator binary "
                                "for its build"}


def _base_role(key: str) -> str:
    """"desktop#2" -> "desktop": the role a duplicate entry claimed."""
    return str(key).split("#", 1)[0]


def _revision_checks(doc: dict) -> list[tuple[str, str | None]]:
    """(label, code_revision) of every host and every component on a host
    (another checkout of ours that host runs, e.g. the WSL helper's)."""
    out: list[tuple[str, str | None]] = []
    for key, rec in sorted((doc.get("hosts") or {}).items()):
        rec = rec or {}
        out.append((key, rec.get("code_revision")))
        for name, comp in sorted((rec.get("components") or {}).items()):
            out.append((f"{key}/{name}", (comp or {}).get("code_revision")))
    return out


def _sha_mismatch(doc: dict) -> bool:
    """True when what DID report already rules one clean SHA out: the
    manifest named no revision, or a host or component runs another
    commit, a dirty tree, or one git could not resolve."""
    want = _revision_base(doc.get("code_revision"))
    if want is None:
        return True
    return any(_revision_base(rev) != want or _dirty_flag(rev) is not False
               for _, rev in _revision_checks(doc))


def stack_problems(doc: dict) -> list[str]:
    """Why a stack record does not show one clean commit across the stack.
    Empty = every host the topology needs reported, each once, every host
    and component runs the manifest's commit on a clean tree, and the
    autopilot named its firmware."""
    out: list[str] = []
    want = _revision_base(doc.get("code_revision"))
    if want is None:
        out.append(f"the manifest's code_revision did not resolve "
                   f"({doc.get('code_revision')!r})")
    for label, rev in _revision_checks(doc):
        base = _revision_base(rev)
        if base is None:
            out.append(f"{label}: code_revision did not resolve ({rev!r})")
            continue
        if want is not None and base != want:
            out.append(f"{label} runs {rev!r}, the manifest names "
                       f"{doc.get('code_revision')!r}")
        if _dirty_flag(rev) is not False:
            out.append(f"{label}: {rev!r} is not a clean tree")
    hosts = doc.get("hosts") or {}
    if not hosts:
        out.append("no host reported its revision")
    topology = doc.get("topology")
    required = stack_required_roles(topology)
    if required is None:
        out.append(f"topology {topology!r}: the hosts its stack runs on are not "
                   f"known, so one SHA across the stack cannot be shown")
    else:
        present = {_base_role(k) for k in hosts}
        for role in required:
            if role not in present:
                out.append(f"{role} has not reported its revision (topology "
                           f"{topology!r} needs {', '.join(required)})")
    for key, rec in sorted(hosts.items()):
        if "#" not in key:
            continue
        first = hosts.get(_base_role(key)) or {}
        out.append(f"{key}: a second host reported as {_base_role(key)} "
                   f"({(rec or {}).get('hostname')!r}, "
                   f"{(rec or {}).get('device_model')!r}) beside "
                   f"{first.get('hostname')!r} ({first.get('device_model')!r}); "
                   f"one of them misread its role")
    if not doc.get("autopilot"):
        out.append("the autopilot did not name its firmware")
    return out


def _finish_stack(doc: dict) -> dict:
    """single_sha and problems, from the record as it stands. False when what
    reported rules one SHA out; None (pending) while a host the topology
    needs has not reported, a role is claimed twice, or the topology's hosts
    are not known; True only when none of that holds. A single host's write
    on a multi-host topology is never True on its own."""
    hosts = doc.get("hosts") or {}
    required = stack_required_roles(doc.get("topology"))
    present = {_base_role(k) for k in hosts}
    doc["required_hosts"] = list(required) if required is not None else None
    if _sha_mismatch(doc):
        doc["single_sha"] = False
    elif (not hosts or required is None
          or any(r not in present for r in required)
          or any("#" in k for k in hosts)):
        doc["single_sha"] = None
    else:
        doc["single_sha"] = True
    doc["problems"] = stack_problems(doc)
    return doc


def stack_document(*, code_revision: str, hosts: dict[str, dict],
                   autopilot: dict | None,
                   simulator: dict | None,
                   topology: str | None = None) -> dict[str, Any]:
    """The stack_revisions.json document (schema STACK_SCHEMA). `topology`
    is the manifest's: it names the hosts the stack needs."""
    return _finish_stack({"schema": STACK_SCHEMA,
                          "run_manifest_schema": RUN_MANIFEST_SCHEMA,
                          "code_revision": code_revision,
                          "topology": topology,
                          "hosts": dict(hosts),
                          "autopilot": autopilot,
                          "simulator": simulator})


def _write_stack(out_dir: Path, doc: dict) -> Path:
    p = Path(out_dir) / STACK_FILE
    p.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8", newline="\n")
    return p


def write_stack_revisions(out_dir: str | Path, manifest: dict, evidence: dict | None,
                          *, simulator: dict | None = None,
                          describe: Callable[[str | Path | None], str | None] = git_describe,
                          warn: Callable[[str], None] | None = None
                          ) -> dict[str, Any] | None:
    """Write stack_revisions.json beside a run's manifest.json: the writing
    host's revision (the manifest's own code_revision, its role read from
    the host evidence in `evidence`, else collected here), the autopilot's
    firmware from the same evidence, and the simulator (`simulator`, else
    ArduPilot SITL when the autopilot is SITL). Best effort: a failure is
    warned and returns None, never ends the run's records."""
    try:
        ev = evidence or {}
        host = ({k: ev.get(k) for k in ("hostname", "device_model", "host_arch",
                                        "l4t_release")}
                if ev.get("hostname") else collect_host_evidence())
        auto = autopilot_revision(ev)
        doc = stack_document(
            code_revision=manifest["code_revision"],
            hosts={host_role(host): host_revision(
                host=host, revision=manifest["code_revision"], describe=describe)},
            autopilot=auto,
            simulator=simulator if simulator is not None else sitl_simulator(auto),
            topology=manifest.get("topology"))
        _write_stack(Path(out_dir), doc)
        return doc
    except Exception as e:                                # noqa: BLE001
        if warn is not None:
            warn(f"{STACK_FILE} not written: {type(e).__name__}: {e}")
        return None


def add_host_revision(run_dir: str | Path, role: str, record: dict) -> dict[str, Any]:
    """Add one host's revision to a run's stack record, after the run: the
    hil harness adds the desktop's once it has fetched the Orin's run folder.
    A run whose writer predates the record (no stack_revisions.json) gets
    one, from its manifest's code_revision and its metrics' hil_evidence;
    the writing host is then not listed, because only it could say which it
    was.

    A role already held by ANOTHER machine (a different hostname or device
    model) is never written over: the new record goes beside it as
    "<role>#2" (#3, ...), and the record's problems name the clash, so a
    companion that misread itself as the desktop cannot be replaced by the
    real desktop and leave one host speaking for the stack. The same
    machine adding itself again replaces its own entry."""
    run_dir = Path(run_dir)
    p = run_dir / STACK_FILE
    if p.is_file():
        doc = json.loads(p.read_text(encoding="utf-8"))
        if doc.get("schema") != STACK_SCHEMA:
            raise ValueError(f"{p}: schema {doc.get('schema')!r}, not {STACK_SCHEMA!r}")
    else:
        def _read(name: str) -> dict:
            try:
                return json.loads((run_dir / name).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return {}
        # No manifest.json (a run that ended before writing one): the record
        # says the manifest named no revision, and stack_problems says so.
        man, met = _read("manifest.json"), _read("metrics.json")
        auto = autopilot_revision(met.get("hil_evidence"))
        doc = stack_document(code_revision=man.get("code_revision"), hosts={},
                             autopilot=auto, simulator=sitl_simulator(auto),
                             topology=man.get("topology"))
    hosts = doc.setdefault("hosts", {})
    key, n = role, 1

    def _machine(rec: dict | None) -> tuple:
        rec = rec or {}
        return (rec.get("hostname"), rec.get("device_model"))
    while key in hosts and _machine(hosts[key]) != _machine(record):
        n += 1
        key = f"{role}#{n}"
    hosts[key] = record
    _finish_stack(doc)
    _write_stack(run_dir, doc)
    return doc


def load_run_manifest(run_dir: str | Path) -> dict[str, Any]:
    """A run's manifest under either run manifest schema: {"run_manifest_schema":
    1 or 2, "manifest": manifest.json's six fields, "stack": the stack record
    or None}. A schema-1 run (no stack_revisions.json) loads as it always did;
    a stack record of an unknown schema is refused, never half-read."""
    run_dir = Path(run_dir)
    man = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
    p = run_dir / STACK_FILE
    if not p.is_file():
        return {"run_manifest_schema": 1, "manifest": man, "stack": None}
    doc = json.loads(p.read_text(encoding="utf-8"))
    if doc.get("schema") != STACK_SCHEMA:
        raise ValueError(f"{p}: schema {doc.get('schema')!r}, not {STACK_SCHEMA!r}")
    return {"run_manifest_schema": int(doc.get("run_manifest_schema", RUN_MANIFEST_SCHEMA)),
            "manifest": man, "stack": doc}


def revisions_by_host(loaded: dict) -> dict[str, str | None]:
    """{role: code_revision} of a load_run_manifest result. A schema-1 run
    has one, under "manifest" (the host that wrote manifest.json, which the
    record does not name)."""
    stack = loaded.get("stack")
    if not stack:
        return {"manifest": loaded["manifest"].get("code_revision")}
    return {role: (rec or {}).get("code_revision")
            for role, rec in (stack.get("hosts") or {}).items()}


# What stack_summary() calls each single_sha value, and the two states a run
# without a usable record is in. "missing" is every run written before
# 2026-11-17 (or by a writer that predates the record); "unreadable" is a
# record that is not JSON, of another schema, or carries no single_sha.
STACK_STATES = ("true", "false", "pending", "missing", "unreadable")


def stack_summary(run_dir: str | Path) -> dict[str, Any]:
    """One run's stack record as a KPI report quotes it (ARCH-31; read by
    tools/kpi_report.py since 2026-11-21, until then nothing read the
    record). `state` is "true" / "false" / "pending" for single_sha True /
    False / None AS WRITTEN, "missing" when the run has no
    stack_revisions.json, "unreadable" when it is not JSON, is of another
    schema or has no single_sha key (`why` says which). Beside it the
    revisions the record names: `companion` and `desktop` (each host of that
    base role, a second claimant "desktop#2" included), `components`
    ({"desktop/wsl_helper": revision}, another checkout a host runs),
    `autopilot` (the firmware as it named itself, None when it never
    answered) and `problems` as written.

    Read only: the record is never rewritten and single_sha is never
    recomputed here - a report says what the run recorded. is_kpi_grade does
    not read it either (the record is evidence, not a gate): a gate keyed on
    single_sha would regrade every run flown since 2026-11-17, whose records
    already carry the field."""
    p = Path(run_dir) / STACK_FILE
    out: dict[str, Any] = {"state": "missing", "single_sha": None,
                           "companion": [], "desktop": [], "components": {},
                           "autopilot": None, "problems": []}
    if not p.is_file():
        out["why"] = f"no {STACK_FILE} beside the manifest"
        return out
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        return dict(out, state="unreadable",
                    why=f"{STACK_FILE} is not readable JSON ({type(e).__name__})")
    if not isinstance(doc, dict) or doc.get("schema") != STACK_SCHEMA:
        got = doc.get("schema") if isinstance(doc, dict) else type(doc).__name__
        return dict(out, state="unreadable",
                    why=f"{STACK_FILE} schema {got!r}, not {STACK_SCHEMA!r}")
    if "single_sha" not in doc or doc["single_sha"] not in (True, False, None):
        return dict(out, state="unreadable",
                    why=f"{STACK_FILE} carries no single_sha verdict "
                        f"({doc.get('single_sha')!r})")
    ss = doc["single_sha"]
    out.update(state={True: "true", False: "false", None: "pending"}[ss],
               single_sha=ss)
    by_role: dict[str, set] = {ROLE_COMPANION: set(), ROLE_DESKTOP: set()}
    hosts = doc.get("hosts") if isinstance(doc.get("hosts"), dict) else {}
    for key, rec in sorted(hosts.items()):
        rec = rec if isinstance(rec, dict) else {}
        by_role.setdefault(_base_role(key), set()).add(str(rec.get("code_revision")))
        comps = rec.get("components") if isinstance(rec.get("components"), dict) else {}
        for name, comp in sorted(comps.items()):
            out["components"][f"{key}/{name}"] = (comp or {}).get("code_revision")
    out["companion"] = sorted(by_role.pop(ROLE_COMPANION))
    out["desktop"] = sorted(by_role.pop(ROLE_DESKTOP))
    if any(by_role.values()):
        # A role no topology needs (never written by this repository's
        # writers): kept, under its own name, never dropped.
        out["other_hosts"] = {r: sorted(v) for r, v in sorted(by_role.items()) if v}
    auto = doc.get("autopilot")
    out["autopilot"] = (auto.get("firmware") if isinstance(auto, dict) else None)
    out["problems"] = list(doc.get("problems") or [])
    return out


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
        # The key a bundle's record carries (guardrail.bundle source_record,
        # 2026-11-16): no signer, so no certificate chain.
        rec["chain"] = "none"
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


def resolve_run_generation_chain(run_dir, recorded: str | None, candidates) -> str | None:
    """A label for a run whose policy changed in flight over REST, or None.

    Such a run (the 29 Oct ROS 2 rail flights: a zone spawned, rotated, moved,
    switched off) records the LAST generation's hash, which no policy file or
    lock recipe rebuilds: the events came from a client, not a file. The run
    writes every generation's policy as policy_g<N>.json. It resolves when
    policy_g0.json resolves to a candidate (resolve_run_policy: it started
    from a known policy) and one of its own generations hashes to `recorded`.
    The chain is read without the flight loaders' runtime gate (a REST zone
    is a dynamic_nfz, and one that moves may not fly from a policy FILE,
    C-16; one without motion may since 2026-11-15)."""
    if not recorded:
        return None
    from guardrail.models import era_context, policy_from_raw
    run_dir = Path(run_dir)

    def gen(f: Path) -> int:
        tail = f.stem[len("policy_g"):]
        return int(tail) if tail.isdigit() else -1
    gens = sorted(run_dir.glob("policy_g*.json"), key=gen)
    if not gens or gens[0].name != "policy_g0.json":
        return None
    try:
        # A run's policy_g<N>.json records no IR schema version: read as
        # written, so a run flown before IR schema 4.0.0, whose rules leave
        # `scope` out, resolves as it did (models.era_context).
        chain = [policy_from_raw(json.loads(f.read_text(encoding="utf-8")), source=f.name,
                                 runtime=False, context=era_context(None)) for f in gens]
    except Exception:                                          # noqa: BLE001
        return None
    base, _, label = resolve_run_policy({"policy_hash": chain[0].policy_hash}, candidates)
    if base is None:
        return None
    for f, pol in zip(gens, chain):
        if pol.matches_hash(recorded):
            return f"{label} -> {f.stem} (generations recorded by the run)"
    return None
