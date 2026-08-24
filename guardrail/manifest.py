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
import json
import os
import re
import subprocess
from pathlib import Path
from typing import Any

UNRESOLVED = "unresolved"

# Our rail is NOT the grant's canonical HIL topology (ArduPilot SITL + MAVROS 2 on
# the dev compose). The field exists so the difference can never be blurred, so
# the value the flight scripts pass is fixed and this module refuses the HIL name.
TOPOLOGY_PROJECTAIRSIM = "projectairsim-single-host"
TOPOLOGY_CANONICAL_HIL = "canonical-hil"
# ArduPilot SITL driven straight over pymavlink (sitl/run_sitl_demo.py). Real
# flight code and a real MAVLink path, but NOT the grant's canonical topology,
# which also requires MAVROS 2. Naming it separately keeps the difference from
# being blurred in either direction: these runs are not Project AirSim, and they
# are not yet KPI-grade either.
TOPOLOGY_ARDUPILOT_SITL = "ardupilot-sitl-pymavlink"

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


def model_hash(model_id: str, weight_paths: list[str | Path] | None = None) -> str:
    """Identify the model that actually ran.

    Hashes the weight FILES when they are on disk, because a HuggingFace id alone
    does not pin a revision and an adapter directory can be edited in place. Falls
    back to the id plus "unresolved" rather than pretending the id is a hash.
    """
    rev = hf_snapshot_revision(model_id)
    if rev and not weight_paths:
        return f"{model_id}@{rev[:16]}"

    h = hashlib.sha256()
    hashed_any = False
    if rev:
        h.update(rev.encode())          # pin the base model as well as the adapter
    for p in (weight_paths or []):
        p = Path(p)
        files = sorted(p.rglob("*")) if p.is_dir() else ([p] if p.is_file() else [])
        for f in files:
            if not f.is_file() or f.suffix in (".log", ".jsonl"):
                continue
            try:
                st = f.stat()
            except OSError:
                continue
            # Size + mtime + name, not the bytes: adapter directories run to
            # hundreds of MB and this is a determinism check, not an integrity one.
            h.update(f.name.encode())
            h.update(str(st.st_size).encode())
            h.update(str(int(st.st_mtime)).encode())
            hashed_any = True
    if not hashed_any:
        return f"{model_id}@{UNRESOLVED}"
    return f"{model_id}@sha256:{h.hexdigest()[:16]}"


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
                   sim_speedup: float | None = None) -> dict[str, Any]:
    """The six fields, in the grant's order and under the grant's names.

    `sim_speedup` may be passed only for a rail with no scene file, and only
    with a value produced by a derivation helper such as
    `sim_speedup_from_mavlink()`. Never pass a literal: the whole reason this
    field is derived rather than declared is that a hand-written 1.0 is exactly
    the number a broken run would also carry.
    """
    if topology == TOPOLOGY_CANONICAL_HIL:
        # Guard, not politeness. Our flights do not run the canonical topology and
        # a number labelled as though they did would misreport the grant.
        raise ValueError(
            "canonical-hil is the grant's ArduPilot SITL + MAVROS topology; "
            "no rail here runs MAVROS 2 and none may claim it")
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


def is_kpi_grade(manifest: dict, metrics: dict) -> tuple[bool, list[str]]:
    """May this run's numbers be quoted as contractual KPIs?

    Every check below exists because the corresponding failure already happened
    and produced a number that looked fine.
    """
    reasons: list[str] = []

    if manifest.get("sim_speedup") != 1.0:
        reasons.append(
            f"sim_speedup is {manifest.get('sim_speedup')}, and 1.0 is mandatory "
            f"for a KPI-bearing run")

    if manifest.get("topology") != TOPOLOGY_CANONICAL_HIL:
        reasons.append(
            f"topology is {manifest.get('topology')!r}, not the grant's "
            f"{TOPOLOGY_CANONICAL_HIL!r}: functional-rail evidence, not a "
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
