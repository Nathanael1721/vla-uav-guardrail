"""WP1's acceptance KPI, recorded: policy load round-trip, and what it is worth.

    python tools/wp1_roundtrip_kpi.py                 # writes docs/data/wp1_roundtrip.json
    python tools/wp1_roundtrip_kpi.py --out x.json    # anywhere else
    python tools/wp1_roundtrip_kpi.py --no-runs       # skip the stored-run census
    python tools/wp1_roundtrip_kpi.py --no-cold-start # skip the cold-start timing
    # from a git worktree, reading the main checkout's gitignored runs and
    # reference repository (read-only):
    python tools/wp1_roundtrip_kpi.py --runs-root <checkout> \
        --reference-repo <checkout>/kuanting-vla-uav-guardrail

WHAT THE GRANT ASKS FOR

WP1 is accepted on "Policy load round-trip; bundle replayability" (Grant
overview, WP table). Until 2026-10-06 the round trip worked for every policy
and no script recorded it, so no report could quote it (audit card WP1-17).

WHY A ROUND-TRIP NUMBER ALONE IS WORTH NOTHING

A loader that accepts everything and a bundle reader that checks nothing both
score N/N on "load -> bundle -> load gives the same policy": nothing in between
ever said no. So this tool reports the round trip beside the two numbers that
give it meaning, and states the zero-skill null outright:

  round_trip       every policies/*.yaml: load -> write_bundle -> check_bundle
                   gives the same policy_hash, the same canonical IR and the
                   same full model dump;
  tamper_refusal   for each of those bundles, one rule field is altered in the
                   archived IR and the reader must refuse it;
  negative_corpus  policies/negative/*.yaml are deliberately broken policies
                   (misspelled key, bow-tie polygon, duplicate rule id, ...)
                   that the DSL must refuse. They are loaded with
                   `runtime=False`, i.e. through the DSL's own validation and
                   lint, so a case is never "refused" merely because the flight
                   gate turned away a declarable-only rule. Any case ACCEPTED
                   is a finding, listed by name. 14/25 were refused on
                   2026-10-05; since 2026-10-06 (strict validation) the
                   original 25 slots are 25/25 (2 replaced) and 10 cases were
                   added, 35/35. Those are two corpora: the 2026-10-05 loader
                   refuses 20/35 of today's.

  null "accept everything": round_trip N/N, tamper_refusal 0/N, negative 0/M.
  null "the 2026-10-05 code": the same columns for 401305a (`null_2026_10_05`).

Two sections added on 2026-10-06 measure the grant's own form:

  grant_form       every policies/*.yaml -> Policy.to_grant_form() -> load ->
                   to_grant_form() again: the two documents must be identical
                   and every rule the same (geometry within 1 um). A metre
                   policy with no origin is anchored at DOC_ANCHOR for this,
                   because the grant's form is WGS84 and needs one.
  reference        the reference implementation's own documents and bundle
                   (kuanting-vla-uav-guardrail/bundles): ours must load each,
                   and its `reference_hash()` must equal the hash the
                   reference computes. Where the reference's 3.11 virtualenv
                   exists its loader is RUN (read-only) for that number, and it
                   is also asked to read a bundle this code wrote. Null: the
                   loader as of 2026-10-05 loaded 0 of the 3 documents and 0 of
                   the 1 bundle (it refused issued_at, the geometry block,
                   project_fix and the missing origin).

One section added on 2026-10-08 (deviation 10, card WP1-05):

  both_hashes      every policies/*.yaml bundled: its manifest names the policy
                   under BOTH canonical forms (this project's and the
                   reference implementation's, null with the reasons where
                   the reference cannot hold it), check_bundle verifies both,
                   and `policy_hash` is still this project's (no stored hash
                   moved). Each bundle is then forged with a wrong reference
                   hash and must be refused. Null, MEASURED: the same forged
                   bundles read by a loader that ignores the record (what
                   10451bc did - it wrote none and read none) are all accepted.
                   The reference section also reads each reference-form
                   bundle's source record back (this project's hash of the
                   policy its delta rebuilds).

One section added on 2026-10-19 (the audit's "KPI WP1 ... cold-start <= 1 s
... belum dilaporkan"):

  cold_start       the grant's "cold-start ingest of a typical 50-rule bundle
                   <= 1 s" (Policy DSL page): a signed 50-rule bundle on disk
                   -> load_for_flight (read, parse, validate, hash, lint,
                   Ed25519 verify, flight gate) -> rail_shield (the runtime
                   IR), in this process and in fresh interpreters (imports
                   included), for a typical mixed WGS84 bundle and for the
                   bench's 50-rule grid. Every timed run must prove it did the
                   work (verified signature, issued hash, 50 rules, every zone
                   compiled, a violation raised inside a fence) and the same
                   path must refuse a tampered and an untrusted bundle, or
                   `within_budget` is False however fast it was. Null: only
                   Shield construction (4.7 ms) had been timed.

Two identity checks ride along because they are the same question asked of
stored artefacts: the JSON Schema export is current, and every stored flight
manifest still resolves to a policy (the census, before and after the
2026-10-06 identity release). Every number printed here is the number written.
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import io
import json
import math
import os
import platform
import statistics
import sys
import tarfile
import tempfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.bundle import (HAVE_CRYPTO, IR_NAME, SCHEMA_PATH,       # noqa: E402
                              check_bundle, check_lock, policy_candidates,
                              policy_json_schema, verifier_name,
                              write_bundle)
from guardrail.fsm import canonical_action                          # noqa: E402
from guardrail.manifest import code_revision, resolve_run_policy     # noqa: E402
from guardrail.models import (HASH_SCHEME, REFERENCE_FORM,           # noqa: E402
                              load_policy, policy_from_raw)

DEFAULT_OUT = ROOT / "docs" / "data" / "wp1_roundtrip.json"
NEGATIVE_DIR = ROOT / "policies" / "negative"
REFERENCE_DIRNAME = "kuanting-vla-uav-guardrail"
REFERENCE_REPO = ROOT / REFERENCE_DIRNAME
REFERENCE_PY = REFERENCE_REPO / ".venv" / "Scripts" / "python.exe"
# The WGS84 anchor a metre policy without an origin gets for the grant-form
# round trip only: policies/wgs84_taipei.yaml's origin, near NTUT.
DOC_ANCHOR = (25.04245, 121.5314)
GEOMETRY_TOL_M = 1e-6


def say(key, value):
    print(f"  {key:44s} {value}")


# --------------------------------------------------------------------------- #
# round trip + tamper refusal
# --------------------------------------------------------------------------- #

def _tamper(src: Path, dst: Path) -> bool:
    """Rewrite the bundle with one numeric field of the first rule changed.

    Returns False when the policy has no numeric field to alter (then the
    case is reported as not run, never as refused)."""
    with tarfile.open(src, "r:gz") as tar:
        members = {m.name: tar.extractfile(m).read() for m in tar.getmembers()}
    ir = json.loads(members[IR_NAME])
    changed = False
    for rule in ir.get("constraints", []):
        for k, v in sorted(rule.items()):
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                rule[k] = v + 1.0
                changed = True
                break
        if changed:
            break
    if not changed:
        return False
    members[IR_NAME] = json.dumps(ir, sort_keys=True, separators=(",", ":")).encode()
    with tarfile.open(dst, "w:gz") as tar:
        for name, data in members.items():
            info = tarfile.TarInfo(name=name)
            info.size, info.mtime = len(data), 0
            tar.addfile(info, io.BytesIO(data))
    return True


def round_trip(policies_dir: Path, signer="auto") -> dict:
    files = sorted(policies_dir.glob("*.yaml"))
    if not files:
        # Zero policies would make every count below 0/0 - a pass by silence.
        raise SystemExit(f"no policies in {policies_dir}; refusing to report 0/0")
    tmp = Path(tempfile.mkdtemp(prefix="wp1rt_"))
    rows, sig = [], Counter()
    for f in files:
        row = {"file": f.name}
        try:
            pol = load_policy(f, runtime=False)
            b = write_bundle(pol, tmp / f"{f.stem}.tar.gz",
                             changelog="wp1 round-trip KPI",
                             issued_at="2026-01-01T00:00:00Z", signer=signer)
            chk = check_bundle(b)
            back = chk.policy
            row.update({
                "policy_id": pol.policy_id, "version": pol.version,
                "policy_hash": pol.policy_hash,
                "same_hash": back.policy_hash == pol.policy_hash,
                "same_canonical_ir": back.canonical_ir() == pol.canonical_ir(),
                "same_model_dump": back.model_dump() == pol.model_dump(),
                "same_rule_count": len(back.constraints) == len(pol.constraints),
                "signature": chk.signature,
            })
            row["ok"] = all(row[k] for k in ("same_hash", "same_canonical_ir",
                                             "same_model_dump", "same_rule_count"))
            sig[chk.signature] += 1
            bad = tmp / f"{f.stem}.tampered.tar.gz"
            if _tamper(b, bad):
                try:
                    check_bundle(bad)
                    row["tamper"] = "ACCEPTED"
                except ValueError as exc:
                    row["tamper"] = "refused"
                    row["tamper_reason"] = str(exc).split(" - ")[0][:120]
            else:
                row["tamper"] = "not run: no numeric field"
        except Exception as exc:                               # noqa: BLE001
            row.update({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        rows.append(row)
    n = len(rows)
    ok = sum(1 for r in rows if r.get("ok"))
    t_run = [r for r in rows if r.get("tamper") in ("refused", "ACCEPTED")]
    t_ref = sum(1 for r in t_run if r["tamper"] == "refused")
    return {"n": n, "passed": ok, "rate": round(ok / n, 4),
            "signatures": dict(sig),
            "tamper_refusal": {"run": len(t_run), "refused": t_ref,
                               "rate": round(t_ref / len(t_run), 4) if t_run else None},
            "rows": rows}


# --------------------------------------------------------------------------- #
# both hashes in every manifest (2026-10-08)
# --------------------------------------------------------------------------- #

_REASON_KINDS = (("metre vertices", "metre vertices"), ("margin_m", "margin ring"),
                 ("is not in the reference's DSL", "a rule type the reference lacks"),
                 ("valid_time", "a schedule (valid_time)"), ("`origin`", "origin"),
                 ("`layers_merged`", "layers_merged"), ("no rules", "no rules"))


def _reason_kind(why: str) -> str:
    for key, label in _REASON_KINDS:
        if key in why:
            return label
    return "other"


def _forge_reference_hash(src: Path, dst: Path) -> None:
    """The bundle with its manifest's reference-form hash replaced by a wrong
    one (the ir.json, signature line and `policy_hash` untouched)."""
    with tarfile.open(src, "r:gz") as tar:
        members = {m.name: tar.extractfile(m).read() for m in tar.getmembers()}
    man = json.loads(members["manifest.json"])
    man["hashes"][REFERENCE_FORM] = "sha256:" + "0" * 64
    members["manifest.json"] = json.dumps(man, indent=2, sort_keys=True).encode()
    with tarfile.open(dst, "w:gz") as tar:
        for name, data in members.items():
            info = tarfile.TarInfo(name=name)
            info.size, info.mtime = len(data), 0
            tar.addfile(info, io.BytesIO(data))


class _RecordIgnored:
    """The null reader: check_bundle with the 2026-10-08 hash record skipped,
    as 10451bc read every bundle (it wrote no record and read none)."""

    def __enter__(self):
        import guardrail.bundle as _b
        self._b, self._saved = _b, _b._verify_both_hashes
        _b._verify_both_hashes = lambda name, man, irb, pol, form, declared: (
            {form: declared}, None)
        return self

    def __exit__(self, *exc):
        self._b._verify_both_hashes = self._saved
        return False


def both_hashes(policies_dir: Path) -> dict:
    files = sorted(policies_dir.glob("*.yaml"))
    if not files:
        raise SystemExit(f"no policies in {policies_dir}; refusing to report 0/0")
    tmp = Path(tempfile.mkdtemp(prefix="wp1hash_"))
    rows, kinds = [], Counter()
    forged: list[tuple[str, Path]] = []
    for f in files:
        row = {"file": f.name}
        try:
            pol = load_policy(f, runtime=False)
            b = write_bundle(pol, tmp / f"{f.stem}.tar.gz", changelog="wp1 both hashes",
                             issued_at="2026-01-01T00:00:00Z", signer=None)
            chk = check_bundle(b)
            rec = chk.manifest.get("hashes") or {}
            ref = pol.reference_hash()
            row.update({
                "policy_hash": pol.policy_hash,
                "policy_hash_unchanged": chk.manifest.get("policy_hash") == pol.policy_hash,
                "both_recorded": set(rec) == {HASH_SCHEME, REFERENCE_FORM},
                "both_verified": chk.hashes == {HASH_SCHEME: pol.policy_hash,
                                                REFERENCE_FORM: ref},
                "reference_hash": ref})
            if ref is None:
                row["reference_problems"] = sorted(
                    {_reason_kind(w) for w in chk.manifest.get("reference_problems") or []})
                kinds.update(row["reference_problems"])
            row["ok"] = (row["policy_hash_unchanged"] and row["both_recorded"]
                         and row["both_verified"])
            bad = tmp / f"{f.stem}.forged.tar.gz"
            _forge_reference_hash(b, bad)
            forged.append((f.name, bad))
        except Exception as exc:                               # noqa: BLE001
            row.update({"ok": False, "error": f"{type(exc).__name__}: {exc}"[:300]})
        rows.append(row)

    def refusals() -> dict:
        out = {}
        for name, bad in forged:
            try:
                check_bundle(bad)
                out[name] = "ACCEPTED"
            except ValueError:
                out[name] = "refused"
        return out
    verdict = refusals()
    with _RecordIgnored():
        null = refusals()
    for r in rows:
        r["forged_reference_hash"] = verdict.get(r["file"], "not run")
    n = len(rows)
    return {"n": n, "passed": sum(1 for r in rows if r.get("ok")),
            "representable": sum(1 for r in rows if r.get("reference_hash")),
            "not_representable_because": dict(sorted(kinds.items())),
            "forged_refused": sum(1 for v in verdict.values() if v == "refused"),
            "forged_run": len(verdict),
            "null_record_ignored_refused": sum(1 for v in null.values() if v == "refused"),
            "rows": rows}


# --------------------------------------------------------------------------- #
# negative corpus
# --------------------------------------------------------------------------- #

def _header(path: Path) -> dict:
    """The `# defect:` / `# currently:` / `# finding:` lines each case carries."""
    h = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.startswith("#"):
            break
        body = line.lstrip("#").strip()
        for key in ("defect", "currently", "finding"):
            if body.startswith(key + ":"):
                h[key] = body[len(key) + 1:].strip()
    return h


def negative_corpus(neg_dir: Path) -> dict:
    files = sorted(neg_dir.glob("*.yaml"))
    if not files:
        raise SystemExit(f"no negative cases in {neg_dir}; a refusal rate over "
                         f"nothing is not a measurement")
    rows = []
    for f in files:
        h = _header(f)
        try:
            load_policy(f, runtime=False)
            verdict, reason = "ACCEPTED", None
        except Exception as exc:                               # noqa: BLE001
            verdict = "refused"
            # Every line but pydantic's "For further information" link: its
            # first line alone ("1 validation error for Policy") says nothing
            # about WHICH check refused the case. The file's own path is cut
            # to its name BEFORE truncating: a long checkout path used to push
            # the reason past the cut (and wrote local paths into the
            # published JSON).
            text = str(exc).replace(str(f), f.name).replace(f.as_posix(), f.name)
            lines = [ln.strip() for ln in text.splitlines()
                     if ln.strip() and "errors.pydantic.dev" not in ln]
            reason = f"{type(exc).__name__}: {' | '.join(lines)}"[:400]
        declared = (h.get("currently") or "").split()[0] if h.get("currently") else None
        rows.append({"file": f.name, "defect": h.get("defect"),
                     "verdict": verdict, "reason": reason,
                     "declared": declared,
                     "declared_matches": (declared or "").lower() == verdict.lower(),
                     "finding": h.get("finding") if verdict == "ACCEPTED" else None})
    refused = sum(1 for r in rows if r["verdict"] == "refused")
    return {"n": len(rows), "refused": refused,
            "rate": round(refused / len(rows), 4),
            "accepted_findings": [{"file": r["file"], "defect": r["defect"],
                                   "finding": r["finding"]}
                                  for r in rows if r["verdict"] == "ACCEPTED"],
            "rows": rows}


# --------------------------------------------------------------------------- #
# identity census over stored runs
# --------------------------------------------------------------------------- #

def _old_code_hash(pol) -> str:
    """Exactly what Policy.policy_hash returned before 2026-10-06."""
    return pol.legacy_hashes()["legacy16-include-defaults"]


def _set_aside(root: Path, cands) -> list[dict]:
    """Runs moved by their authors into a `demo/out/_<reason>/` folder: out of
    the census proper (`demo/out/*/manifest.json`), never out of the record.
    On the morning of 2026-10-07 another unit moved two hot-apply runs whose
    hash no lock recipe can rebuild into `_hot_apply_not_in_lock/`, and the
    census went from 83/84 to 85/85 without a word; this list is why that is
    visible."""
    out = []
    for m in sorted(glob.glob(str(root / "demo" / "out" / "_*" / "*" / "manifest.json"))):
        man = json.loads(Path(m).read_text(encoding="utf-8"))
        pol, form, _ = resolve_run_policy(man, cands)
        out.append({"run": Path(m).parent.name, "folder": Path(m).parent.parent.name,
                    "policy_hash": man.get("policy_hash"), "resolves": pol is not None,
                    "form": form})
    return out


def stored_run_census(root: Path) -> dict:
    mans = sorted(glob.glob(str(root / "demo" / "out" / "*" / "manifest.json")))
    if not mans:
        return {"n": 0, "note": "no demo/out/*/manifest.json on this machine "
                                "(demo/out is gitignored); census not run"}
    cands = policy_candidates()
    old_index = {_old_code_hash(p) for name, p in cands
                 if name.endswith(".yaml")}
    forms, unmatched, before = Counter(), [], 0
    # The five ArduPilot SITL + MAVROS 2 runs. Graded KPI-grade at flight
    # time; under the 2026-10-06 gate they are `dev` runs and are NOT (see
    # guardrail.manifest.is_kpi_grade), so the key does not call them that.
    mavros5 = ("ros2_shield_on", "ros2_shield_off", "ros2_shield_on_dynamic",
               "ros2_ped_on", "ros2_ped_off")
    five = {}
    for m in mans:
        tag = Path(m).parent.name
        man = json.loads(Path(m).read_text(encoding="utf-8"))
        h = man.get("policy_hash")
        was = h in old_index
        before += was
        pol, form, label = resolve_run_policy(man, cands)
        if pol is None:
            unmatched.append({"run": tag, "policy_hash": h})
        else:
            forms[form] += 1
        if tag in mavros5:
            five[tag] = {"policy_hash": h, "matched_before": was,
                         "matched_after": pol is not None, "form": form,
                         "policy": label}
    return {"n": len(mans), "matched_before": before,
            "matched_after": len(mans) - len(unmatched),
            "after_by_form": dict(forms), "unmatched_after": unmatched,
            "set_aside": _set_aside(root, cands),
            "mavros_dev_five": five,
            "before_means": ("Policy.policy_hash as computed by the code before "
                             "2026-10-06 (16 hex over model_dump() with nulls), "
                             "against every policies/*.yaml"),
            "after_means": ("guardrail.manifest.resolve_run_policy over "
                            "guardrail.bundle.policy_candidates(): the 64-hex "
                            "hash, every legacy 16-hex form, and the hot-applied "
                            "derivations in policies/policy.lock.json")}


# --------------------------------------------------------------------------- #
# the grant's own form
# --------------------------------------------------------------------------- #

def _rule_meaning(c) -> dict:
    """What a rule means, independent of how it is written: points in metres
    (to GEOMETRY_TOL_M), the grant's spelling of the breach action, absent
    scope/layer/altitude_ref as their defaults."""
    d = c.model_dump(mode="json")

    def pts(n):
        if isinstance(n, list):
            return [pts(v) for v in n]
        if isinstance(n, dict):
            if "x" in n and "y" in n:
                return (round(n["x"] / GEOMETRY_TOL_M), round(n["y"] / GEOMETRY_TOL_M))
            return {k: pts(v) for k, v in n.items()}
        return n
    d = pts(d)
    d["violation_action"] = canonical_action(c.violation_action)
    d["scope"], d["layer"] = c.effective_scope, c.effective_layer
    if "altitude_ref" in d:
        d["altitude_ref"] = d["altitude_ref"] or "AGL"
    return d


def grant_form_round_trip(policies_dir: Path) -> dict:
    """policy -> grant form -> policy -> grant form: same document, same rules."""
    rows = []
    for f in sorted(policies_dir.glob("*.yaml")):
        row = {"file": f.name}
        try:
            pol = load_policy(f, runtime=False)
            g1 = pol.to_grant_form(origin=DOC_ANCHOR)
            back = policy_from_raw(json.loads(json.dumps(g1)), source=f.name,
                                   runtime=False)
            g2 = back.to_grant_form()
            row.update({
                "anchored_at_doc_anchor": pol.origin is None and not pol.is_geographic
                and any(True for _ in _points(pol)),
                "same_document": g1 == g2,
                "same_rules": [_rule_meaning(c) for c in back.constraints]
                == [_rule_meaning(c) for c in pol.constraints],
                "extensions": pol.grant_form_extensions(),
            })
            row["ok"] = row["same_document"] and row["same_rules"]
        except Exception as exc:                               # noqa: BLE001
            row.update({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        rows.append(row)
    ok = sum(1 for r in rows if r.get("ok"))
    return {"n": len(rows), "passed": ok, "rows": rows}


def _points(pol):
    for c in pol.constraints:
        for name in ("vertices", "centerline"):
            for v in getattr(c, name, None) or []:
                yield v


def run_reference(script: str, timeout: float = 120.0) -> dict:
    """Run a snippet in the reference implementation's own Python 3.11
    virtualenv, READ-ONLY (no bytecode written, cwd its repo), and return the
    JSON it prints. {"run": False, ...} where the environment is absent."""
    import subprocess
    if not REFERENCE_PY.is_file():
        return {"run": False, "note": f"no reference virtualenv at {REFERENCE_DIRNAME}/"
                                      f"{REFERENCE_PY.relative_to(REFERENCE_REPO).as_posix()}"}
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    try:
        cp = subprocess.run([str(REFERENCE_PY), "-c", script], cwd=str(REFERENCE_REPO),
                            capture_output=True, text=True, timeout=timeout, env=env)
    except (OSError, subprocess.SubprocessError) as exc:
        return {"run": False, "note": f"{type(exc).__name__}: {exc}"}
    if cp.returncode != 0:
        return {"run": True, "ok": False,
                "error": (cp.stderr.strip().splitlines() or ["?"])[-1][:300]}
    try:
        return {"run": True, "ok": True, **json.loads(cp.stdout.strip().splitlines()[-1])}
    except (ValueError, IndexError):
        return {"run": True, "ok": False, "error": f"unreadable output {cp.stdout[:200]!r}"}


# Bundles are passed as {document rel path: temp bundle path} and reported by
# the document's rel path: a temp path in the output would change on every run
# and publish a local directory name.
_REF_HASH_SCRIPT = """
import json, sys
from policy_dsl import ingest_file, load_bundle
out = {"documents": {}, "bundles": {}}
for p in %r:
    try: out["documents"][p] = ingest_file(p).policy_hash
    except Exception as e: out["documents"][p] = "REFUSED " + type(e).__name__
for rel, p in %r.items():
    try: out["bundles"][rel] = load_bundle(p).policy_hash
    except Exception as e: out["bundles"][rel] = "REFUSED " + type(e).__name__ + ": " + str(e).splitlines()[0][:160]
print(json.dumps(out))
"""


def reference_cross_load(run_reference_loader: bool = True) -> dict:
    """Our loader on the reference's documents and bundle, and - where its
    environment exists - the reference's loader on the same documents and on
    a bundle written here with ir_form="reference"."""
    docs = sorted((REFERENCE_REPO / "bundles").glob("*.yaml"))
    tars = sorted((REFERENCE_REPO / "bundles").glob("*.tar.gz"))
    if not docs:
        return {"run": False, "note": "reference repository not present"}
    ours_docs, ours_tars, written = {}, {}, {}
    tmp = Path(tempfile.mkdtemp(prefix="wp1ref_"))
    for d in docs:
        rel = d.relative_to(REFERENCE_REPO).as_posix()
        try:
            pol = load_policy(d, runtime=False)     # the declaration
            ours_docs[rel] = {"loads": True, "reference_hash": pol.reference_hash(),
                              "policy_hash": pol.policy_hash,
                              "frame_origin": list(pol.frame_origin or [])}
            b = write_bundle(pol, tmp / f"{d.stem}.reference.tar.gz",
                             changelog="wp1 cross-load", signer=None,
                             ir_form="reference")
            written[rel] = str(b)
            # Both hashes of the bundle, read back by this loader: the
            # reference's (declared) and this project's hash of the source
            # its reference_delta rebuilds (2026-10-08).
            back = check_bundle(b)
            ours_docs[rel]["source_verified"] = (
                back.source_policy is not None
                and back.source_policy.policy_hash == pol.policy_hash
                and back.hashes == {HASH_SCHEME: pol.policy_hash,
                                    REFERENCE_FORM: pol.reference_hash()})
        except Exception as exc:                               # noqa: BLE001
            ours_docs[rel] = {"loads": False, "error": f"{type(exc).__name__}: {exc}"[:300]}
    for t in tars:
        rel = t.relative_to(REFERENCE_REPO).as_posix()
        try:
            chk = check_bundle(t)
            ours_tars[rel] = {"loads": True, "hash_form": chk.hash_form,
                              "signature": chk.signature,
                              "policy_hash_declared": chk.manifest.get("policy_hash")}
        except Exception as exc:                               # noqa: BLE001
            ours_tars[rel] = {"loads": False, "error": f"{type(exc).__name__}: {exc}"[:300]}
    ref = {"run": False, "note": "skipped"}
    if run_reference_loader:
        ref = run_reference(_REF_HASH_SCRIPT % (
            [d.relative_to(REFERENCE_REPO).as_posix() for d in docs],
            {rel: str(Path(p)) for rel, p in written.items()}))
    agree = None
    reads_ours = None
    if ref.get("ok"):
        agree = {rel: ref["documents"].get(rel) == row.get("reference_hash")
                 for rel, row in ours_docs.items()}
        reads_ours = {rel: ref["bundles"].get(rel) == ours_docs[rel]["reference_hash"]
                      for rel in written}
    return {
        "run": True,
        "our_loader_on_reference_documents": ours_docs,
        "our_loader_on_reference_bundles": ours_tars,
        "reference_loader": ref,
        "reference_hash_agrees": agree,
        "reference_loader_reads_our_reference_form_bundle": reads_ours,
        "form_name": REFERENCE_FORM,
        "null_2026_10_05": {"documents_loaded": f"0/{len(docs)}",
                            "bundles_loaded": f"0/{len(tars)}",
                            "why": "issued_at refused; geometry block, project_fix, "
                                   "altitude_min_m unknown; no origin refused"},
    }


# --------------------------------------------------------------------------- #
# cold start: a signed 50-rule bundle on disk -> a Shield ready to filter
# (2026-10-19, audit WP1 KPI line "cold-start <= 1 s ... belum dilaporkan")
# --------------------------------------------------------------------------- #
#
# THE GRANT'S NUMBER. Policy DSL page, "Acceptance KPIs (locked)": "Cold-start
# ingest of a typical 50-rule bundle <= 1 s". The ingest diagram on the same
# page names the path: parse + schema validation -> geometry normalisation ->
# lint -> "Build runtime IR (R-tree / grid spatial index ...)" -> sign + version
# -> signed bundle, and "Cold-start path ... runs once per mission start".
#
# WHAT WAS MEASURED BEFORE. Only Shield construction at 50 rules, median 4.7 ms
# (docs/DESIGN-shield-enforcement.md, experiments/bench_shield_50rules.py),
# which that bench itself calls "only the IR part of a cold start - no YAML
# parse, no bundle signature check". Nobody had timed the rest, so the KPI was
# reported nowhere (docs/AUDIT-KONTRAK-2026-10-05.md, KPI WP1).
#
# WHAT IS TIMED HERE: the path a flight rail takes at mission start, through
# the entry points the rails call and nothing re-implemented -
#
#   guardrail.bundle.load_for_flight(bundle, trust_store=...)
#       tar.gz read; ir.json parsed and validated by the pydantic models (WGS84
#       points projected); policy_hash recomputed over the canonical IR and
#       compared with the manifest; both hash forms verified; DSL lint; the
#       Ed25519 signature verified against a trust store read from disk; the
#       flight gate (_refuse_for_flight); runtime_notes;
#   guardrail.replay.rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=...)
#       the Shield every rail builds: the runtime IR (compiled margin rings,
#       the STRtree, schedules, corridors).
#
# twice over: in this process (warm: imports done), and in FRESH interpreters -
# one process per run, timing its own imports of guardrail.bundle /
# guardrail.replay / guardrail.shield, then the first call of the path (which
# pays any lazy cost once), then the same path again in that process. The
# parent also times each child from spawn to exit, which adds the
# interpreter's own start-up. The KPI is the fresh process, imports included;
# the whole wall time is printed beside it, so neither reading is hidden. On
# this desktop the imports are ~95 % of it (docs/DESIGN-wp1-cold-start.md).
#
# TWO BUNDLES, BOTH 50 RULES. "typical": written the way the DSL page calls
# canonical (WGS84 with an origin, projected at load), mixing the classes a
# mission policy carries - 32 polygon fences of 6 to 12 vertices with margin
# rings, 10 circle fences (each a derived 32-gon), 2 keep-in corridors, an
# altitude and a kinematic envelope, obstacle clearance, a subject standoff and
# 2 time-window switches, a third of the rules on weekly schedules. "grid": the
# 50-rule load every monitor budget in this repository is quoted against
# (experiments/bench_shield_50rules.fifty_rule_policy, 2 envelopes + 48 square
# fences in metres). Both are signed with a fixed Ed25519 seed whose public key
# sits in a trust store FILE written beside them, so the signature status is
# "verified" on any machine, with or without the lab's private key.
#
# WHY A FAST NUMBER IS NOT ENOUGH ("silence reads as success"). A path that
# skipped the signature, the hash or the IR would post the best time in the
# table. So every timed run must also return a VERIFIED signature, the
# expected policy_hash, 50 rules and the expected number of compiled zones; one
# Shield per bundle must raise a geofence violation from inside its first
# fence; and the same path must REFUSE a tampered bundle and one signed by a
# key the trust store does not hold. `verdict.within_budget` is True only when
# all of that holds AND the p95 fits: a fast path that verified nothing reads
# as a failure, not a pass.
#
# NOT MEASURED: a reboot-cold disk (one discarded fresh run warms the OS file
# cache and the .pyc files first, as on any second mission of a day), and the
# onboard computer the grant's budget is for (Jetson Orin): re-run
# `python tools/wp1_roundtrip_kpi.py` there before quoting a margin for it.

COLD_START_BUDGET_MS = 1000.0
COLD_START_RULES = 50
# The timing bundles' signing key: a fixed seed, so the bundles are
# byte-reproducible and verifiable everywhere. It signs nothing but these
# timing fixtures, and its identity names itself as a bench key; the lab's
# development key (policies/keys) is never used or needed here.
COLD_START_AUTHORITY = "wp1-cold-start-bench"
COLD_START_SEED = hashlib.sha256(b"wp1 cold-start timing fixture key").digest()
COLD_START_POLICIES = ("typical", "grid")
# Fresh-interpreter child: times its own imports, then the first call of the
# flight path, then the same path once more in the same process. It must not
# import this tool (whose top level imports guardrail before any timer starts).
_COLD_CHILD = r"""
import json, sys, time
t0 = time.perf_counter()
root, bundle, store = sys.argv[1:4]
sys.path.insert(0, root)
from guardrail.bundle import load_for_flight, verifier_name
from guardrail.replay import rail_shield
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
t1 = time.perf_counter()
pol, rec = load_for_flight(bundle, trust_store=store)
t2 = time.perf_counter()
sh = rail_shield(pol, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
t3 = time.perf_counter()
pol2, rec2 = load_for_flight(bundle, trust_store=store)
rail_shield(pol2, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
t4 = time.perf_counter()
print(json.dumps({"import_ms": (t1 - t0) * 1e3, "load_for_flight_ms": (t2 - t1) * 1e3,
                  "shield_ms": (t3 - t2) * 1e3, "second_call_ms": (t4 - t3) * 1e3,
                  "signature": rec["signature"], "verifier": verifier_name(),
                  "policy_hash": rec["loaded_policy_hash"],
                  "rules": len(pol.constraints), "fences": len(sh._rt.ir.fences)}))
"""


def typical_fifty_rule_raw() -> dict:
    """The "typical 50-rule bundle" as a policy author writes it: WGS84
    points with an origin (DOC_ANCHOR), the mix of rule classes described in
    the section comment above. Deterministic; no rule covers the frame origin
    (the take-off point), and the corridors' schedules never overlap, so the
    policy passes the DSL lint and the flight gate as written."""
    from guardrail.projection import LocalProjection
    proj = LocalProjection(*DOC_ANCHOR)

    def ll(x, y):
        lat, lon = proj.to_latlon(x, y)
        return {"lat": round(lat, 7), "lon": round(lon, 7)}

    def ring(cx, cy, r, n):
        return [ll(cx + r * math.cos(2 * math.pi * k / n),
                   cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]

    weekdays = {"recurrence": {"days": ["Mon", "Tue", "Wed", "Thu", "Fri"],
                               "start_time": "07:30", "end_time": "17:30"}}
    nights = {"recurrence": {"start_time": "22:00", "end_time": "06:00"}}
    rules = [
        {"id": "alt-band", "type": "altitude_envelope", "priority": "P0",
         "alt_min_m": 5.0, "alt_max_m": 120.0},
        {"id": "kin-caps", "type": "kinematic_envelope", "priority": "P1",
         "speed_max_mps": 12.0, "climb_rate_max_mps": 4.0, "yaw_rate_max_dps": 60.0},
    ]
    # 32 polygon fences, an 8 x 4 block grid 60 m apart, north-east of the
    # take-off point.
    for k in range(32):
        r_, c_ = divmod(k, 8)
        fence = {"id": f"nfz-{k:02d}", "type": "polygon_fence",
                 "priority": "P0" if k % 4 else "P1",
                 "vertices": ring(60.0 + 60.0 * r_, 60.0 + 60.0 * c_,
                                  10.0 + (k % 5) * 2.0, 6 + k % 7),
                 "altitude_floor_m": 0.0,
                 "altitude_ceiling_m": 120.0 if k % 2 == 0 else 40.0,
                 "margin_m": 1.0 + (k % 3)}
        if k % 3 == 0:
            fence["valid_time"] = weekdays
        rules.append(fence)
    # 10 circle fences in a row south of the grid.
    for k in range(10):
        rules.append({"id": f"disc-{k}", "type": "circle_fence", "priority": "P0",
                      "center": ll(-80.0, 40.0 + 45.0 * k),
                      "radius_m": 8.0 + k, "altitude_ceiling_m": 120.0,
                      "margin_m": 1.5, **({"valid_time": nights} if k % 2 else {})})
    # Two keep-in corridors, one by day and one by night (never in force
    # together: the lint compares hard corridor bands that can be).
    rules.append({"id": "corridor-day", "type": "corridor", "priority": "P1",
                  "centerline": [ll(0.0, 0.0), ll(30.0, 30.0), ll(30.0, 520.0),
                                 ll(300.0, 520.0)],
                  "width_m": 30.0, "altitude_floor_m": 5.0, "altitude_ceiling_m": 120.0,
                  "valid_time": weekdays})
    rules.append({"id": "corridor-night", "type": "corridor", "priority": "P1",
                  "centerline": [ll(0.0, 0.0), ll(-40.0, 0.0), ll(-40.0, 520.0)],
                  "width_m": 24.0, "altitude_floor_m": 5.0, "altitude_ceiling_m": 60.0,
                  "valid_time": nights})
    rules.append({"id": "clearance", "type": "obstacle_clearance", "priority": "P0",
                  "min_clearance_m": 3.0, "soft_margin_m": 2.0})
    rules.append({"id": "standoff-ped", "type": "subject_standoff", "priority": "P0",
                  "subject_class": "pedestrian", "min_range_m": 10.0})
    # Two switches holding two fences on at night as well.
    for k in (0, 3):
        rules.append({"id": f"night-hold-{k:02d}", "type": "time_window_switch",
                      "priority": "P1", "target_id": f"nfz-{k:02d}", "active": True,
                      "valid_time": nights})
    return {"policy_id": "wp1-cold-start-typical", "version": "1.0.0",
            "origin": {"lat": DOC_ANCHOR[0], "lon": DOC_ANCHOR[1]},
            "constraints": rules}


def _cold_start_policies() -> dict:
    """name -> (Policy, authored YAML text or None)."""
    import yaml
    sys.path.insert(0, str(ROOT / "experiments"))
    import bench_shield_50rules as bench
    raw = typical_fifty_rule_raw()
    typical = policy_from_raw(raw, source="wp1-cold-start-typical")
    return {"typical": (typical, yaml.safe_dump(raw, sort_keys=False)),
            "grid": (bench.fifty_rule_policy(), None)}


def _cold_start_signer(seed: bytes = COLD_START_SEED):
    """The fixture key: `cryptography`'s Ed25519 where installed, else the
    module's RFC 8032 implementation. Ed25519 is deterministic, so both sign
    the same bytes the same way."""
    import guardrail.bundle as gb
    if gb.HAVE_CRYPTO:
        key = gb.Ed25519PrivateKey.from_private_bytes(seed)
    else:
        key = gb._PyEd25519TestKey(seed)
    return gb.Signer(authority=COLD_START_AUTHORITY, private_key=key)


def _trust_store_file(path: Path, signer) -> Path:
    path.write_text(json.dumps({"signers": {signer.identity: {
        "authority": signer.authority, "algorithm": "ed25519",
        "public_key_hex": signer.public_raw.hex(),
        "note": "wp1 cold-start timing fixture key, not a CA"}}}, indent=2),
        encoding="utf-8", newline="\n")
    return path


def _pctl(xs: list[float], q: float) -> float:
    """Nearest-rank percentile (no interpolation: a value that was measured)."""
    s = sorted(xs)
    return s[max(0, math.ceil(q / 100.0 * len(s)) - 1)]


def _ms_stats(xs: list[float]) -> dict:
    return {"n": len(xs), "median_ms": round(statistics.median(xs), 3),
            "p95_ms": round(_pctl(xs, 95), 3), "min_ms": round(min(xs), 3),
            "max_ms": round(max(xs), 3)}


class _PurePythonVerifier:
    """Verify with the RFC 8032 fallback, as the WSL venvs without
    `cryptography` do (guardrail/bundle.py, "VERIFYING EVERYWHERE")."""

    def __enter__(self):
        import guardrail.bundle as gb
        self._gb, self._saved = gb, gb.HAVE_CRYPTO
        gb.HAVE_CRYPTO = False
        return self

    def __exit__(self, *exc):
        self._gb.HAVE_CRYPTO = self._saved
        return False


def _first_fence_raises(policy, shield) -> bool:
    """Does a Shield built by the timed path raise a geofence violation from
    the centroid of the policy's first polygon fence (inside its band)? A
    Shield whose IR came out empty would answer False."""
    from guardrail.models import Action4D, State
    fence = next(c for c in policy.constraints if c.type == "polygon_fence")
    xs = [v.x for v in fence.vertices]
    ys = [v.y for v in fence.vertices]
    up = min(20.0, (fence.altitude_floor_m + fence.altitude_ceiling_m) / 2.0)
    st = State(x=sum(xs) / len(xs), y=sum(ys) / len(ys), up=up)
    return any(v.category == "geofence" for v in shield._check(st, Action4D()))


def cold_start(warm_runs: int = 100, fresh_runs: int = 20,
               python: str | None = None) -> dict:
    """Time the cold-start ingest (section comment above). `warm_runs`
    in-process repetitions per bundle; `fresh_runs` fresh interpreters per
    bundle (0 skips them); `python`, the interpreter the children run
    (default: this one)."""
    import subprocess
    import time
    from guardrail.bundle import (BundleSignatureError, load_for_flight,
                                  verify_signature, parse_signature,
                                  MANIFEST_NAME, SIGNATURE_NAME, _read)
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    from guardrail.models import Policy
    if warm_runs < 1:
        raise ValueError("warm_runs must be >= 1: a timing over no run is not one")
    tmp = Path(tempfile.mkdtemp(prefix="wp1cold_"))
    signer = _cold_start_signer()
    store = _trust_store_file(tmp / "trusted_signers.json", signer)
    pols = _cold_start_policies()
    fixtures, checks = {}, {}
    for name, (pol, text) in pols.items():
        b = write_bundle(pol, tmp / f"{name}.tar.gz", changelog="wp1 cold-start timing",
                         issued_at="2026-01-01T00:00:00Z", signer=signer)
        fixtures[name] = {
            "rules": len(pol.constraints),
            "by_type": dict(sorted(Counter(c.type for c in pol.constraints).items())),
            "scheduled_rules": sum(1 for c in pol.constraints if c.valid_time is not None),
            # Every keep-out zone the Shield must compile (a circle_fence is a
            # derived 32-gon); a timed run whose IR holds fewer did not build it.
            "zones": sum(1 for c in pol.constraints
                         if c.type in ("polygon_fence", "circle_fence", "dynamic_nfz")),
            "geographic": pol.is_geographic, "policy_hash": pol.policy_hash,
            "bundle_bytes": b.stat().st_size, "_bundle": b, "_policy": pol, "_yaml": text}
        if len(pol.constraints) != COLD_START_RULES:
            raise SystemExit(f"cold-start fixture {name} has {len(pol.constraints)} "
                             f"rules, not {COLD_START_RULES}")

    def flight_path(b):
        t0 = time.perf_counter()
        pol, rec = load_for_flight(b, trust_store=store)
        t1 = time.perf_counter()
        sh = rail_shield(pol, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
        t2 = time.perf_counter()
        return (t1 - t0) * 1e3, (t2 - t1) * 1e3, pol, rec, sh

    def proof(name, rec, pol, sh) -> bool:
        """The timed run did the work: verified, the issued policy, every rule,
        every zone compiled."""
        f = fixtures[name]
        return (rec.get("signature") == "verified"
                and rec.get("loaded_policy_hash") == f["policy_hash"]
                and len(pol.constraints) == COLD_START_RULES
                and len(sh._rt.ir.fences) == f["zones"])

    in_proc, py_verify, author = {}, {}, {}
    for name, f in fixtures.items():
        b = f["_bundle"]
        lff, shd, tot, bad = [], [], [], 0
        row = {}
        first = None
        for i in range(warm_runs):
            a, s, pol, rec, sh = flight_path(b)
            if i == 0:
                first = a + s
                row["fences"] = len(sh._rt.ir.fences)
                row["geofence_raised_inside_first_fence"] = _first_fence_raises(pol, sh)
                row["verifier"] = rec.get("signature_detail", "")
            lff.append(a)
            shd.append(s)
            tot.append(a + s)
            bad += not proof(name, rec, pol, sh)
        # Where the warm time goes (informational; the KPI is the path above).
        tar_ms, parse_ms, sig_ms = [], [], []
        for _ in range(min(warm_runs, 50)):
            t0 = time.perf_counter()
            with tarfile.open(b, "r:gz") as tar:
                ir_b, man_b = _read(tar, IR_NAME), _read(tar, MANIFEST_NAME)
                sig_t = _read(tar, SIGNATURE_NAME).decode()
            t1 = time.perf_counter()
            Policy.model_validate(json.loads(ir_b))
            t2 = time.perf_counter()
            _, who, sig_hex = parse_signature(sig_t)
            status, _ = verify_signature(who, sig_hex, man_b, store)
            t3 = time.perf_counter()
            assert status == "verified", status
            tar_ms.append((t1 - t0) * 1e3)
            parse_ms.append((t2 - t1) * 1e3)
            sig_ms.append((t3 - t2) * 1e3)
        in_proc[name] = {"load_for_flight": _ms_stats(lff), "shield": _ms_stats(shd),
                         "total": _ms_stats(tot), "first_call_ms": round(first, 3),
                         "stages_ms_median": {
                             "tar_read": round(statistics.median(tar_ms), 3),
                             "ir_parse_and_validate": round(statistics.median(parse_ms), 3),
                             "signature_verify": round(statistics.median(sig_ms), 3)}}
        checks[name] = {"runs_failing_a_check": bad, "fences_compiled": row["fences"],
                        "geofence_raised_inside_first_fence":
                            row["geofence_raised_inside_first_fence"],
                        "verifier": row["verifier"]}
        with _PurePythonVerifier():
            pp = []
            for _ in range(max(3, warm_runs // 5)):
                a, s, pol, rec, sh = flight_path(b)
                pp.append(a + s)
                checks[name]["runs_failing_a_check"] += not proof(name, rec, pol, sh)
            py_verify[name] = {"total": _ms_stats(pp),
                               "verifier": rec.get("signature_detail", "")}
        if f["_yaml"] is not None:
            # The author's side of the grant's diagram: YAML -> parse,
            # normalise, lint -> canonical IR, hash -> signed bundle.
            src = tmp / f"{name}.yaml"
            src.write_text(f["_yaml"], encoding="utf-8", newline="\n")
            ya = []
            for i in range(max(3, warm_runs // 5)):
                t0 = time.perf_counter()
                p = load_policy(src)
                write_bundle(p, tmp / f"{name}.author.tar.gz", changelog="author",
                             issued_at="2026-01-01T00:00:00Z", signer=signer)
                ya.append((time.perf_counter() - t0) * 1e3)
            author[name] = {"yaml_to_signed_bundle": _ms_stats(ya),
                            "same_policy_hash": p.policy_hash == f["policy_hash"]}

    # The same path must say NO.
    refusals = {}
    probe = fixtures["typical"]["_bundle"]
    bad = tmp / "tampered.tar.gz"
    _tamper(probe, bad)
    try:
        load_for_flight(bad, trust_store=store)
        refusals["tampered_ir"] = "ACCEPTED"
    except ValueError:
        refusals["tampered_ir"] = "refused"
    stranger = _cold_start_signer(hashlib.sha256(b"not in the trust store").digest())
    foreign = write_bundle(fixtures["typical"]["_policy"], tmp / "foreign.tar.gz",
                           changelog="foreign", issued_at="2026-01-01T00:00:00Z",
                           signer=stranger)
    try:
        load_for_flight(foreign, trust_store=store)
        refusals["untrusted_signer"] = "ACCEPTED"
    except BundleSignatureError:
        refusals["untrusted_signer"] = "refused"

    fresh: dict = {}
    py = python or sys.executable
    env = dict(os.environ)
    env.pop("PYTHONDONTWRITEBYTECODE", None)
    if fresh_runs > 0:
        # One discarded child: compiles any stale .pyc and warms the OS file
        # cache, as on the second mission of a day (not measured: reboot-cold).
        subprocess.run([py, "-c", _COLD_CHILD, str(ROOT), str(probe), str(store)],
                       cwd=str(tmp), capture_output=True, text=True, timeout=120, env=env)
        rows: dict = {n: [] for n in fixtures}
        for i in range(fresh_runs):
            for name, f in fixtures.items():            # interleaved: shared noise
                t0 = time.perf_counter()
                cp = subprocess.run([py, "-c", _COLD_CHILD, str(ROOT), str(f["_bundle"]),
                                     str(store)], cwd=str(tmp), capture_output=True,
                                    text=True, timeout=120, env=env)
                wall = (time.perf_counter() - t0) * 1e3
                if cp.returncode != 0:
                    raise SystemExit(f"cold-start child failed ({name}): "
                                     f"{(cp.stderr.strip().splitlines() or ['?'])[-1]}")
                r = json.loads(cp.stdout.strip().splitlines()[-1])
                r["wall_ms"] = wall
                rows[name].append(r)
        for name, rs in rows.items():
            f = fixtures[name]
            failing = [r for r in rs if not (
                r["signature"] == "verified" and r["policy_hash"] == f["policy_hash"]
                and r["rules"] == COLD_START_RULES and r["fences"] == f["zones"])]
            checks[name]["fresh_runs_failing_a_check"] = len(failing)
            fresh[name] = {
                "runs": len(rs), "verifier": rs[0]["verifier"],
                "import": _ms_stats([r["import_ms"] for r in rs]),
                "load_for_flight": _ms_stats([r["load_for_flight_ms"] for r in rs]),
                "shield": _ms_stats([r["shield_ms"] for r in rs]),
                "path_first_call": _ms_stats([r["load_for_flight_ms"] + r["shield_ms"]
                                              for r in rs]),
                "ingest_with_imports": _ms_stats([r["import_ms"] + r["load_for_flight_ms"]
                                                  + r["shield_ms"] for r in rs]),
                "whole_process_wall": _ms_stats([r["wall_ms"] for r in rs]),
                "second_call_same_process": _ms_stats([r["second_call_ms"] for r in rs])}

    all_checks = (all(c["runs_failing_a_check"] == 0 and c["geofence_raised_inside_first_fence"]
                      and c.get("fresh_runs_failing_a_check", 0) == 0
                      for c in checks.values())
                  and all(v == "refused" for v in refusals.values()))
    if fresh:
        head = max(fresh.values(), key=lambda v: v["ingest_with_imports"]["p95_ms"])
        p95 = head["ingest_with_imports"]["p95_ms"]
        wall = max(v["whole_process_wall"]["p95_ms"] for v in fresh.values())
        reading = "fresh process, imports + verify + parse + IR + Shield (worst bundle)"
    else:
        p95 = max(v["total"]["p95_ms"] for v in in_proc.values())
        wall = None
        reading = "in process only (fresh runs skipped): verify + parse + IR + Shield"
    verdict = {"reading": reading, "p95_ms": p95, "whole_process_wall_p95_ms": wall,
               "budget_ms": COLD_START_BUDGET_MS, "checks_hold": all_checks,
               "within_budget": bool(all_checks and p95 <= COLD_START_BUDGET_MS)}
    for f in fixtures.values():
        for k in ("_bundle", "_policy", "_yaml"):
            f.pop(k)
    return {
        "grant": "Policy DSL page, Acceptance KPIs (locked): cold-start ingest of a "
                 "typical 50-rule bundle <= 1 s",
        "path": "guardrail.bundle.load_for_flight (tar read, IR parse + validation, "
                "policy_hash, both hash forms, lint, Ed25519 verify against a trust "
                "store file, flight gate, runtime notes) + guardrail.replay.rail_shield "
                "(the runtime IR: margin rings, STRtree, schedules)",
        "machine": {"processor": platform.processor() or platform.machine(),
                    "system": f"{platform.system()} {platform.release()}",
                    "python": platform.python_version(),
                    "child_python": py if python else "this interpreter"},
        "warm_runs": warm_runs, "fresh_runs": fresh_runs,
        "bundles": fixtures, "in_process": in_proc,
        "pure_python_verifier": py_verify, "author_side": author,
        "fresh_process": fresh if fresh else {"note": "skipped (fresh_runs=0)"},
        "checks": checks, "refusals": refusals, "verdict": verdict,
        "null_before_2026_10_19": ("only Shield construction was timed (median 4.7 ms "
                                   "at 50 rules, docs/DESIGN-shield-enforcement.md): no "
                                   "bundle read, no IR parse or validation, no hash, no "
                                   "signature check, no imports, no fresh process"),
        "not_measured": ["a reboot-cold disk: one discarded fresh run warms the OS "
                         "file cache and the .pyc files first",
                         "the onboard computer the budget is for (Jetson Orin)"],
    }


def _cold_start_kpi(cs: dict | None) -> dict:
    if not cs:
        return {"cold_start_50_rules": "not run",
                "cold_start_within_budget": "not run"}
    v = cs["verdict"]
    text = f"p95 {v['p95_ms']:.0f} ms <= {v['budget_ms']:.0f} ms ({v['reading']})"
    if v["whole_process_wall_p95_ms"] is not None:
        text += f"; whole process incl. interpreter start p95 {v['whole_process_wall_p95_ms']:.0f} ms"
    if v["p95_ms"] > v["budget_ms"]:
        text = text.replace(" <= ", " > ")
    return {"cold_start_50_rules": text, "cold_start_within_budget": v["within_budget"]}


# --------------------------------------------------------------------------- #

def schema_status() -> dict:
    want = policy_json_schema()
    if not SCHEMA_PATH.is_file():
        return {"path": SCHEMA_PATH.relative_to(ROOT).as_posix(), "current": False,
                "note": "missing - run python -m guardrail.bundle schema"}
    have = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    return {"path": SCHEMA_PATH.relative_to(ROOT).as_posix(), "current": have == want,
            "definitions": len(want.get("$defs", {}))}


def json_compatible(node):
    """A YAML document in JSON's data model: a YAML timestamp becomes its ISO
    text, which is how the loader reads it (models.Policy._timestamp_text)."""
    from datetime import date
    if isinstance(node, dict):
        return {k: json_compatible(v) for k, v in node.items()}
    if isinstance(node, list):
        return [json_compatible(v) for v in node]
    if isinstance(node, (datetime, date)):
        return node.isoformat()
    return node


def _authored(path: Path):
    import yaml
    return json_compatible(yaml.safe_load(path.read_text(encoding="utf-8")))


def schema_validation(policies_dir: Path, neg_dir: Path = NEGATIVE_DIR) -> dict:
    """The grant's DSL KPI 'schema validation pass rate', against the exported
    JSON Schema - the published contract for external tooling. Four counts:

      authored    each policies/*.yaml AS WRITTEN against the schema's root
                  (the authoring form);
      grant_documents  the reference's bundles/*.yaml, as written, against
                  the root (the grant's worked example is checked by
                  tests/test_policy_dsl_grant_form.py). Null: the schema
                  published at 401305a (2026-10-05, the IR only) passed 0/3
                  of the reference documents and 28/29 authored policies
                  (measured 2026-10-07, scratchpad null_probe.py);
      canonical_ir  each policy's canonical IR against $defs/CanonicalIR;
      negative_refused_by_schema  how many deliberately broken policies the
                  schema ALONE refuses. Not a pass rate: polygon validity,
                  unique ids and conflicting limits are beyond a shape, which
                  is why load_policy, not the schema, is the authority.

    Needs jsonschema; reported as not run without it."""
    try:
        import jsonschema
    except ImportError:
        return {"run": False, "note": "jsonschema not installed in this interpreter"}
    schema = policy_json_schema()
    root = jsonschema.Draft202012Validator(schema)
    canon = jsonschema.Draft202012Validator({**schema, "$ref": "#/$defs/CanonicalIR"})

    def check(validator, items):
        rows = []
        for name, load in items:
            try:
                errs = list(validator.iter_errors(load()))
            except Exception as exc:                           # noqa: BLE001
                rows.append({"file": name, "valid": False,
                             "error": f"{type(exc).__name__}: {str(exc)[:160]}"})
                continue
            rows.append({"file": name, "valid": not errs,
                         **({"error": errs[0].message[:160]} if errs else {})})
        return rows

    files = sorted(policies_dir.glob("*.yaml"))
    authored = check(root, [(f.name, lambda f=f: _authored(f)) for f in files])
    canonical = check(canon, [(f.name, lambda f=f: load_policy(f, runtime=False).canonical_ir())
                              for f in files])
    # Named as in this checkout (kuanting-vla-uav-guardrail/bundles/...) also
    # when --reference-repo reads another checkout's copy, so the published
    # file does not change with where it was generated.
    grant_items = [(f"{REFERENCE_DIRNAME}/{d.relative_to(REFERENCE_REPO).as_posix()}",
                    lambda d=d: _authored(d))
                   for d in sorted((REFERENCE_REPO / "bundles").glob("*.yaml"))]
    grant = check(root, grant_items)
    negs = sorted(neg_dir.glob("*.yaml"))
    neg_rows = check(root, [(f.name, lambda f=f: _authored(f)) for f in negs])

    def rate(rows):
        return f"{sum(r['valid'] for r in rows)}/{len(rows)}"
    return {"run": True, "n": len(files),
            "passed": sum(r["valid"] for r in canonical),
            "authored": rate(authored), "canonical_ir": rate(canonical),
            "grant_documents": rate(grant) if grant else "not run (no reference repository)",
            "negative_refused_by_schema": f"{sum(not r['valid'] for r in neg_rows)}/{len(neg_rows)}",
            "null_401305a_schema": {"authored": "28/29", "grant_documents": "0/3"},
            "failures": [r for r in authored + canonical + grant if not r["valid"]]}


def build(policies_dir: Path = ROOT / "policies", neg_dir: Path = NEGATIVE_DIR,
          runs: bool = True, signer="auto", reference: bool = True,
          cold: bool = True, cold_warm: int = 100, cold_fresh: int = 20,
          runs_root: Path = ROOT) -> dict:
    rt = round_trip(policies_dir, signer=signer)
    neg = negative_corpus(neg_dir)
    gf = grant_form_round_trip(policies_dir)
    bh = both_hashes(policies_dir)
    xref = reference_cross_load(run_reference_loader=reference)
    cs = cold_start(warm_runs=cold_warm, fresh_runs=cold_fresh) if cold else None
    n = rt["n"]
    agree = xref.get("reference_hash_agrees")
    reads = xref.get("reference_loader_reads_our_reference_form_bundle")
    docs = xref.get("our_loader_on_reference_documents") or {}
    tars = xref.get("our_loader_on_reference_bundles") or {}
    return {
        "generated": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "command": "python tools/wp1_roundtrip_kpi.py",
        "code_revision": code_revision(),
        "python": platform.python_version(),
        "cryptography_available": HAVE_CRYPTO,
        "signature_verifier": verifier_name(),
        "hash_scheme": HASH_SCHEME,
        "kpi": {
            "round_trip": f"{rt['passed']}/{n}",
            "tamper_refusal": (f"{rt['tamper_refusal']['refused']}/"
                               f"{rt['tamper_refusal']['run']}"),
            "negative_refusal": f"{neg['refused']}/{neg['n']}",
            "negative_accepted": [x["file"] for x in neg["accepted_findings"]],
            "grant_form_round_trip": f"{gf['passed']}/{gf['n']}",
            "reference_documents_loaded": (f"{sum(r.get('loads', False) for r in docs.values())}"
                                           f"/{len(docs)}"),
            "reference_bundles_loaded": (f"{sum(r.get('loads', False) for r in tars.values())}"
                                         f"/{len(tars)}"),
            "reference_hash_agrees": (f"{sum(agree.values())}/{len(agree)}"
                                      if agree is not None else "not run"),
            "reference_loader_reads_ours": (f"{sum(reads.values())}/{len(reads)}"
                                            if reads is not None else "not run"),
            "both_hashes_recorded_and_verified": f"{bh['passed']}/{bh['n']}",
            "reference_hash_representable": f"{bh['representable']}/{bh['n']}",
            "forged_reference_hash_refused": f"{bh['forged_refused']}/{bh['forged_run']}",
            "reference_form_source_verified": (
                f"{sum(bool(r.get('source_verified')) for r in docs.values())}/{len(docs)}"),
            # 2026-10-19: the grant's "cold-start ingest of a typical 50-rule
            # bundle <= 1 s", timed bundle -> verified policy -> Shield.
            **_cold_start_kpi(cs),
        },
        # The 2026-10-08 hash record, read by a loader that ignores it (what
        # 10451bc did): measured on the same forged bundles.
        "null_record_ignored": {
            "forged_reference_hash_refused": (f"{bh['null_record_ignored_refused']}/"
                                              f"{bh['forged_run']}"),
            "why": "10451bc wrote no `hashes` record and read none; the forged "
                   "reference hash is invisible to such a reader"},
        "null_accept_everything": {
            "round_trip": f"{n}/{n}", "tamper_refusal": f"0/{rt['tamper_refusal']['run']}",
            "negative_refusal": f"0/{neg['n']}",
            "why": "a reader that verifies nothing passes every round trip; only "
                   "the refusal columns separate it from a real one"},
        # The same columns for the code as it stood on 2026-10-05 (401305a).
        # Measured, not assumed, where the code existed: its loader on today's
        # negative corpus, and its published schema on the reference
        # documents (scratchpad null_probe.py, 2026-10-07). The grant-form and
        # reference columns are 0 because the functions they call
        # (to_grant_form, reference_hash, the reference IR form) did not exist.
        "null_2026_10_05": {
            "negative_refusal": "20/35 (14/25 on the original 25 cases)",
            "grant_form_round_trip": f"0/{gf['n']}",
            "reference_documents_loaded": f"0/{len(docs)}",
            "reference_bundles_loaded": f"0/{len(tars)}",
            "reference_hash_agrees": f"0/{len(docs)}",
            "reference_loader_reads_ours": f"0/{len(docs)}",
            "schema_grant_documents": "0/3",
            "schema_authored": "28/29",
            # Not run then: no script timed bundle -> policy -> Shield. The only
            # figure was Shield construction alone (2.7 ms at 401305a,
            # docs/DESIGN-shield-enforcement.md), no read, parse, hash or
            # signature in it.
            "cold_start_50_rules": "not measured (Shield construction alone: 2.7 ms)"},
        "round_trip": rt,
        "negative_corpus": neg,
        "grant_form": gf,
        "both_hashes": bh,
        "reference": xref,
        "cold_start": cs if cs is not None else {"note": "skipped (--no-cold-start)"},
        "version_lock_problems": check_lock(policies_dir),
        "schema": schema_status(),
        "schema_validation": schema_validation(policies_dir),
        "stored_runs": stored_run_census(runs_root) if runs else {"note": "skipped (--no-runs)"},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--no-runs", action="store_true")
    ap.add_argument("--no-reference", action="store_true",
                    help="do not run the reference implementation's loader")
    ap.add_argument("--no-cold-start", action="store_true",
                    help="do not time the cold-start ingest (the KPI then says 'not run')")
    ap.add_argument("--cold-start-runs", type=int, default=100,
                    help="in-process repetitions per 50-rule bundle (default 100)")
    ap.add_argument("--cold-start-fresh", type=int, default=20,
                    help="fresh interpreters per 50-rule bundle (default 20; 0 = none)")
    # A git worktree holds neither the gitignored flight runs nor the
    # reference repository; these read them from another checkout, read-only.
    ap.add_argument("--runs-root", default=None,
                    help="the checkout whose demo/out/*/manifest.json the census reads "
                         "(default: this one)")
    ap.add_argument("--reference-repo", default=None,
                    help="the reference implementation's repository (default: "
                         "kuanting-vla-uav-guardrail/ in this checkout)")
    args = ap.parse_args(argv)
    if args.reference_repo:
        global REFERENCE_REPO, REFERENCE_PY
        REFERENCE_REPO = Path(args.reference_repo).resolve()
        REFERENCE_PY = REFERENCE_REPO / ".venv" / "Scripts" / "python.exe"
    res = build(runs=not args.no_runs, reference=not args.no_reference,
                cold=not args.no_cold_start, cold_warm=args.cold_start_runs,
                cold_fresh=args.cold_start_fresh,
                runs_root=Path(args.runs_root).resolve() if args.runs_root else ROOT)

    print("\n[wp1] policy load round-trip KPI")
    for k, v in res["kpi"].items():
        say(k, v)
    say("null: accept everything", {k: v for k, v in res["null_accept_everything"].items()
                                     if k != "why"})
    say("null: the 2026-10-05 code", res["null_2026_10_05"])
    say("null: the hash record ignored (10451bc)",
        {k: v for k, v in res["null_record_ignored"].items() if k != "why"})
    say("not representable by the reference, because",
        res["both_hashes"]["not_representable_because"])
    say("signatures on the round-trip bundles", res["round_trip"]["signatures"])
    for r in res["round_trip"]["rows"]:
        if not r.get("ok") or r.get("tamper") != "refused":
            say(f"  ! {r['file']}", r.get("error") or r.get("tamper"))
    print("\n[wp1] negative corpus - accepted although broken (findings)")
    for x in res["negative_corpus"]["accepted_findings"]:
        say(f"  {x['file']}", x["defect"])
    for r in res["negative_corpus"]["rows"]:
        if not r["declared_matches"]:
            say(f"  ! header out of date: {r['file']}",
                f"says {r['declared']}, actually {r['verdict']}")
    for r in res["grant_form"]["rows"]:
        if not r.get("ok"):
            say(f"  ! grant form {r['file']}", r.get("error") or r)
    rl = res["reference"].get("reference_loader") or {}
    if rl and not rl.get("ok", False):
        say("  reference loader", rl.get("note") or rl.get("error"))
    cs = res["cold_start"]
    if "verdict" in cs:
        print("\n[wp1] cold start: signed 50-rule bundle -> verified policy -> Shield")
        for name, b in cs["bundles"].items():
            ip = cs["in_process"][name]
            say(f"{name}: in process, median / p95 (first call)",
                f"{ip['total']['median_ms']:.1f} / {ip['total']['p95_ms']:.1f} ms "
                f"({ip['first_call_ms']:.1f} ms)  stages {ip['stages_ms_median']}")
            pp = cs["pure_python_verifier"][name]["total"]
            say(f"{name}: in process, pure-Python verifier",
                f"{pp['median_ms']:.1f} / {pp['p95_ms']:.1f} ms")
            fr = cs["fresh_process"].get(name)
            if fr:
                say(f"{name}: fresh process imports / path / ingest p95",
                    f"{fr['import']['p95_ms']:.0f} / {fr['path_first_call']['p95_ms']:.0f}"
                    f" / {fr['ingest_with_imports']['p95_ms']:.0f} ms (wall p95 "
                    f"{fr['whole_process_wall']['p95_ms']:.0f} ms, n={fr['runs']})")
            au = cs["author_side"].get(name)
            if au:
                say(f"{name}: author side YAML -> signed bundle",
                    f"{au['yaml_to_signed_bundle']['median_ms']:.1f} / "
                    f"{au['yaml_to_signed_bundle']['p95_ms']:.1f} ms")
            say(f"{name}: checks", cs["checks"][name])
        say("refusals through the same path", cs["refusals"])
        say("verdict", cs["verdict"])
    print("\n[wp1] identity")
    say("version lock problems", len(res["version_lock_problems"]))
    for x in res["version_lock_problems"]:
        say("  !", x)
    say("policy_dsl.schema.json current", res["schema"].get("current"))
    sv = res["schema_validation"]
    say("schema validation pass rate (canonical IR)",
        f"{sv['passed']}/{sv['n']}" if sv.get("run") else sv.get("note"))
    if sv.get("run"):
        say("schema: authored files / grant documents",
            f"{sv['authored']} / {sv['grant_documents']}")
        say("schema alone refuses (negative corpus)", sv["negative_refused_by_schema"])
    sr = res["stored_runs"]
    if sr.get("n"):
        say("stored manifests matched, before", f"{sr['matched_before']}/{sr['n']}")
        say("stored manifests matched, after", f"{sr['matched_after']}/{sr['n']}")
        say("after, by hash form", sr["after_by_form"])
        aside = sr.get("set_aside") or []
        say("set aside in demo/out/_*/ (not counted)",
            f"{len(aside)}, resolving {sum(a['resolves'] for a in aside)}/{len(aside)}: "
            + ", ".join(f"{a['folder']}/{a['run']}" for a in aside) if aside else "0")
        five = sr["mavros_dev_five"]
        say("SITL+MAVROS 2 runs (dev) matched before/after",
            f"{sum(v['matched_before'] for v in five.values())}/{len(five)} -> "
            f"{sum(v['matched_after'] for v in five.values())}/{len(five)}")
    else:
        say("stored manifests", sr.get("note"))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    try:
        shown = os.path.relpath(out, ROOT)
    except ValueError:                      # another drive on Windows
        shown = str(out)
    print(f"\nwrote {shown}")
    return 0 if res["round_trip"]["passed"] == res["round_trip"]["n"] else 1


if __name__ == "__main__":
    sys.exit(main())
