"""WP1's acceptance KPI, recorded: policy load round-trip, and what it is worth.

    python tools/wp1_roundtrip_kpi.py                 # writes docs/data/wp1_roundtrip.json
    python tools/wp1_roundtrip_kpi.py --out x.json    # anywhere else
    python tools/wp1_roundtrip_kpi.py --no-runs       # skip the stored-run census

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

Two identity checks ride along because they are the same question asked of
stored artefacts: the JSON Schema export is current, and every stored flight
manifest still resolves to a policy (the census, before and after the
2026-10-06 identity release). Every number printed here is the number written.
"""
from __future__ import annotations

import argparse
import glob
import io
import json
import os
import platform
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
REFERENCE_REPO = ROOT / "kuanting-vla-uav-guardrail"
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
        return {"run": False, "note": f"no reference virtualenv at "
                                      f"{REFERENCE_PY.relative_to(ROOT).as_posix()}"}
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
    grant_items = [(d.relative_to(ROOT).as_posix(), lambda d=d: _authored(d))
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
          runs: bool = True, signer="auto", reference: bool = True) -> dict:
    rt = round_trip(policies_dir, signer=signer)
    neg = negative_corpus(neg_dir)
    gf = grant_form_round_trip(policies_dir)
    xref = reference_cross_load(run_reference_loader=reference)
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
        },
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
            "schema_authored": "28/29"},
        "round_trip": rt,
        "negative_corpus": neg,
        "grant_form": gf,
        "reference": xref,
        "version_lock_problems": check_lock(policies_dir),
        "schema": schema_status(),
        "schema_validation": schema_validation(policies_dir),
        "stored_runs": stored_run_census(ROOT) if runs else {"note": "skipped (--no-runs)"},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--no-runs", action="store_true")
    ap.add_argument("--no-reference", action="store_true",
                    help="do not run the reference implementation's loader")
    args = ap.parse_args(argv)
    res = build(runs=not args.no_runs, reference=not args.no_reference)

    print("\n[wp1] policy load round-trip KPI")
    for k, v in res["kpi"].items():
        say(k, v)
    say("null: accept everything", {k: v for k, v in res["null_accept_everything"].items()
                                     if k != "why"})
    say("null: the 2026-10-05 code", res["null_2026_10_05"])
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
