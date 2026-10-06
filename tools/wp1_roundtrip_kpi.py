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
                   that load_policy() should refuse. The ones it ACCEPTS are
                   findings, listed by name - this unit does not change
                   validation strictness, it records it.

  null "accept everything": round_trip N/N, tamper_refusal 0/N, negative 0/M.

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
from guardrail.manifest import code_revision, resolve_run_policy     # noqa: E402
from guardrail.models import HASH_SCHEME, load_policy                # noqa: E402

DEFAULT_OUT = ROOT / "docs" / "data" / "wp1_roundtrip.json"
NEGATIVE_DIR = ROOT / "policies" / "negative"


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
            pol = load_policy(f)
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
            load_policy(f)
            verdict, reason = "ACCEPTED", None
        except Exception as exc:                               # noqa: BLE001
            verdict = "refused"
            reason = f"{type(exc).__name__}: {str(exc).splitlines()[0][:140]}"
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
            "mavros_dev_five": five,
            "before_means": ("Policy.policy_hash as computed by the code before "
                             "2026-10-06 (16 hex over model_dump() with nulls), "
                             "against every policies/*.yaml"),
            "after_means": ("guardrail.manifest.resolve_run_policy over "
                            "guardrail.bundle.policy_candidates(): the 64-hex "
                            "hash, every legacy 16-hex form, and the hot-applied "
                            "derivations in policies/policy.lock.json")}


# --------------------------------------------------------------------------- #

def schema_status() -> dict:
    want = policy_json_schema()
    if not SCHEMA_PATH.is_file():
        return {"path": SCHEMA_PATH.relative_to(ROOT).as_posix(), "current": False,
                "note": "missing - run python -m guardrail.bundle schema"}
    have = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    return {"path": SCHEMA_PATH.relative_to(ROOT).as_posix(), "current": have == want,
            "definitions": len(want.get("$defs", {}))}


def schema_validation(policies_dir: Path) -> dict:
    """The grant's DSL KPI 'schema validation pass rate', against the exported
    JSON Schema. Needs jsonschema; reported as not run without it."""
    try:
        import jsonschema
    except ImportError:
        return {"run": False, "note": "jsonschema not installed in this interpreter"}
    schema = policy_json_schema()
    rows = []
    for f in sorted(policies_dir.glob("*.yaml")):
        try:
            jsonschema.validate(load_policy(f).canonical_ir(), schema)
            rows.append({"file": f.name, "valid": True})
        except Exception as exc:                               # noqa: BLE001
            rows.append({"file": f.name, "valid": False,
                         "error": f"{type(exc).__name__}: {str(exc)[:120]}"})
    ok = sum(r["valid"] for r in rows)
    return {"run": True, "n": len(rows), "passed": ok,
            "failures": [r for r in rows if not r["valid"]]}


def build(policies_dir: Path = ROOT / "policies", neg_dir: Path = NEGATIVE_DIR,
          runs: bool = True, signer="auto") -> dict:
    rt = round_trip(policies_dir, signer=signer)
    neg = negative_corpus(neg_dir)
    n = rt["n"]
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
        },
        "null_accept_everything": {
            "round_trip": f"{n}/{n}", "tamper_refusal": f"0/{rt['tamper_refusal']['run']}",
            "negative_refusal": f"0/{neg['n']}",
            "why": "a reader that verifies nothing passes every round trip; only "
                   "the refusal columns separate it from a real one"},
        "round_trip": rt,
        "negative_corpus": neg,
        "version_lock_problems": check_lock(policies_dir),
        "schema": schema_status(),
        "schema_validation": schema_validation(policies_dir),
        "stored_runs": stored_run_census(ROOT) if runs else {"note": "skipped (--no-runs)"},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--no-runs", action="store_true")
    args = ap.parse_args(argv)
    res = build(runs=not args.no_runs)

    print("\n[wp1] policy load round-trip KPI")
    for k, v in res["kpi"].items():
        say(k, v)
    say("null: accept everything", {k: v for k, v in res["null_accept_everything"].items()
                                     if k != "why"})
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
    print("\n[wp1] identity")
    say("version lock problems", len(res["version_lock_problems"]))
    for x in res["version_lock_problems"]:
        say("  !", x)
    say("policy_dsl.schema.json current", res["schema"].get("current"))
    sv = res["schema_validation"]
    say("schema validation pass rate",
        f"{sv['passed']}/{sv['n']}" if sv.get("run") else sv.get("note"))
    sr = res["stored_runs"]
    if sr.get("n"):
        say("stored manifests matched, before", f"{sr['matched_before']}/{sr['n']}")
        say("stored manifests matched, after", f"{sr['matched_after']}/{sr['n']}")
        say("after, by hash form", sr["after_by_form"])
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
