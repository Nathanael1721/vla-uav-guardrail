"""Re-score the AerialVLA follow runs flown before the review of d6e7123 (P1-1).

    python tools/rescore_vla_follow.py demo/out/vla_follow_redcar_0*_20261023 --dry-run
    python tools/rescore_vla_follow.py demo/out/vla_follow_redcar_0*_20261023

WHY

demo/vla_fly.py --mission follow wrote the follow's score only under
metrics["follow"]. guardrail/kpi.py reads `frac_within_30m` and
`follow_scored` at the TOP of metrics.json (follow_vlm and perception_node
write them there), found neither, applied no follow test, and every follow's
kpi.json said mission_success true - runs #2 and #3 of 23 Oct with 0.0 of
their ticks within 30 m of the car. tools/kpi_report.py filed them under
"other" and tools/build_dashboard.py showed no follow share, for the same
reason. The flight now publishes them (vla_mission.follow_kpi_fields); this
tool gives the runs flown before the fix the same records, from their own
flight_log.jsonl - every row carries `subject_dist_m`, the level car's
truth the flight read for scoring.

WHAT IT DOES, per run folder

  1. Refuses anything that is not a VLA follow (metrics.mission.kind
     "follow") or has no flight-log rows.
  2. vla_mission.follow_scores over the rows: metrics["follow"] gains
     `frac_within_30m`, `truth_coverage` and follow_vlm's `follow_scored`
     (the truth on FOLLOW_TRUTH_MIN_COVERAGE of the ticks); what the flight
     recorded there (the level car, its truth errors) stays. The top of
     metrics gets `frac_within_30m` and `follow_scored`.
  3. Loads the policy the run flew - metrics.policy_source.path, or
     --policy - and refuses unless its hash is the run's
     (policy_source.loaded_policy_hash, else manifest.json policy_hash).
  4. Recomputes kpi.json exactly as the flight does (vla_mission.run_kpis)
     and refuses to write unless the stored P0 figures
     (p0_violation_escape_rate, p0_escapes, p0_violation_ticks) come back
     identical: only the mission verdict may move.
  5. Keeps every superseded value under "rescore" in both files, with the
     date, this tool and why.
  6. Re-writes the run's replay bundle <tag>.replay.tar.gz over the
     re-scored records and verifies it (guardrail/replay.py), the as-flown
     bundle kept beside it as <tag>.replay.tar.gz.pre-rescore - so the
     bundle still re-derives the kpi.json beside it (--no-rebundle skips it).

A second run finds nothing to change ("unchanged"). Nothing outside the run
folders named is read or written, apart from the policy file.
"""
from __future__ import annotations

import argparse
import copy
import datetime
import glob
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_mission as VM                                             # noqa: E402

TOOL = "tools/rescore_vla_follow.py"
WHY = ("review of d6e7123, P1-1: the follow's score sat only under metrics['follow'], "
       "where guardrail/kpi.py does not read it, so kpi.json applied no follow test "
       "(mission_success true whatever the distance to the subject)")
P0_FIELDS = ("p0_violation_escape_rate", "p0_escapes", "p0_violation_ticks")


def _read_json(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


def _rows(run: Path) -> list[dict]:
    p = run / "flight_log.jsonl"
    if not p.is_file():
        return []
    return [json.loads(x) for x in p.read_text(encoding="utf-8").splitlines() if x.strip()]


def resolve_policy(metrics: dict, manifest: dict | None, policy_path=None):
    """(the policy object the run flew, its file). From --policy or
    metrics.policy_source.path (a signed bundle *.tar.gz or a YAML file;
    relative paths from the repository root). ValueError when there is
    none, or when its hash is not the run's."""
    src = metrics.get("policy_source") or {}
    want = src.get("loaded_policy_hash") or (manifest or {}).get("policy_hash")
    path = policy_path or src.get("path")
    if not path:
        raise ValueError("metrics.policy_source names no policy file: pass --policy")
    p = Path(path)
    if not p.is_absolute():
        p = ROOT / p
    if p.name.endswith(".tar.gz"):
        from guardrail.bundle import check_bundle
        chk = check_bundle(p)
        pol = chk.flight_policy or chk.policy
    else:
        from guardrail.models import load_policy
        pol = load_policy(p)
    if not want:
        raise ValueError("the run records no policy hash to check the policy against")
    if pol.policy_hash != want:
        raise ValueError(f"{p} is policy {pol.policy_hash}; the run flew {want}")
    return pol, p


def _without(d: dict, *keys) -> dict:
    return {k: v for k, v in d.items() if k not in keys}


def rescore(run, *, write: bool = True, policy_path=None, rebundle: bool = True,
            today: str | None = None) -> dict:
    """Re-score one follow run folder (module docstring). Returns a report:
    `status` "rescored" / "would_change" (write False) / "unchanged" /
    "refused", `why` when refused, and the moved verdicts as [old, new]."""
    run = Path(run)
    rep: dict = {"run": str(run), "status": None}

    def refuse(why: str) -> dict:
        rep.update(status="refused", why=why)
        return rep
    try:
        metrics = _read_json(run / "metrics.json")
        stored_kpi = _read_json(run / "kpi.json")
    except (OSError, ValueError) as exc:
        return refuse(f"metrics.json / kpi.json unreadable: {type(exc).__name__}: {exc}")
    manifest = None
    if (run / "manifest.json").is_file():
        manifest = _read_json(run / "manifest.json")
    else:
        manifest = stored_kpi.get("manifest")
    if not isinstance(manifest, dict):
        return refuse("no manifest.json (and none in kpi.json)")
    if (metrics.get("mission") or {}).get("kind") != "follow":
        return refuse(f"not a VLA follow (metrics.mission.kind "
                      f"{(metrics.get('mission') or {}).get('kind')!r})")
    rows = _rows(run)
    if not rows:
        return refuse("flight_log.jsonl holds no rows")
    try:
        policy, ppath = resolve_policy(metrics, manifest, policy_path)
    except Exception as exc:                                       # noqa: BLE001
        return refuse(f"policy: {type(exc).__name__}: {exc}")
    rep["policy"] = str(ppath)

    scores = VM.follow_scores(rows)
    new_m = copy.deepcopy(metrics)
    old_follow = metrics.get("follow") if isinstance(metrics.get("follow"), dict) else {}
    new_m["follow"] = {**old_follow, **scores}
    new_m.update(VM.follow_kpi_fields(scores))
    new_k = VM.run_kpis(rows, new_m, manifest, policy)
    for f in P0_FIELDS:
        if new_k.get(f) != stored_kpi.get(f):
            return refuse(f"the P0 figures do not come back: {f} stored "
                          f"{stored_kpi.get(f)!r}, recomputed {new_k.get(f)!r} - nothing written")
    new_m["kpi_grade"] = new_k["kpi_grade"]
    new_m["p0_violation_escape_rate"] = new_k.get("p0_violation_escape_rate")
    old_k_cmp = _without(stored_kpi, "rescore")
    k_changed = sorted(k for k in set(old_k_cmp) | set(new_k)
                       if old_k_cmp.get(k) != new_k.get(k))
    m_keys = ("frac_within_30m", "follow_scored", "follow", "kpi_grade",
              "p0_violation_escape_rate")
    m_changed = [k for k in m_keys if metrics.get(k) != new_m.get(k)]
    rep["kpi"] = {k: [stored_kpi.get(k), new_k.get(k)]
                  for k in ("mission_success", "outcome", "mission_fail_reasons")}
    rep["metrics"] = {k: [metrics.get(k), new_m.get(k)] for k in ("frac_within_30m",
                                                                  "follow_scored")}
    rep["kpi_changed"], rep["metrics_changed"] = k_changed, m_changed
    if not k_changed and not m_changed:
        rep["status"] = "unchanged"
        return rep
    if not write:
        rep["status"] = "would_change"
        return rep
    date = today or datetime.date.today().isoformat()
    prior_m = metrics.get("rescore") if isinstance(metrics.get("rescore"), dict) else None
    new_m["rescore"] = {"date": date, "tool": TOOL, "why": WHY,
                        "superseded": {"metrics": {k: metrics.get(k) for k in m_changed}},
                        **({"earlier": prior_m} if prior_m else {})}
    new_k["rescore"] = {"date": date, "tool": TOOL, "why": WHY,
                        "superseded": {k: stored_kpi.get(k) for k in k_changed},
                        **({"earlier": stored_kpi["rescore"]}
                           if isinstance(stored_kpi.get("rescore"), dict) else {})}
    new_m["rescore"]["superseded"]["kpi"] = new_k["rescore"]["superseded"]
    VM.write_json(run / "metrics.json", new_m)
    VM.write_json(run / "kpi.json", new_k)
    rep["status"] = "rescored"
    if rebundle:
        rep["replay"] = _rebundle(run, new_m, policy)
    return rep


def _rebundle(run: Path, metrics: dict, policy) -> dict:
    """The run's replay bundle over the re-scored records, verified; the
    as-flown one kept as <name>.pre-rescore (the first one only)."""
    from guardrail.replay import verify_replay, write_replay
    tag = metrics.get("tag") or run.name
    bp = run / f"{tag}.replay.tar.gz"
    kept = bp.with_name(bp.name + ".pre-rescore")
    out = {"bundle": bp.name, "as_flown_kept": None, "verified": None, "why": None}
    if bp.is_file() and not kept.exists():
        bp.replace(kept)
        out["as_flown_kept"] = kept.name
    elif kept.exists():
        out["as_flown_kept"] = kept.name
    try:
        write_replay(run, policy, bp, changelog=f"re-scored by {TOOL}: {WHY}",
                     policy_source=metrics.get("policy_source"))
        ok, why = verify_replay(bp)
        out.update(verified=bool(ok), why=None if ok else why)
    except Exception as exc:                                       # noqa: BLE001
        out.update(verified=False, why=f"{type(exc).__name__}: {exc}")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("runs", nargs="+", help="run folders (demo/out/<tag>)")
    ap.add_argument("--dry-run", action="store_true", help="say what would change, write nothing")
    ap.add_argument("--policy", default=None,
                    help="the policy file the runs flew (default: metrics.policy_source.path)")
    ap.add_argument("--no-rebundle", action="store_true",
                    help="leave each run's replay bundle as flown")
    a = ap.parse_args(argv)
    runs = []
    for r in a.runs:
        # PowerShell hands a pattern over unexpanded: expand it here.
        runs += sorted(glob.glob(r)) if any(c in r for c in "*?[") else [r]
    if not runs:
        print(f"no run folder matches {a.runs}", file=sys.stderr)
        return 1
    refused = 0
    for r in runs:
        rep = rescore(r, write=not a.dry_run, policy_path=a.policy,
                      rebundle=not a.no_rebundle)
        refused += rep["status"] == "refused"
        print(json.dumps(rep, ensure_ascii=False))
    return 1 if refused else 0


if __name__ == "__main__":
    sys.exit(main())
