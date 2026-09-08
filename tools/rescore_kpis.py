"""Add the two newly-measured KPIs to flights that were flown before they existed.

    python tools/rescore_kpis.py --dry-run     # show what would change
    python tools/rescore_kpis.py               # write it

WHY THIS IS A TOOL AND NOT A ONE-OFF

`mean repair magnitude` and `mean time to safe` are two of the grant's five
acceptance KPIs and were never computed (see `guardrail/kpi.py`). Every field
they need has been in `flight_log.jsonl` all along, so the 42 delivered runs can
answer for themselves without being re-flown. That will be true again the next
time a KPI is added, and the September deck reads its numbers from these
artefacts at build time - so the recomputation has to be repeatable rather than
something typed once into a shell.

WHAT IT WILL NOT DO

It only ADDS fields. Existing numbers - above all `p0_violation_escape_rate` -
are never overwritten, because a tool that silently rewrites delivered
contractual figures is a tool that can quietly improve them.

Where the run's `policy_hash` matches a policy file we still hold, the full KPI
set is recomputed as a CHECK: the stored P0 figures must come back identical. A
mismatch is reported loudly and the file is left alone. Runs whose hash does not
match any policy on disk - the `--dynamic` ones, where a mid-flight hot-apply
restamped the hash by design - simply cannot be re-derived, and are reported as
unverified rather than assumed fine.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K, load_policy                        # noqa: E402
from guardrail.models import State                                 # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

# Fields this tool is allowed to introduce. Anything already present in a
# kpi.json is left exactly as delivered, even if recomputation disagrees -
# the disagreement is reported instead.
NEW_FIELDS = (
    "mean_repair_magnitude_mps", "max_repair_magnitude_mps",
    "mean_yaw_repair_dps", "repaired_ticks", "repair_ticks_not_measurable",
    "mean_time_to_safe_s", "max_time_to_safe_s", "time_to_safe_episodes",
    "time_to_safe_censored", "time_to_safe_not_measurable",
    "time_to_safe_reconstructed",
    "mean_intervention_s", "max_intervention_s", "intervention_episodes",
    "intervention_censored", "intervention_not_measurable",
)

# The figures that must not move. If recomputation changes one of these, the
# artefact and the code have diverged and a human needs to look.
INVARIANTS = ("p0_violation_escape_rate", "p0_escapes", "p0_violation_ticks",
              "p0_ticks_not_measurable", "failsafe_trigger_correctness")


def policies_by_hash() -> dict[str, object]:
    """policy_hash -> loaded Policy, for every policy file we still hold."""
    out = {}
    for p in sorted((ROOT / "policies").glob("*.yaml")):
        try:
            pol = load_policy(p)
        except Exception as e:                                   # noqa: BLE001
            print(f"  ! {p.name}: will not load ({type(e).__name__}: {e})")
            continue
        out[pol.policy_hash] = pol
    return out


def rescore(run: Path, by_hash: dict) -> dict | None:
    """Return a report row for one run directory, or None if it has no KPI."""
    kpi_p, log_p = run / "kpi.json", run / "flight_log.jsonl"
    if not kpi_p.is_file() or not log_p.is_file():
        return None

    stored = json.loads(kpi_p.read_text(encoding="utf-8"))
    rows = [json.loads(x) for x in
            log_p.read_text(encoding="utf-8").splitlines() if x.strip()]
    if not rows:
        return {"run": run.name, "status": "empty log"}

    # The new KPIs need no rule priorities: a repair magnitude and a recovery
    # time are the same numbers whatever the rule was called. So they can be
    # computed for every run, including the ones whose policy we cannot resolve.
    metrics = {}
    mp = run / "metrics.json"
    if mp.is_file():
        metrics = json.loads(mp.read_text(encoding="utf-8"))

    pol_hash = None
    man = run / "manifest.json"
    if man.is_file():
        pol_hash = json.loads(man.read_text(encoding="utf-8")).get("policy_hash")
    pol = by_hash.get(pol_hash)

    # `unsafe` - was the POSITION illegal on this tick - postdates every
    # delivered flight, so reconstruct it from the logged pose. Only possible
    # where the policy resolves; without it the run honestly reports
    # time_to_safe_not_measurable rather than a flattering zero.
    reconstructed = False
    if pol is not None and any(r.get("x") is not None for r in rows):
        sh = Shield(pol, lookahead_s=3.0, dt=0.5)
        for r in rows:
            if r.get("x") is None or r.get("up") is None:
                continue
            # A stand-off rule is inert until the Shield is told where the
            # subject is, exactly as in flight - and it binds PER CLASS, so the
            # class has to come too. Without it only the "*" catch-all binds and
            # the 10 m pedestrian ring can never fire during a rescore, which is
            # the same inert-rule failure this project already flew once.
            #
            # Rows written from 2026-09-07 carry truth: {class, pts}; the class
            # is what selects the rule and the first point is the subject the
            # Shield was actually given. Older rows have only tgt_x/tgt_y, which
            # is the car - correct for a flight that never retargeted, and all
            # of them are.
            truth = r.get("truth") or {}
            pts = truth.get("pts") or []
            if pts:
                sh.set_subject(pts[0][0], pts[0][1], truth.get("class"))
            elif r.get("tgt_x") is not None and r.get("tgt_y") is not None:
                sh.set_subject(r["tgt_x"], r["tgt_y"], truth.get("class"))
            st = State(x=r["x"], y=r["y"], up=r["up"],
                       yaw_deg=r.get("yaw_deg", 0.0) or 0.0)
            r["unsafe"] = bool(sh.state_is_unsafe(st))
        reconstructed = True

    fresh = K.compute(rows, K.rule_priorities(pol) if pol else {}, metrics)
    fresh["time_to_safe_reconstructed"] = reconstructed

    # Invariant check, only where the policy is genuinely resolvable.
    drift = []
    if pol is not None:
        for f in INVARIANTS:
            if f in stored and stored[f] != fresh.get(f):
                drift.append(f"{f}: {stored[f]} -> {fresh.get(f)}")

    added = {f: fresh[f] for f in NEW_FIELDS
             if f in fresh and f not in stored}
    return {
        "run": run.name,
        "verified": pol is not None,
        "drift": drift,
        "added": added,
        "stored": stored,
        "path": kpi_p,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--roots", nargs="*",
                    default=[str(ROOT / "demo" / "out"), str(ROOT / "sitl" / "out")])
    args = ap.parse_args()

    by_hash = policies_by_hash()
    print(f"{len(by_hash)} policy files loaded\n")

    rows, drifted, unverified, written = [], [], 0, 0
    for root in args.roots:
        rp = Path(root)
        if not rp.is_dir():
            continue
        for run in sorted(p for p in rp.iterdir() if p.is_dir()):
            r = rescore(run, by_hash)
            if r is None or "status" in r:
                continue
            rows.append(r)
            if r["drift"]:
                drifted.append(r)
            if not r["verified"]:
                unverified += 1
            if r["added"] and not args.dry_run and not r["drift"]:
                merged = dict(r["stored"], **r["added"])
                # allow_nan=False: Python writes a bare `NaN` token that is not
                # valid JSON, so an artefact carrying one is readable by Python
                # and by nothing else — including the deck build, which is where
                # exactly that was caught.
                r["path"].write_text(
                    json.dumps(merged, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")
                written += 1

    print(f"{'run':<22} {'repairs':>8} {'mean m/s':>9} {'max m/s':>8} "
          f"{'t2safe':>8} {'interv':>8} {'cens':>5}  chk")
    for r in rows:
        v = dict(r["stored"], **r["added"])
        mm = v.get("mean_repair_magnitude_mps")
        mx = v.get("max_repair_magnitude_mps")
        ts = v.get("mean_time_to_safe_s")
        iv = v.get("mean_intervention_s")
        print(f"{r['run']:<22} {v.get('repaired_ticks', 0):>8} "
              f"{('-' if mm is None else f'{mm:.3f}'):>9} "
              f"{('-' if mx is None else f'{mx:.3f}'):>8} "
              f"{('-' if ts is None else f'{ts:.2f}'):>8} "
              f"{('-' if iv is None else f'{iv:.2f}'):>8} "
              f"{v.get('time_to_safe_censored', 0):>5}  "
              f"{'ok' if r['verified'] else '??'}")

    print(f"\n{len(rows)} runs scored, {written} files updated"
          f"{' (dry run)' if args.dry_run else ''}")
    print(f"{unverified} could not be re-derived (policy_hash not on disk) - "
          f"their new fields are still valid, their P0 figures are unchecked")
    if drifted:
        print(f"\n!! {len(drifted)} runs DISAGREE with their stored figures - "
              f"not written, needs a human:")
        for r in drifted:
            print(f"   {r['run']}: {'; '.join(r['drift'])}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
