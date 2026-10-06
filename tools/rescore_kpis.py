"""Add newly-measured KPIs to flights that were flown before they existed.

    python tools/rescore_kpis.py --dry-run     # show what would change
    python tools/rescore_kpis.py               # write it
    python tools/rescore_kpis.py --only 'ros2_*' 'sitl_*'

WHY THIS IS A TOOL AND NOT A ONE-OFF

`mean repair magnitude` and `mean time to safe` are two of the grant's five
acceptance KPIs and were never computed (see `guardrail/kpi.py`). Every field
they need has been in `flight_log.jsonl` all along, so the 42 delivered runs can
answer for themselves without being re-flown. That was true again on
2026-10-06 for the repair success rate (Safety Shield page) and the false-trigger
half of fail-safe correctness (Stress Testing page), and the September deck
reads its numbers from these artefacts at build time - so the recomputation has
to be repeatable rather than something typed once into a shell.

WHAT IT WILL NOT DO

It only ADDS fields. Existing numbers - above all `p0_violation_escape_rate` -
are never overwritten, because a tool that silently rewrites delivered
contractual figures is a tool that can quietly improve them.

ONE EXCEPTION, AND WHY: the time-to-safe fields that THIS TOOL reconstructed
(`time_to_safe_reconstructed: true` in the stored table). They were never
delivered by a flight; they are this tool's own earlier answer, and on
2026-10-06 one of them was found to be wrong. `ros2_ped_off` stored
`time_to_safe_episodes: 0` while its own metrics.json records 2.3 s inside the
10 m pedestrian ring (`standoff_s`). The SITL rails declare the subject in
metrics.json (`subject: {class, x, y, position_source: declared}`) and never
write it per row, and the reconstruction read the subject only from per-row
`truth` / `tgt_x` - so the stand-off rule was inert for the whole rescore and
the one run that breached it scored clean. A wrong answer this tool wrote is
corrected by this tool, with the old values kept under
`time_to_safe_superseded` so the correction is visible rather than silent -
but ONLY where the declared-subject fix explains the difference and the
rail's independently measured dwell agrees. "Explains" is tested, not
assumed: the reconstruction is run again WITHOUT the declared subject, and it
must give back the stored values exactly (`_subject_explains`). A
reconstruction that moved for any other reason (retarget_demo and
retarget_demo2 do, under the pre-fix tool too: Shield/IR code changed what
"unsafe" means since their rescore) is reported for a human and left alone.

A SECOND, NARROWER EXCEPTION: three false-trigger fields this tool wrote on
2026-10-06 under a definition withdrawn the same day (`RETIRED_FIELDS`). They
are removed on rewrite, with their values kept under `rescore.retired`.

POLICY RESOLUTION

Where the run's `policy_hash` matches a policy file we still hold, the full KPI
set is recomputed as a CHECK: the stored P0 figures must come back identical. A
mismatch is reported loudly and the file is left alone.

A hash can stop matching without a byte of YAML changing - the IR gained
optional fields on 2026-09-01 and every policy re-fingerprinted (audit card
X-09). Two fallbacks, both weaker than a hash and both labelled as such:

  * the policy's legacy short hashes, where `guardrail.models.Policy` can
    produce them (`policy_hash_short`, `legacy_hashes()`);
  * the `policy_id` the run recorded at takeoff (prompt.yaml), only when
    exactly one policy file carries it AND the takeoff hash equals the
    manifest hash. A run whose hash moved in flight - the `--dynamic` ones,
    where a hot-apply restamped it by design - is NOT resolved this way: the
    file on disk lacks the zone that was added mid-flight, and reconstructing
    against it would under-report the time spent unsafe.

An id-resolved run must reproduce its stored P0 figures AND its reconstructed
dwell must agree with the dwell the rail measured independently
(`nfz_s`, `alt_violation_s`, `standoff_s`), or nothing is written.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import statistics
import sys
from datetime import date
from pathlib import Path

import yaml

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
    # 2026-10-06: repair success rate (Safety Shield, acceptance KPIs) ...
    "repair_success_rate", "repair_success_status", "repair_attempt_ticks",
    "repair_success_ticks", "repair_unmeasured_ticks", "repair_outcomes",
    "repair_not_applied_ticks", "repairs_to_standstill",
    "repairs_converged_while_unsafe", "repair_success_basis",
    "repair_theta_unchecked_ticks", "repair_theta_sources", "repair_theta_m",
    "failsafe_instrumented",
    # ... the counts behind fail-safe correctness, and its false-trigger half
    "failsafe_expected_ticks", "failsafe_correct_ticks",
    "violation_free_ticks", "false_trigger_ticks", "false_trigger_rate",
    "failsafe_permitted_ticks",
    # ... and why a mission failed
    "mission_goal_status", "mission_fail_reasons", "unsafe_p0_ticks",
    "mission_started_unsafe",
)

# New fields whose value depends on rule PRIORITIES (P0 vs P1). Without a
# resolved policy every rule defaults to P0, which would count a P1-only tick as
# "a fail-safe was expected here" - so these are not written for a run whose
# policy cannot be resolved. The repair success rate itself does not depend on
# priority (a residual violation of either level is a non-converged repair),
# and neither does the false-trigger half: its denominator is ticks that broke
# no rule at all.
NEEDS_PRIORITIES = (
    "failsafe_expected_ticks", "failsafe_correct_ticks",
    "failsafe_permitted_ticks", "repair_outcomes",
    "mission_fail_reasons", "unsafe_p0_ticks", "mission_started_unsafe",
)

# Fields this tool itself wrote on 2026-10-06 under a definition withdrawn the
# same day: the false-trigger half then counted a brake on a P1-only tick as
# false, which the grant's FSM permits (Safety Shield p4). They are this tool's
# own answer, never a flight's, so a rewrite removes them (recorded under
# `rescore.retired`) instead of leaving a field whose name now means something
# else. Only in a table that carries this tool's `rescore` block of that date.
RETIRED_FIELDS = ("failsafe_not_expected_ticks", "failsafe_false_triggers",
                  "failsafe_false_trigger_rate")
RETIRED_ON = "2026-10-06"

# The time-to-safe fields this tool reconstructs, and may therefore correct
# when its own earlier reconstruction was wrong. See the module docstring.
CORRECTABLE = (
    "mean_time_to_safe_s", "max_time_to_safe_s", "time_to_safe_episodes",
    "time_to_safe_censored", "time_to_safe_not_measurable",
)

# The figures that must not move. If recomputation changes one of these, the
# artefact and the code have diverged and a human needs to look.
INVARIANTS = ("p0_violation_escape_rate", "p0_escapes", "p0_violation_ticks",
              "p0_ticks_not_measurable", "failsafe_trigger_correctness")

# metrics.json dwell field <- the Violation.category whose unsafe ticks it counts
DWELL_FIELDS = {"nfz_s": "geofence", "alt_violation_s": "altitude",
                "standoff_s": "standoff"}

_HASH_MIN = len("sha256:") + 16


# --------------------------------------------------------------------------- #
# policies
# --------------------------------------------------------------------------- #

def load_policies() -> list[tuple[Path, object]]:
    out = []
    for p in sorted((ROOT / "policies").glob("*.yaml")):
        try:
            out.append((p, load_policy(p)))
        except Exception as e:                                   # noqa: BLE001
            print(f"  ! {p.name}: will not load ({type(e).__name__}: {e})")
    return out


def _hash_keys(pol) -> set[str]:
    """Every hash string a stored run may carry for this policy: the current
    one, its short form, and the legacy forms where the model can produce them
    (guarded - the policy-identity work adding them may not be present)."""
    keys = {pol.policy_hash}
    short = getattr(pol, "policy_hash_short", None)
    if isinstance(short, str):
        keys.add(short)
    legacy = getattr(pol, "legacy_hashes", None)
    if callable(legacy):
        try:
            keys |= {h for h in (legacy() or {}).values() if isinstance(h, str)}
        except Exception:                                        # noqa: BLE001
            pass
    return keys


def policies_by_hash(loaded: list | None = None) -> dict[str, object]:
    """policy_hash -> loaded Policy, for every policy file we still hold."""
    out: dict[str, object] = {}
    for _, pol in (loaded if loaded is not None else load_policies()):
        for h in _hash_keys(pol):
            out.setdefault(h, pol)
    return out


def policies_by_id(loaded: list | None = None) -> dict[str, list]:
    """policy_id -> every loaded Policy carrying it (ambiguous when > 1)."""
    out: dict[str, list] = {}
    for _, pol in (loaded if loaded is not None else load_policies()):
        out.setdefault(pol.policy_id, []).append(pol)
    return out


def _lookup_hash(h: str | None, by_hash: dict):
    if not h:
        return None
    if h in by_hash:
        return by_hash[h]
    # A 16-hex stored hash against a 64-hex current one (or the reverse) is the
    # same SHA-256 truncated - accept a prefix match, never a shorter one.
    for k, pol in by_hash.items():
        if min(len(k), len(h)) >= _HASH_MIN and (k.startswith(h) or h.startswith(k)):
            return pol
    return None


def takeoff_policy(run: Path) -> tuple[str | None, str | None]:
    """(policy_id, policy_hash) the run recorded at takeoff, from prompt.yaml."""
    p = run / "prompt.yaml"
    if not p.is_file():
        return None, None
    try:
        c = (yaml.safe_load(p.read_text(encoding="utf-8")) or {}).get("constraints") or {}
    except yaml.YAMLError:
        return None, None
    return c.get("policy_id"), c.get("policy_hash")


def resolve_policy(run: Path, manifest: dict, by_hash: dict,
                   by_id: dict | None) -> tuple[object | None, str | None, str]:
    """(policy, how, note). how is "hash", "policy_id" or None."""
    h = manifest.get("policy_hash")
    pol = _lookup_hash(h, by_hash)
    if pol is not None:
        return pol, "hash", ""
    if by_id is None:
        return None, None, "policy_hash not on disk"
    pid, takeoff_hash = takeoff_policy(run)
    if not pid:
        return None, None, "policy_hash not on disk; no takeoff policy_id recorded"
    cands = by_id.get(pid) or []
    if len(cands) != 1:
        return None, None, (f"policy_hash not on disk; policy_id {pid!r} names "
                            f"{len(cands)} policy files")
    if not takeoff_hash or takeoff_hash != h:
        return None, None, (f"policy_hash moved in flight ({takeoff_hash} at "
                            f"takeoff, {h} in the manifest): a hot-applied rule "
                            f"is not in any file, so the run cannot be re-derived")
    return cands[0], "policy_id", f"resolved by policy_id {pid!r}, not by hash"


# --------------------------------------------------------------------------- #
# reconstruction of `unsafe`
# --------------------------------------------------------------------------- #

def declared_subject(metrics: dict) -> tuple[float, float, str | None] | None:
    """The subject a SITL rail declared for the whole flight (metrics.json)."""
    s = metrics.get("subject")
    if isinstance(s, dict) and s.get("x") is not None and s.get("y") is not None:
        return float(s["x"]), float(s["y"]), s.get("class")
    return None


def subject_for_row(r: dict, declared) -> tuple[float, float, str | None, str] | None:
    """(x, y, class, source) the Shield was given on this tick, or None.

    Rows written from 2026-09-07 carry truth: {class, pts}; the class is what
    selects the rule and the first point is the subject the Shield was given.
    Older AirSim rows have only tgt_x/tgt_y, which is the car. The SITL rails
    write neither: their subject is DECLARED once, in metrics.json, and set on
    the Shield every tick - so that is what the Shield had, and the only
    honest fallback. Before 2026-10-06 that fallback did not exist, and a
    stand-off rule bound to a declared pedestrian could never fire on a rescore.
    """
    truth = r.get("truth") or {}
    pts = truth.get("pts") or []
    if len(pts) == 1:
        return pts[0][0], pts[0][1], truth.get("class"), "row_truth"
    if pts:
        # More than one candidate means the subject was a CLASS, and the log
        # does not record which member the tracker held. pts[0] is whichever
        # decoy spawned first, so serving it would reconstruct `unsafe` - and
        # therefore mean_time_to_safe_s, one of the grant's five KPIs -
        # against an arbitrary bystander. The nearest member is the only
        # defensible choice: it is the one a stand-off would bind hardest
        # against, so it cannot UNDER-report unsafe time, and under-reporting
        # is the direction that flatters.
        bx, by = min(pts, key=lambda q: (q[0] - r["x"]) ** 2 + (q[1] - r["y"]) ** 2)
        return bx, by, truth.get("class"), "row_truth_nearest"
    if r.get("tgt_x") is not None and r.get("tgt_y") is not None:
        return r["tgt_x"], r["tgt_y"], truth.get("class"), "row_tgt"
    if declared is not None:
        return declared[0], declared[1], declared[2], "metrics_declared"
    return None


def reconstruct_unsafe(rows: list[dict], pol, metrics: dict) -> dict:
    """Set r["unsafe"] on every row with a pose, exactly as in flight.

    Returns what it did: how many ticks were reconstructed, where each tick's
    subject came from, and the seconds spent unsafe per rule category - the
    last so the result can be checked against the rail's own dwell figures.
    """
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    declared = declared_subject(metrics)
    sources: dict[str, int] = {}
    cat_ticks: dict[str, int] = {}
    n = 0
    for r in rows:
        if r.get("x") is None or r.get("y") is None or r.get("up") is None:
            continue
        subj = subject_for_row(r, declared)
        if subj is None:
            sh.set_subject(None)
            sources["none"] = sources.get("none", 0) + 1
        else:
            sh.set_subject(subj[0], subj[1], subj[2])
            sources[subj[3]] = sources.get(subj[3], 0) + 1
        st = State(x=r["x"], y=r["y"], up=r["up"],
                   yaw_deg=r.get("yaw_deg", 0.0) or 0.0)
        bad = sh.state_is_unsafe(st)
        r["unsafe"] = bool(bad)
        # Which rules, so kpi.compute can tell a P0 position violation (fails
        # the mission) from a P1 one (does not).
        r["unsafe_rules"] = sorted({v.rule_id for v in bad})
        for c in {v.category for v in bad}:
            cat_ticks[c] = cat_ticks.get(c, 0) + 1
        n += 1
    ts = [float(r["t"]) for r in rows if r.get("t") is not None]
    dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    tick = statistics.median(dts) if dts else None
    return {"ticks": n, "subject_source": sources, "tick_s": tick,
            "unsafe_s_by_category": ({c: round(k * tick, 2) for c, k in cat_ticks.items()}
                                     if tick else {})}


def dwell_crosscheck(info: dict, metrics: dict) -> dict:
    """Reconstructed seconds unsafe vs the rail's independently measured dwell.

    A gross-disagreement guard, not a precision claim. The rails count
    trajectory samples x 0.1 s against their own geometry; the reconstruction
    counts logged ticks against the Shield's. They are known to differ by a few
    ticks - ros2_shield_off: 41 unsafe ticks vs nfz_s 3.7 s - so agreement is
    within max(0.5 s, 25 %). What it catches is the failure that motivated it:
    0.0 reconstructed against 2.3 s measured.
    """
    out, ok = {}, True
    for field, cat in DWELL_FIELDS.items():
        rail = metrics.get(field)
        if rail is None:
            continue
        rec = (info.get("unsafe_s_by_category") or {}).get(cat, 0.0)
        agree = abs(rec - rail) <= max(0.5, 0.25 * rail)
        out[field] = {"rail_s": rail, "reconstructed_s": rec, "agree": agree}
        ok = ok and agree
    return {"fields": out, "agree": ok if out else None}


# --------------------------------------------------------------------------- #
# one run
# --------------------------------------------------------------------------- #

def _read_json(p: Path) -> dict:
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def _subject_explains(rows: list[dict], pol, metrics: dict, stored: dict,
                      prios: dict) -> bool:
    """Does the declared-subject fix ALONE explain a moved time to safe?

    Rerun the reconstruction exactly as the pre-fix tool did - the same rows,
    the same policy, the metrics without `subject` - and require it to give
    back the stored time-to-safe fields to the last digit. Only then is the
    difference the one defect this tool can name. "Some tick was served the
    declared subject" (the first version's test) is not enough: a run whose
    difference ALSO comes from Shield / IR code that changed since would be
    auto-corrected under a reason that is only half true.
    """
    again = [{k: v for k, v in r.items() if k not in ("unsafe", "unsafe_rules")}
             for r in rows]
    bare = {k: v for k, v in metrics.items() if k != "subject"}
    reconstruct_unsafe(again, pol, bare)
    old_way = K.compute(again, prios, bare)
    return all(stored.get(f) == old_way.get(f) for f in CORRECTABLE if f in stored)


def rescore(run: Path, by_hash: dict, by_id: dict | None = None) -> dict | None:
    """Return a report row for one run directory, or None if it has no KPI."""
    kpi_p, log_p = run / "kpi.json", run / "flight_log.jsonl"
    if not kpi_p.is_file() or not log_p.is_file():
        return None

    stored = json.loads(kpi_p.read_text(encoding="utf-8"))
    rows = [json.loads(x) for x in
            log_p.read_text(encoding="utf-8").splitlines() if x.strip()]
    if not rows:
        return {"run": run.name, "status": "empty log"}

    metrics = _read_json(run / "metrics.json")
    manifest = _read_json(run / "manifest.json")
    pol, how, note = resolve_policy(run, manifest, by_hash, by_id)

    # `unsafe` - was the POSITION illegal on this tick - postdates every
    # delivered flight, so reconstruct it from the logged pose. Only possible
    # where the policy resolves; without it the run honestly reports
    # time_to_safe_not_measurable rather than a flattering zero.
    reconstructed, info, cross = False, {}, {"fields": {}, "agree": None}
    flight_logged_unsafe = any("unsafe" in r for r in rows)
    if (pol is not None and not flight_logged_unsafe
            and any(r.get("x") is not None for r in rows)):
        info = reconstruct_unsafe(rows, pol, metrics)
        cross = dwell_crosscheck(info, metrics)
        reconstructed = True

    prios = K.rule_priorities(pol) if pol else {}
    fresh = K.compute(rows, prios, metrics)
    fresh["time_to_safe_reconstructed"] = reconstructed

    # Invariant check, only where the policy is genuinely resolvable.
    drift = []
    if pol is not None:
        for f in INVARIANTS:
            if f in stored and stored[f] != fresh.get(f):
                drift.append(f"{f}: {stored[f]} -> {fresh.get(f)}")

    withheld = []
    # An id-resolved policy is a claim about which rules flew; the dwell the
    # rail measured independently is what corroborates it.
    id_unconfirmed = how == "policy_id" and cross["agree"] is False
    if id_unconfirmed:
        withheld.append("resolved by policy_id but the reconstructed dwell "
                        "disagrees with the rail's: " + json.dumps(cross["fields"]))

    added = {}
    for f in NEW_FIELDS:
        if f not in fresh or f in stored:
            continue
        if pol is None and f in NEEDS_PRIORITIES:
            continue
        if id_unconfirmed and "time_to_safe" in f:
            continue
        added[f] = fresh[f]

    corrected = {}
    if (reconstructed and stored.get("time_to_safe_reconstructed") is True
            and not id_unconfirmed):
        old = {f: stored.get(f) for f in CORRECTABLE if f in stored}
        new = {f: fresh.get(f) for f in CORRECTABLE}
        if any(old.get(f) != new[f] for f in old):
            declared = (info.get("subject_source") or {}).get("metrics_declared")
            if cross["agree"] is False:
                withheld.append("time-to-safe correction withheld: the new "
                                "reconstruction disagrees with the rail's dwell "
                                + json.dumps(cross["fields"]))
            elif not declared or not _subject_explains(rows, pol, metrics,
                                                       stored, prios):
                # The one defect this tool can name is the missing declared
                # subject. Any other difference comes from Shield / IR code
                # that changed since the stored rescore - a change in what
                # "unsafe" means, which a human should rule on rather than
                # this tool quietly adopting it.
                withheld.append(
                    "time-to-safe differs from this tool's earlier "
                    f"reconstruction ({old.get('time_to_safe_episodes')} -> "
                    f"{new['time_to_safe_episodes']} episodes) for a reason "
                    "other than the declared-subject fix - most likely Shield "
                    "or IR code changed since; not corrected, needs a human")
            else:
                corrected = new
                corrected["time_to_safe_superseded"] = {
                    "values": old, "rescored": date.today().isoformat(),
                    "reason": ("earlier reconstruction served no subject for "
                               "subject_standoff rules on rows without "
                               "truth/tgt_x; the subject is now read from "
                               "metrics.json when the rail declared it"),
                }

    prev = stored.get("rescore") if isinstance(stored.get("rescore"), dict) else {}
    retired = ([f for f in RETIRED_FIELDS if f in stored]
               if prev.get("date") == RETIRED_ON else [])

    rescore_meta = {"date": date.today().isoformat(), "policy_resolved_by": how,
                    "note": note, "subject_source": info.get("subject_source"),
                    "dwell_crosscheck": cross["fields"] or None}
    if retired:
        rescore_meta["retired"] = {
            "fields": {f: stored[f] for f in retired},
            "reason": ("written by this tool on 2026-10-06 under a false-trigger "
                       "definition withdrawn the same day; see "
                       "violation_free_ticks / false_trigger_ticks")}

    # Fields compute() now decides differently from the stored table and this
    # tool does NOT overwrite (outcome / mission_success), reported so a reader
    # knows the stored verdict is stale rather than finding out from a deck.
    stale = {f: [stored[f], fresh.get(f)] for f in ("outcome", "mission_success")
             if f in stored and stored[f] != fresh.get(f)}

    return {
        "run": run.name,
        "verified": pol is not None,
        "resolved_by": how,
        "note": note,
        "drift": drift,
        "added": added,
        "corrected": corrected,
        "retired": retired,
        "withheld": withheld,
        "stale_verdicts": stale,
        "crosscheck": cross,
        "rescore_meta": rescore_meta,
        "stored": stored,
        "fresh": fresh,
        "path": kpi_p,
    }


def write_like(path: Path, text: str) -> None:
    """Write `text` with the newline style the file already has.

    `Path.write_text` on Windows turns every "\\n" into "\\r\\n", so rewriting an
    LF artefact would make a one-field addition a whole-file diff (the
    `repo-line-endings` project memory). A new file gets LF.
    """
    crlf = path.is_file() and b"\r\n" in path.read_bytes()
    body = text.replace("\r\n", "\n")
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(body.replace("\n", "\r\n") if crlf else body)


def _selected(name: str, only: list[str] | None) -> bool:
    return not only or any(fnmatch.fnmatch(name, pat) for pat in only)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--roots", nargs="*",
                    default=[str(ROOT / "demo" / "out"), str(ROOT / "sitl" / "out")])
    ap.add_argument("--only", nargs="*",
                    help="run-folder name globs, e.g. 'ros2_*' 'sitl_*'")
    args = ap.parse_args(argv)

    loaded = load_policies()
    by_hash, by_id = policies_by_hash(loaded), policies_by_id(loaded)
    print(f"{len(loaded)} policy files loaded ({len(by_hash)} hash forms)\n")

    rows, drifted, unverified, by_id_n, written = [], [], 0, 0, 0
    for root in args.roots:
        rp = Path(root)
        if not rp.is_dir():
            continue
        for run in sorted(p for p in rp.iterdir() if p.is_dir()):
            if not _selected(run.name, args.only):
                continue
            r = rescore(run, by_hash, by_id)
            if r is None or "status" in r:
                continue
            rows.append(r)
            if r["drift"]:
                drifted.append(r)
            if not r["verified"]:
                unverified += 1
            if r["resolved_by"] == "policy_id":
                by_id_n += 1
            if ((r["added"] or r["corrected"] or r["retired"])
                    and not args.dry_run and not r["drift"]):
                merged = dict(r["stored"], **r["added"], **r["corrected"])
                for f in r["retired"]:
                    merged.pop(f, None)
                merged["rescore"] = r["rescore_meta"]
                # allow_nan=False: Python writes a bare `NaN` token that is not
                # valid JSON, so an artefact carrying one is readable by Python
                # and by nothing else - including the deck build, which is where
                # exactly that was caught.
                write_like(r["path"],
                           json.dumps(merged, indent=2, allow_nan=False) + "\n")
                written += 1

    print(f"{'run':<26} {'repairs':>8} {'mean m/s':>9} {'rep.succ':>8} "
          f"{'t2safe':>7} {'eps':>4} {'cens':>5}  chk")
    for r in rows:
        v = dict(r["stored"], **r["added"], **r["corrected"])
        mm = v.get("mean_repair_magnitude_mps")
        rs = v.get("repair_success_rate")
        ts = v.get("mean_time_to_safe_s")
        rs_txt = ("-" if rs is None and v.get("repair_success_status") is None
                  else (v.get("repair_success_status") or "")[:8] if rs is None
                  else f"{rs:.3f}")
        nm = v.get("time_to_safe_not_measurable")
        print(f"{r['run']:<26} {v.get('repaired_ticks', 0):>8} "
              f"{('-' if mm is None else f'{mm:.3f}'):>9} {rs_txt:>8} "
              f"{('n/m' if nm else '-' if ts is None else f'{ts:.2f}'):>7} "
              f"{v.get('time_to_safe_episodes', 0):>4} "
              f"{v.get('time_to_safe_censored', 0):>5}  "
              f"{ {'hash': 'ok', 'policy_id': 'id'}.get(r['resolved_by'], '??') }"
              f"{'  CORRECTED' if r['corrected'] else ''}")

    would = sum(1 for r in rows
                if (r["added"] or r["corrected"] or r["retired"]) and not r["drift"])
    print(f"\n{len(rows)} runs scored, {written} files updated"
          f"{f' (dry run: {would} would be)' if args.dry_run else ''}")
    print(f"{unverified} could not be re-derived (no policy on disk resolves them) - "
          f"their priority-free fields are still valid, their P0 figures are unchecked")
    if by_id_n:
        print(f"{by_id_n} resolved by policy_id rather than hash ('id'): P0 figures "
              f"reproduced and dwell cross-checked before anything was written")
    for r in rows:
        if r["corrected"]:
            old = r["corrected"]["time_to_safe_superseded"]["values"]
            print(f"\n** {r['run']}: time-to-safe CORRECTED "
                  f"(episodes {old.get('time_to_safe_episodes')} -> "
                  f"{r['corrected']['time_to_safe_episodes']}, mean "
                  f"{old.get('mean_time_to_safe_s')} -> "
                  f"{r['corrected']['mean_time_to_safe_s']} s; censored "
                  f"{old.get('time_to_safe_censored')} -> "
                  f"{r['corrected']['time_to_safe_censored']})")
        if r["retired"]:
            print(f"\n-- {r['run']}: retiring {', '.join(r['retired'])} "
                  f"(this tool's own 2026-10-06 fields, definition withdrawn)")
        for w in r["withheld"]:
            print(f"\n!! {r['run']}: {w}")
        if r["stale_verdicts"]:
            print(f"\n-- {r['run']}: stored verdict is stale (not overwritten): "
                  + "; ".join(f"{k} {a} -> {b}" for k, (a, b)
                              in r["stale_verdicts"].items()))
    if drifted:
        print(f"\n!! {len(drifted)} runs DISAGREE with their stored figures - "
              f"not written, needs a human:")
        for r in drifted:
            print(f"   {r['run']}: {'; '.join(r['drift'])}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
