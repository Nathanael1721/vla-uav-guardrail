"""The project's open-work list: tracker/data/todo.json.

    python tools/todo.py list                 # open items, overdue first
    python tools/todo.py list --all
    python tools/todo.py check                # validate (exit 1 on a problem)
    python tools/todo.py set VLA-02 --status doing
    python tools/todo.py close A-14 --commit 1a2b3c4
    python tools/todo.py close A-03 --changelog "### 2026-10-21 — ..."
    python tools/todo.py add --id C-18 --wp WP3 --owner agent --kind bug \\
        --en "..." --id-text "..." --zh "..."

The Dashboard's "Open to-do" panel and its VLA milestones read this file
(through tools/build_dashboard.py). Like the update log (tools/add_update.py)
every item carries its title in English, Indonesian and Traditional Chinese,
and `check` refuses the ways a to-do list quietly goes wrong:
  - an id missing, malformed or used twice;
  - a title missing in a language, still "TODO", or with a Simplified-only
    character in zh;
  - an owner that is not a role (user / agent / pi: the list names no
    colleague), a status, kind or WP outside the allowed values, a due date
    that is not YYYY-MM-DD;
  - a card id the tracker does not have;
  - an item marked done with nothing that closed it: a commit git knows, a
    CHANGELOG heading that exists, or (for the auto milestone) a run folder
    the VLA flew (classify_run: a manifest naming a real VLA and the
    server's answers in the run's records). "done" with no proof reads as
    progress that may never have happened;
  - a probe (a file + a string whose presence the Dashboard reports beside
    the item) that points outside the repository.

The auto milestone: an item with "auto": "first_vla_flight" shows as done on
the Dashboard as soon as a demo/out run is a VLA flight (the same classifier
as the Dashboard, classify_run): its manifest names a real VLA in
vla_model_hash (an OWL-ViT detector, the StubVLA or a hand-written controller
never count) AND its own records show the VLA SERVER answered and its steps
were flown (vla_evidence): enough answers and ticks, at least half of the
requests answered and of the loop ticks flown by the VLA, and the model the
server reported is the one the manifest configured (evidence_problem).
`todo.py sync` writes that into the file, with the run as its proof.

Nothing here reads or writes the tracker's board (tracker/data/state.json).
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TODO = ROOT / "tracker" / "data" / "todo.json"
sys.path.insert(0, str(Path(__file__).resolve().parent))

import add_update as AU                                     # noqa: E402

LANGS = ("en", "id", "zh")
OWNERS = ("user", "agent", "pi")
STATUSES = ("open", "doing", "blocked", "done")
KINDS = ("milestone", "gate", "kpi", "deliverable", "bug", "decision", "task")
AREAS = ("WP1", "WP2", "WP3", "WP4", "ARCH", "CROSS", "CLAIMS")
AUTOS = ("first_vla_flight",)
NEEDS = ("flight", "orin", "pi", "gpu", "push")
EFFORTS = ("S", "M", "L")
ID_RE = re.compile(r"^[A-Z]{1,5}-\d{2,3}$")
# add_update's set holds the common slips; these add frequent ones that a
# short title is likely to use (none is normal Traditional usage).
SIMPLIFIED = set(AU.SIMPLIFIED) | set("务单车场东压参确称稳类组细终给络统继续维编联节获补装视觉计订讨训记论访评语说请读调谈负费资转轮较输运远违连选递逻针钟门闭队阶际险难项须领频题风驱")


# ------------------------------------------------------------------ classify
VLA_ALLOW = (re.compile(r"(?i)openvla"), re.compile(r"(?i)aerial[-_ ]?vla"),
             re.compile(r"(?i)aero[-_]?vla"))


def classify_pilot(vla_model_hash) -> str:
    """vla_model_hash -> none | stub | detector | vla | unknown | missing.

    Only an id on the allow-list (OpenVLA, AerialVLA and its adapter) is a
    VLA. 65 of the 99 manifests in demo/out on 2026-10-21 record the OWL-ViT
    DETECTOR here (google/owlvit-base-patch32@...), 24 the StubVLA and 10
    none:hand-written-controller; none of them is a VLA flight, and a
    dashboard that counted "anything not none" would report 89."""
    if vla_model_hash is None or str(vla_model_hash).strip() == "":
        return "missing"
    s = str(vla_model_hash).strip()
    ident = s.split("@", 1)[0].strip()
    low = ident.lower()
    if s.startswith("none:") or low == "none":
        return "none"
    if low.startswith("guardrail.vla_stub") or "stubvla" in low:
        return "stub"
    if "owlvit" in low or "owl-vit" in low or "owlv2" in low:
        return "detector"
    if any(p.search(ident) for p in VLA_ALLOW):
        return "vla"
    return "unknown"


# ------------------------------------------------------------------ the server's evidence
# A configured VLA is not a flown one. The manifest's vla_model_hash is
# computed on the PILOT side, from the configured model id and the local
# weight files (guardrail/manifest.py model_hash / weights_digest): nothing
# in it says the server ever answered. A flight configured with AerialVLA
# whose protocol-v2 server never answered, or answered only parse_ok=false,
# holds zero or hovers on every tick (guardrail/vla_link.py StepTracker), and
# the first version of this file still counted it as a VLA flight and closed
# the first-VLA-flight milestone with it (review of 8035d0a, P2). So a run is
# a VLA flight only on the run's OWN records of what the server returned:
#   vla_steps.jsonl (guardrail/vla_link.py StepLog, schema vla_steps/1: one
#     line per request with the server's answer, its model identity and what
#     the tracker made of it), else
#   metrics.json "pilot" (or "pilot"."vla") in RemoteVLABackend.stats() shape
#     (decisions_accepted, ticks_by_author, identity).
VLA_STEPS_SCHEMA = "vla_steps/1"             # guardrail/vla_link.py VLA_STEPS_SCHEMA
# Judgements, not grant numbers: ten answers the model parsed and the tracker
# took (about 3.5 s of decisions at the paper's 0.35 s each on an RTX 4090),
# and one second of the 10 Hz loop flown toward their steps. Fewer is a
# connection that answered a few times, not a flight the VLA flew.
MIN_VLA_DECISIONS = 10
MIN_VLA_STEP_TICKS = 10
# SHARES, NOT ONLY COUNTS (review of d180e01, P3). The two floors above are
# absolute over the whole run: a server that answered ten times and then died
# left ten accepted decisions with one vla_step tick each and 1150
# link_failure rows (about 115 s of hover), and the run classified as the
# VLA's flight, became last_vla_flight and would have closed VLA-05. A VLA
# flight now also needs the VLA to have answered most of what it was asked,
# and to have authored most of the mission's loop:
#   decision share = decisions / requests, the requests being the decisions
#     plus the answers without a usable digest, the parse failures, the
#     server errors and the link failures. A request the TRACKER refused
#     (stale on arrival, during a rail hold) is the guardrail's doing, not a
#     failure of the VLA, and is left out of both sides.
#   tick share = loop ticks the VLA authored (VLA_AUTHORS: vla_step, and
#     hold, its finished step held; guardrail/vla_link.py AUTHORS) / the
#     mission's loop ticks, read in this order: the pilot's own
#     ticks_by_author (one per act() call, hover and none included),
#     metrics.json "ticks", kpi.json "ticks" (one per flight-log row,
#     guardrail/kpi.py), else the time vla_steps.jsonl's rows span at
#     LOOP_HZ (an under-count: ticks before the first request are not in it).
#     No mission length at all is "mission_ticks_unknown", never a pass.
# One half each is a judgement, not a grant number: pipelined decisions every
# ~0.35 s supersede each other, so a run the VLA flew has nearly every tick
# authored by it, and a run that is mostly hover or guardrail hold is not one
# the VLA flew, whatever it did in its first seconds.
MIN_DECISION_SHARE = 0.5
MIN_VLA_TICK_SHARE = 0.5
VLA_AUTHORS = ("vla_step", "hold")
LOOP_HZ = 10.0                              # the rails' loop (perception_node TICK = 0.1 s)
_HEX64 = re.compile(r"[0-9a-f]{64}")
# guardrail/manifest.py resolve_vla_hash (wave V1a): the pilot side's own
# verdict that the server ran other bytes than the ones it holds under that id,
# and the source that says its local digest equalled the server's report.
REPORTED_DIGEST_MISMATCH = "reported-digest-mismatch"
REPORTED_AGREES = "local; reported agrees"
_MANIFEST_DIGEST = re.compile(r"@sha256:([0-9a-f]{64})$")


def _count(x) -> int | None:
    return x if isinstance(x, int) and not isinstance(x, bool) and x >= 0 else None


def weights_digest_ok(d) -> bool:
    """64 lowercase hex digits (the protocol's weights_sha256), and not one
    digit repeated: tests/vla_fake_server.py reports "0" * 64, a stand-in
    that is not a model."""
    return isinstance(d, str) and bool(_HEX64.fullmatch(d)) and len(set(d)) > 1


def _num(x) -> float | None:
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) else None


def _evidence_from_steps(path: Path) -> dict:
    ev = {"source": "vla_steps.jsonl", "rows": 0, "bad_lines": 0, "scripted": 0, "decisions": 0,
          "vla_step_ticks": 0, "vla_ticks": 0, "no_digest": 0, "parse_fail": 0, "server_error": 0,
          "link_failure": 0, "refused": 0}
    digests, ids = set(), set()
    t_lo = t_hi = None
    try:
        fh = path.open(encoding="utf-8", errors="replace")
    except OSError:
        return dict(ev, unreadable=True)
    with fh:
        for line in fh:
            if not line.strip():
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                ev["bad_lines"] += 1
                continue
            if not isinstance(rec, dict) or rec.get("schema") != VLA_STEPS_SCHEMA:
                continue
            ev["rows"] += 1
            kind = rec.get("kind")
            if rec.get("source") == "scripted":
                # Decision.of(): built without the link (offline replay,
                # tests), no server ever answered it
                ev["scripted"] += 1
                continue
            # the time this request's row covers, for the mission length when
            # no other record gives it (MIN_VLA_TICK_SHARE above)
            out = rec.get("outcome") if isinstance(rec.get("outcome"), dict) else {}
            for t in (rec.get("t_capture"), rec.get("t_sent"), rec.get("t_received"),
                      out.get("t_start"), out.get("t_end")):
                t = _num(t)
                if t is not None:
                    t_lo = t if t_lo is None else min(t_lo, t)
                    t_hi = t if t_hi is None else max(t_hi, t)
            if kind in ("parse_fail", "server_error", "link_failure"):
                ev[kind] += 1
            if rec.get("accepted") is not True:
                ev["refused"] += rec.get("accepted") is False
                continue
            # an answer the model PARSED (a step or LAND) that the tracker took
            if kind not in ("step", "land") or rec.get("parse_ok") is not True:
                continue
            model = rec.get("model") if isinstance(rec.get("model"), dict) else {}
            if not weights_digest_ok(model.get("weights_sha256")):
                ev["no_digest"] += 1
                continue
            ev["decisions"] += 1
            digests.add(model["weights_sha256"])
            ids.add(str(model.get("id") or ""))
            ticks = out.get("ticks") if isinstance(out.get("ticks"), dict) else {}
            ev["vla_step_ticks"] += _count(ticks.get("vla_step")) or 0
            ev["vla_ticks"] += sum(_count(ticks.get(a)) or 0 for a in VLA_AUTHORS)
    ev["digests"] = sorted(digests)
    ev["model_ids"] = sorted(ids)
    ev["span_s"] = round(t_hi - t_lo, 3) if t_lo is not None and t_hi > t_lo else None
    return ev


def _pilot_stats(metrics) -> tuple[str | None, dict | None]:
    """("pilot/vla" or "pilot", the RemoteVLABackend.stats() block) of a
    metrics.json, or (None, None)."""
    pilot = metrics.get("pilot") if isinstance(metrics, dict) else None
    if isinstance(pilot, dict):
        for key, st in (("pilot/vla", pilot.get("vla")), ("pilot", pilot)):
            if isinstance(st, dict) and "decisions_accepted" in st:
                return key, st
    return None, None


def _evidence_from_metrics(metrics: dict) -> dict | None:
    key, st = _pilot_stats(metrics)
    if st is None:
        return None
    ident = st.get("identity") if isinstance(st.get("identity"), dict) else {}
    digest = ident.get("weights_sha256")
    by = st.get("ticks_by_author") if isinstance(st.get("ticks_by_author"), dict) else {}
    n = _count(st.get("decisions_accepted")) or 0
    ok = weights_digest_ok(digest)
    return {"source": f"metrics.json#/{key}", "decisions": n if ok else 0,
            "no_digest": 0 if ok else n, "vla_step_ticks": _count(by.get("vla_step")) or 0,
            "vla_ticks": sum(_count(by.get(a)) or 0 for a in VLA_AUTHORS),
            "digests": [digest] if ok else [], "model_ids": [str(ident.get("id") or "")] if ok else [],
            "parse_fail": _count(st.get("parse_fail")) or 0,
            "server_error": _count(st.get("server_error")) or 0,
            "link_failure": _count(st.get("link_failure")) or 0}


def _read_json(p: Path):
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _shares(ev: dict, metrics, kpi) -> None:
    """The decision share and the tick share (MIN_DECISION_SHARE above), with
    the mission length's source, into `ev`."""
    req = sum(ev.get(k) or 0 for k in ("decisions", "no_digest", "parse_fail", "server_error", "link_failure"))
    ev["requests"] = req
    ev["decision_share"] = round(ev["decisions"] / req, 4) if req else None
    n = src = None
    key, st = _pilot_stats(metrics)
    by = st.get("ticks_by_author") if st is not None and isinstance(st.get("ticks_by_author"), dict) else {}
    tot = sum(_count(v) or 0 for v in by.values())
    if tot > 0:
        n, src = tot, f"metrics.json#/{key}/ticks_by_author"
    for name, doc in (("metrics.json#/ticks", metrics), ("kpi.json#/ticks", kpi)):
        if n is None and isinstance(doc, dict) and (_count(doc.get("ticks")) or 0) > 0:
            n, src = doc["ticks"], name
    if n is None and ev.get("span_s"):
        n, src = max(1, round(ev["span_s"] * LOOP_HZ)), f"vla_steps.jsonl time span x {LOOP_HZ:g} Hz"
    ev["mission_ticks"], ev["mission_ticks_source"] = n, src
    ev["vla_tick_share"] = round(min(1.0, (ev.get("vla_ticks") or 0) / n), 4) if n else None


def vla_evidence(run: Path, metrics: dict | None = None, kpi: dict | None = None) -> dict:
    """What the VLA SERVER returned during this run, from the run's own
    records (vla_steps.jsonl first, else metrics.json's pilot block; the
    block above). `decisions` counts answers the model parsed (a step or
    LAND) that the tracker accepted and that carry a real weights digest;
    `weights_sha256` / `model_id` are set only when exactly one was seen;
    `decision_share` / `vla_tick_share` are the shares MIN_DECISION_SHARE /
    MIN_VLA_TICK_SHARE judge (None when there is nothing to divide by);
    `pilot_agreed_digest` is the server digest the pilot side recorded its
    own local digest agreeing with (metrics.json pilot record), if any.
    Never raises; a run with neither record has source None."""
    ev = None
    try:
        run = Path(run)
        if metrics is None:
            metrics = _read_json(run / "metrics.json")
        if kpi is None:
            kpi = _read_json(run / "kpi.json")
        metrics = metrics if isinstance(metrics, dict) else None
        kpi = kpi if isinstance(kpi, dict) else None
        p = run / "vla_steps.jsonl"
        if p.is_file():
            ev = _evidence_from_steps(p)
        elif metrics is not None:
            ev = _evidence_from_metrics(metrics)
        if ev is not None:
            _shares(ev, metrics, kpi)
            pilot = (metrics or {}).get("pilot")
            if (isinstance(pilot, dict) and pilot.get("hash_source") == REPORTED_AGREES
                    and weights_digest_ok(pilot.get("reported_digest"))):
                ev["pilot_agreed_digest"] = pilot["reported_digest"]
    except Exception:                                        # noqa: BLE001 - evidence never breaks a build
        ev = None
    ev = ev or {"source": None, "decisions": 0, "vla_step_ticks": 0, "digests": [], "model_ids": []}
    ev["weights_sha256"] = ev["digests"][0] if len(ev.get("digests") or []) == 1 else None
    ev["model_id"] = ev["model_ids"][0] if len(ev.get("model_ids") or []) == 1 else None
    return ev


def vla_family(model_id) -> str | None:
    """"aerialvla" (the AerialVLA adapter, on whatever base), "openvla" (the
    OpenVLA base alone) or None. AerialVLA is checked first: its ids name
    the base as well ("aerialvla:openvla-7b+aero_vla")."""
    s = str(model_id or "").split("@", 1)[0]
    if VLA_ALLOW[1].search(s) or VLA_ALLOW[2].search(s):
        return "aerialvla"
    return "openvla" if VLA_ALLOW[0].search(s) else None


def server_model_problem(vla_model_hash, ev: dict) -> str | None:
    """Why the model the server reported is not the one the manifest
    configured, or None (review of d180e01, P7: classify_run took any
    allow-listed model the server reported, so a run configured as AerialVLA
    and served by the bare OpenVLA base, or by a run1-3 fine-tune - other
    AerialVLA weights - counted as that VLA flight). Both the manifest's
    digest and the server's weights_sha256 are guardrail/manifest.py
    weights_digest over the base shards + adapter (demo/vla_server_v2.py), so
    the same weights give the same 64 hex, except for an id that resolves a
    HuggingFace snapshot, whose local hash mixes the revision in: that run
    passes on the pilot side's own record that its local digest equalled the
    server's report (resolve_vla_hash "local; reported agrees")."""
    h = str(vla_model_hash or "").strip()
    if REPORTED_DIGEST_MISMATCH in h:
        return "server_model_differs:the pilot side recorded a reported-digest mismatch"
    want, got = vla_family(h), vla_family(ev.get("model_id"))
    if want != got:
        return f"server_model_differs:configured {want}, server {got}"
    m = _MANIFEST_DIGEST.search(h)
    served = ev.get("weights_sha256")
    if m and served and m.group(1) != served and ev.get("pilot_agreed_digest") != served:
        return f"server_model_differs:weights {served[:12]} not the configured {m.group(1)[:12]}"
    return None


def evidence_problem(ev: dict, vla_model_hash=None) -> str | None:
    """Why this evidence does not show a VLA flying, as a reason code (with
    a count, a share or a detail after ':' where one helps), or None. With
    `vla_model_hash`, the server's model must also be the configured one
    (server_model_problem)."""
    if not ev or ev.get("source") is None:
        return "no_records"
    n = ev.get("decisions") or 0
    if n == 0:
        return "no_weights_digest" if ev.get("no_digest") else "no_accepted_decisions"
    if len(ev.get("digests") or []) > 1:
        return f"several_weights_digests:{len(ev['digests'])}"
    if len(ev.get("model_ids") or []) != 1 or classify_pilot(ev["model_ids"][0]) != "vla":
        return "model_not_a_vla:" + ",".join(m.split("@", 1)[0][:40] for m in ev.get("model_ids") or [])
    if vla_model_hash is not None:
        why = server_model_problem(vla_model_hash, ev)
        if why is not None:
            return why
    if n < MIN_VLA_DECISIONS:
        return f"too_few_decisions:{n}"
    share = ev.get("decision_share")
    if share is None or share < MIN_DECISION_SHARE:
        return f"decision_share_low:{n}/{ev.get('requests')}"
    if (ev.get("vla_step_ticks") or 0) < MIN_VLA_STEP_TICKS:
        return f"too_few_vla_step_ticks:{ev.get('vla_step_ticks') or 0}"
    if ev.get("mission_ticks") is None:
        return "mission_ticks_unknown"
    if (ev.get("vla_tick_share") or 0.0) < MIN_VLA_TICK_SHARE:
        return f"vla_tick_share_low:{ev.get('vla_ticks') or 0}/{ev['mission_ticks']}"
    return None


def classify_run(vla_model_hash, evidence: dict | None) -> tuple[str, str | None]:
    """(pilot class, why a configured VLA is unverified). "vla" needs both
    the rail's claim (its manifest names a real VLA: classify_pilot) and the
    server's evidence (evidence_problem is None): records without the claim
    may be a VLA run in the shadow of another controller, a claim without
    records is a VLA nobody heard from, and a server that served another
    model than the configured one did not fly the configured VLA. A
    configured VLA without the evidence is "vla_unverified"; every other
    class is classify_pilot's."""
    configured = classify_pilot(vla_model_hash)
    if configured != "vla":
        return configured, None
    why = evidence_problem(evidence or {}, vla_model_hash)
    return ("vla", None) if why is None else ("vla_unverified", why)


def vla_runs(demo_out: Path) -> list[tuple[str, str]]:
    """(tag, vla_model_hash) of every demo/out/<tag> run that is a VLA
    flight (classify_run: the manifest names a real VLA and the run's own
    records carry the server's evidence), oldest first by manifest time.
    Folders starting with '_' are set aside and skipped."""
    out = []
    if not demo_out.is_dir():
        return out
    for m in demo_out.glob("*/manifest.json"):
        if m.parent.name.startswith("_"):
            continue
        try:
            h = json.loads(m.read_text(encoding="utf-8")).get("vla_model_hash")
        except (OSError, ValueError, AttributeError):
            continue
        if classify_pilot(h) == "vla" and classify_run(h, vla_evidence(m.parent))[0] == "vla":
            out.append((m.stat().st_mtime, m.parent.name, str(h)))
    return [(t, h) for _, t, h in sorted(out)]


def effective(item: dict, vla: list[tuple[str, str]]) -> tuple[str, dict | None]:
    """(status as the Dashboard shows it, the auto evidence or None)."""
    if item.get("auto") == "first_vla_flight" and vla:
        return "done", {"run": vla[0][0], "vla_model_hash": vla[0][1]}
    return item.get("status", "open"), None


# ------------------------------------------------------------------ check
def card_ids(root: Path) -> set[str] | None:
    files = sorted((root / "tracker" / "data" / "cards").glob("batch_*.json"))
    if not files:
        return None
    ids = set()
    for f in files:
        try:
            ids |= {c["id"] for c in json.loads(f.read_text(encoding="utf-8"))}
        except (OSError, ValueError, KeyError, TypeError):
            pass
    return ids


def commit_exists(root: Path, sha: str) -> bool:
    if not re.fullmatch(r"[0-9a-f]{7,40}", sha or ""):
        return False
    try:
        r = subprocess.run(["git", "cat-file", "-e", f"{sha}^{{commit}}"], cwd=root,
                           capture_output=True, timeout=20)
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def changelog_headings(root: Path) -> set[str]:
    try:
        return {s.heading for s in AU.parse_changelog((root / "CHANGELOG.md").read_text(encoding="utf-8"))}
    except OSError:
        return set()


def _safe_rel(p) -> bool:
    if not isinstance(p, str) or not p or p.startswith(("/", "\\")) or re.match(r"^[A-Za-z]:", p):
        return False
    return ".." not in Path(p).parts


def check(doc: dict, root: Path = ROOT, demo_out: Path | None = None,
          cards: set[str] | None = None, git=commit_exists) -> list[str]:
    errs: list[str] = []
    items = doc.get("items") if isinstance(doc, dict) else None
    if not isinstance(doc, dict) or doc.get("schema") != 1 or not isinstance(items, list):
        return ['todo.json must be {"schema": 1, "items": [...]}']
    cards = card_ids(root) if cards is None else cards
    heads = None
    vla = vla_runs(demo_out if demo_out is not None else root / "demo" / "out")
    seen = set()
    for n, it in enumerate(items):
        tag = f"#{n + 1} {it.get('id', '?') if isinstance(it, dict) else '?'}"
        if not isinstance(it, dict):
            errs.append(f"{tag}: not an object")
            continue
        iid = it.get("id")
        if not isinstance(iid, str) or not ID_RE.match(iid):
            errs.append(f"{tag}: id must look like VLA-01 or A-12")
        elif iid in seen:
            errs.append(f"{tag}: duplicate id")
        seen.add(iid)
        tri = it.get("i18n") or {}
        for lang in LANGS:
            title = ((tri.get(lang) or {}).get("title") or "").strip()
            if not title:
                errs.append(f"{tag}: {lang} title missing")
            elif "TODO" in title:
                errs.append(f"{tag}: {lang} title still says TODO")
        zh = " ".join(str(v) for v in (tri.get("zh") or {}).values())
        bad = sorted({ch for ch in zh if ch in SIMPLIFIED})
        if bad:
            errs.append(f"{tag}: zh contains Simplified characters: {''.join(bad)}")
        if it.get("owner") not in OWNERS:
            errs.append(f"{tag}: owner must be a role, one of {OWNERS}")
        if it.get("status") not in STATUSES:
            errs.append(f"{tag}: status must be one of {STATUSES}")
        if it.get("kind") not in KINDS:
            errs.append(f"{tag}: kind must be one of {KINDS}")
        if it.get("wp") not in AREAS:
            errs.append(f"{tag}: wp must be one of {AREAS}")
        due = it.get("due")
        if due is not None:
            try:
                date.fromisoformat(due)
            except (TypeError, ValueError):
                errs.append(f"{tag}: due must be YYYY-MM-DD or null")
        for k, allowed in (("needs", NEEDS),):
            for v in it.get(k) or []:
                if v not in allowed:
                    errs.append(f"{tag}: {k} value {v!r} not in {allowed}")
        if it.get("effort") not in (None,) + EFFORTS:
            errs.append(f"{tag}: effort must be one of {EFFORTS} or null")
        if it.get("auto") not in (None,) + AUTOS:
            errs.append(f"{tag}: auto must be one of {AUTOS} or null")
        if not isinstance(it.get("cards", []), list):
            errs.append(f"{tag}: cards must be a list")
        elif cards is not None:
            for cid in it.get("cards", []):
                if cid not in cards:
                    errs.append(f"{tag}: unknown card {cid}")
        probe = it.get("probe")
        if probe is not None:
            if not isinstance(probe, dict) or not _safe_rel(probe.get("file")) \
                    or not isinstance(probe.get("contains", ""), str):
                errs.append(f"{tag}: probe must be {{file: <repo-relative path>, contains: <text>}}")
        if it.get("status") == "done":
            cb = it.get("closed_by") or {}
            ok = False
            if cb.get("commit"):
                ok = git(root, cb["commit"])
                if not ok:
                    errs.append(f"{tag}: closed_by commit {cb['commit']} is not a commit git knows")
            elif cb.get("changelog"):
                heads = changelog_headings(root) if heads is None else heads
                ok = cb["changelog"] in heads
                if not ok:
                    errs.append(f"{tag}: closed_by changelog heading not found: {cb['changelog'][:60]}")
            elif cb.get("run"):
                ok = it.get("auto") == "first_vla_flight" and cb["run"] in {t for t, _ in vla}
                if not ok:
                    errs.append(f"{tag}: closed_by run {cb['run']} is not a demo/out run the VLA flew "
                                f"(a manifest naming a real VLA and records of the server's answers)")
            else:
                errs.append(f"{tag}: done, but closed_by names no commit, CHANGELOG heading or run")
    return errs


# ------------------------------------------------------------------ io
def load(path: Path = TODO) -> dict:
    if not path.is_file():
        return {"schema": 1, "items": []}
    return json.loads(path.read_text(encoding="utf-8"))


def find(doc: dict, iid: str) -> dict:
    for it in doc.get("items", []):
        if it.get("id") == iid:
            return it
    raise SystemExit(f"no item {iid}")


def _save_checked(doc: dict, path: Path, root: Path = ROOT) -> int:
    errs = check(doc, root)
    if errs:
        for e in errs:
            print("ERROR", e)
        print("not written")
        return 1
    AU.write_atomic(path, doc)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Maintain the project's to-do list.")
    ap.add_argument("--file", default=str(TODO))
    sp = ap.add_subparsers(dest="cmd", required=True)
    ls = sp.add_parser("list")
    ls.add_argument("--all", action="store_true")
    ls.add_argument("--lang", choices=LANGS, default="en")
    sp.add_parser("check")
    sp.add_parser("sync", help="mark the auto milestone done when a run names a real VLA")
    st = sp.add_parser("set")
    st.add_argument("id")
    st.add_argument("--status", choices=STATUSES)
    st.add_argument("--owner", choices=OWNERS)
    st.add_argument("--due")
    st.add_argument("--branch")
    cl = sp.add_parser("close")
    cl.add_argument("id")
    g = cl.add_mutually_exclusive_group(required=True)
    g.add_argument("--commit")
    g.add_argument("--changelog")
    ad = sp.add_parser("add")
    ad.add_argument("--id", required=True)
    ad.add_argument("--wp", required=True, choices=AREAS)
    ad.add_argument("--owner", required=True, choices=OWNERS)
    ad.add_argument("--kind", required=True, choices=KINDS)
    ad.add_argument("--en", required=True)
    ad.add_argument("--id-text", required=True, dest="id_text")
    ad.add_argument("--zh", required=True)
    ad.add_argument("--cards", nargs="*", default=[])
    ad.add_argument("--due")
    a = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    path = Path(a.file)
    doc = load(path)
    if a.cmd == "check":
        errs = check(doc)
        for e in errs:
            print("ERROR", e)
        print(f"{len(doc.get('items', []))} items, {len(errs)} problem(s)")
        return 1 if errs else 0
    if a.cmd == "list":
        vla = vla_runs(ROOT / "demo" / "out")
        today = date.today().isoformat()
        rows = []
        for it in doc.get("items", []):
            st, ev = effective(it, vla)
            if st == "done" and not a.all:
                continue
            late = bool(it.get("due") and it["due"] < today and st != "done")
            rows.append((not late, it.get("due") or "9999", it["id"], st, it, ev, late))
        for _, _, iid, st, it, ev, late in sorted(rows):
            t = ((it.get("i18n") or {}).get(a.lang) or {}).get("title", "")
            extra = f"  [auto: {ev['run']}]" if ev else ""
            print(f"{iid:<8} {st:<8} {it.get('owner', ''):<6} {it.get('wp', ''):<6} "
                  f"{(it.get('due') or ''):<10}{' LATE' if late else '     '} {t}{extra}")
        return 0
    if a.cmd == "sync":
        vla = vla_runs(ROOT / "demo" / "out")
        changed = 0
        for it in doc.get("items", []):
            st, ev = effective(it, vla)
            if ev and it.get("status") != "done":
                it["status"], it["closed_by"] = "done", {"run": ev["run"]}
                changed += 1
        if not changed:
            print("nothing to sync")
            return 0
        return _save_checked(doc, path)
    if a.cmd == "set":
        it = find(doc, a.id)
        for k in ("status", "owner", "due", "branch"):
            v = getattr(a, k)
            if v is not None:
                it[k] = v
        return _save_checked(doc, path)
    if a.cmd == "close":
        it = find(doc, a.id)
        it["status"] = "done"
        it["closed_by"] = {"commit": a.commit} if a.commit else {"changelog": a.changelog}
        return _save_checked(doc, path)
    if a.cmd == "add":
        if any(x.get("id") == a.id for x in doc.get("items", [])):
            raise SystemExit(f"{a.id} exists")
        doc.setdefault("items", []).append({
            "id": a.id, "kind": a.kind, "wp": a.wp, "owner": a.owner, "status": "open",
            "cards": a.cards, "due": a.due, "branch": None, "closed_by": None,
            "milestone": None, "auto": None, "probe": None, "needs": [], "effort": None,
            "i18n": {"en": {"title": a.en}, "id": {"title": a.id_text}, "zh": {"title": a.zh}}})
        return _save_checked(doc, path)
    return 2


if __name__ == "__main__":
    sys.exit(main())
