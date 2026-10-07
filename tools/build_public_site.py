#!/usr/bin/env python3
"""Build the public progress site, docs/progress/, from curated sources.

The project keeps two websites:

* tracker/ - the local grant tracker: complete, internal, never published.
* docs/progress/ - this site: what is built, what is shown, how it works,
  what changed and what comes next, in English and Traditional Chinese, for
  readers outside the lab. docs/ is the GitHub Pages source, so the pages are
  plain static HTML with no build step on Pages, and they also open from
  file:// and from `python -m http.server -d docs`.

Where the text comes from:

* docs/progress/_src/site.json - curated public wording (en + zh), each claim
  with the files its numbers come from. This is the file to edit.
* tracker/data/content_overview.json and content_flow.json - explanation and
  diagram text, copied through an explicit ALLOW_LIST of fields only. The
  tracker data is local (gitignored), so every build also writes the copied
  fields to docs/progress/_src/tracker_extract.json, and a clone without the
  tracker data builds from that snapshot.
* CHANGELOG.md - the timeline: every published change entry must anchor to a
  CHANGELOG section, and its numbers must appear in that section, outside its
  Retracted / Found / Known limitations / Not measured subsections.
* docs/img, docs/video - evidence. An image must be tracked by git (or it would
  404 on Pages), or sit in the site's own docs/progress/assets/img/. A video is
  linked only when it is tracked and small (MAX_VIDEO_BYTES); otherwise the
  page shows a placeholder naming the file and its size.

What the build refuses (PolicyViolation), so that a silent leak cannot happen:

* an allow-list entry that names a disallowed field (verdict, needs_pi,
  evidence, notes, retracted, decisions, ...), and any disallowed key anywhere
  in the extract, the curated content or the page model;
* text that restates a withdrawn claim, carries internal-record markers
  (audit verdicts, tracker card ids, meeting records, waiver talk), an
  e-mail address, bare "AirSim", or a person's name: commit authors, the
  remote owner, a local deny list, in Latin or Chinese characters (only the
  PI, as the grant lists him, is named);
* Simplified-only characters in the Traditional Chinese text;
* an image that is not in site.json "reviewed_images", or whose bytes changed
  since its digest was recorded there: the text scan cannot see inside an
  image, so a person looks at each one and records that they did.

The same text scan runs again on every rendered page, so text that reaches the
HTML by any route is caught.

What the build refuses as inconsistent (BuildError):

* a number in curated text that does not appear in any source it cites, or a
  number that differs between the English and the Chinese text (curated text
  and the tracker extract alike). A JSON source is cited by the path of the
  value (docs/data/x.json#a.b.c), and the number must match that value;
* a quantity written as a word in curated text ("three runs"): it would
  escape both checks, so it is written as digits;
* a change entry whose CHANGELOG section does not exist;
* a code path, or the script of a Reproduce command, that does not exist;
* an image that is missing or not tracked; a video whose recorded size does
  not match the file;
* no git: the author-name and committed-image checks need it (--no-git
  builds without them, for a local look only).

Usage:
    python tools/build_public_site.py            # build docs/progress/
    python tools/build_public_site.py --check    # exit 1 if the pages are stale,
                                                 # a CHANGELOG section the site
                                                 # watches changed since the last
                                                 # build, or the CHANGELOG is
                                                 # newer than as_of
    python tools/build_public_site.py --out DIR  # build somewhere else
    python tools/build_public_site.py --no-git   # local look without git

Standard library only, Python 3.10+.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import re
import subprocess
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "docs" / "progress"
SRC_DIR = OUT_DIR / "_src"
CONTENT_FILE = SRC_DIR / "site.json"
SNAPSHOT_FILE = SRC_DIR / "tracker_extract.json"
MANIFEST_FILE = SRC_DIR / "build_manifest.json"
TRACKER_DIR = ROOT / "tracker" / "data"
CHANGELOG = ROOT / "CHANGELOG.md"
DENY_NAMES_FILE = TRACKER_DIR / "_public_deny_names.txt"   # optional, local only
SITE_ASSETS = "docs/progress/assets/"    # images committed with the pages themselves

LANGS = ("en", "zh")
HTML_LANG = {"en": "en", "zh": "zh-Hant-TW"}
LANG_LABEL = {"en": "EN", "zh": "繁中"}
PAGES = ("index", "flow", "wp1", "wp2", "wp3", "wp4", "gallery", "changes",
         "roadmap", "glossary")
WP_KEYS = ("WP1", "WP2", "WP3", "WP4")
MAX_VIDEO_BYTES = 10_000_000
GENERATED = ("<!-- Generated by tools/build_public_site.py from "
             "docs/progress/_src/site.json. Do not edit by hand. -->")


class PolicyViolation(Exception):
    """Content that must not reach the public site."""


class BuildError(Exception):
    """A source is missing, or two sources disagree."""


# --------------------------------------------------------------- the policy
# Keys that never appear in anything the site is built from or renders. The
# first six are the ones the content policy names; the rest are the tracker's
# own names for the same kind of internal record.
DISALLOWED_KEYS = frozenset({
    "verdict", "needs_pi", "evidence", "notes", "retracted", "decisions",
    "verdicts", "status", "status_now", "pi_decisions", "pi_questions",
    "retractions", "honesty", "gap", "gaps", "not_proves", "built",
    "blocker", "blocker_detail", "audit", "cards", "proof", "contract_facts",
    "our_status", "we_have", "grant_asks", "changed_since_audit",
})

# The only tracker fields that may be copied. name: (file, top-level key,
# id field, plain fields copied as they are, i18n text fields). A glossary
# entry's i18n value is the text itself, so its text fields are None.
ALLOW_LIST = {
    "project": ("content_overview.json", "project", None, (),
                ("one_liner", "problem", "solution")),
    "roles": ("content_overview.json", "mental_model", "role_key", (),
              ("analogy", "in_project")),
    "wps": ("content_overview.json", "wps", "key",
            ("grant_page", "grant_quote"), ("name", "plain")),
    "architecture": ("content_flow.json", "grant_architecture", "key", (),
                     ("label", "short")),
    "pipeline": ("content_flow.json", "demo_pipeline", "key", (),
                 ("label", "short")),
    "shield_steps": ("content_flow.json", "shield_steps", "key", (),
                     ("label",)),
    "hud_callouts": ("content_flow.json", "hud_callouts", "key", ("x", "y"),
                     ("label", "detail")),
    "statuses": ("content_flow.json", "statuses", "key", ("color",),
                 ("label", "meaning")),
    "glossary": ("content_flow.json", "glossary", "term", (), None),
}

# Text that never reaches the page. (id, pattern, why)
FORBIDDEN_TEXT = (
    ("withdrawn", r"(?i)canonical[\s_-]*hil", "the desktop rail is the grant's dev topology"),
    ("withdrawn", r"(?i)\bhard\s+KPI", "P0 escape is one of five acceptance KPIs"),
    ("withdrawn", r"(?i)\b41\s+shielded", "the 41 flights are not a KPI figure"),
    ("withdrawn", r"3\.76\s*[-–]\s*5\.22", "detector rate recomputed as 2.77-4.31 Hz"),
    ("withdrawn", r"\b1\s+of\s+34\b", "recomputed as 0 of 34"),
    ("withdrawn", r"11713\s*/\s*11713", "the Shield's re-check, not the repair-success KPI"),
    ("withdrawn", r"(?i)\btarget\W{0,4}1\.0\b", "the fail-safe target is >= 99 %"),
    ("withdrawn", r"(?i)\bfour\s+(fully[- ]run\s+)?rails\b", "not four rails run end to end"),
    ("withdrawn", r"(?i)stretch\s+goal", "hil and flight are first-class in the grant"),
    ("withdrawn", r"(?i)structured\s+mission", "the Prefix Compiler emits a CSP"),
    ("withdrawn", r"(?i)byte-for-byte", "replay is not byte-for-byte"),
    ("contract-status", r"(?i)contract(ual)?\s+KPI|KPI-grade|合約\s*KPI", "no KPI status claims"),
    ("internal", r"(?i)(grant|contract)\s+audit|\baudit\s*:|AUDIT-KONTRAK", "audit verdicts are internal"),
    ("internal", r"\b(DRIFT|PARTIAL|MISSING)\b", "audit verdict words"),
    ("internal", r"(?i)needs[_ ]pi|PI\s+decision|\bPQ\d\b", "PI decisions are internal"),
    ("internal", r"\b(WP\d|ARCH|CROSS|C)-\d{2}\b", "tracker card ids are internal"),
    ("internal", r"(?i)waiver|豁免", "waiver discussion is internal"),
    ("internal", r"(?i)retract|withdrawn|inflated|撤回", "retraction lists are internal"),
    # A meeting as an event or a record; "meeting the 5 ms budget" is fine.
    ("internal", r"(?i)\b(at|in|from|during|after|before|since)\s+(the|our|a|this|that|last|next)\s+"
                 r"meetings?\b|\bmeetings?\s+(notes?|records?|minutes|packs?|of\s+\d)|"
                 r"\b(PI|ITRI|lab|group|progress|weekly|monthly|kick-?off|review)\s+meetings?\b|會議",
     "meeting records are internal"),
    ("internal", r"(?i)WORKLOG|GAMBARAN|CORRECTIONS?-20|PI-DECISIONS|state\.json|meeting notes|"
                 r"share pack|tracker/data", "internal documents"),
    ("internal", r"(?i)known\s+limitations?|known\s+issues?|found,\s+not\s+fixed|\bblocker|shortcoming",
     "the site reports progress; open work appears as Next on the roadmap"),
    ("contact", r"[\w.+-]+@[\w-]+\.[\w.-]+", "no e-mail addresses"),
    # The city flights run on Project AirSim. The July OpenVLA run used classic
    # AirSim, which may be named as such.
    ("wording", r"(?<!Project )(?<!classic )(?<!Classic )(?<![A-Za-z])AirSim(?![A-Za-z])",
     "say Project AirSim, or classic AirSim for the July run; bare 'AirSim' is ambiguous"),
)

# Curated text writes quantities as digits, so that each one is checked
# against its source and between the two languages ("three runs" would pass
# both checks unseen). "one" is left alone: it is mostly not a count.
NUMBER_WORDS = {
    "en": re.compile(r"(?i)\b(two|three|four|five|six|seven|eight|nine|ten|eleven|twelve|"
                     r"twenty|thirty|forty|fifty|hundred|dozen)\b"),
    "zh": re.compile(r"(?<!第)[兩二三四五六七八九十]+\s*(?:次|條|個|份|組|輛|名|項|種|架|天|秒|步|輪|"
                     r"台|臺|段|類|位|筆|張|頁|層|款|分鐘|小時)"),
}

# Characters that exist only in Simplified Chinese (same list as
# tools/add_update.py). Not exhaustive; it catches the common slips.
SIMPLIFIED = set("这们个为发说时实现过来对没还会进动问题关样经点种从开间无飞机区则检测图数据视频录试验证"
                 "错误变显应该报设计节约线条结构认识让处当务网页优软长头标级档维护层传输载达态况适轨规")

# The PI as the grant lists him; his name may appear, no other person's.
PI_NAME_TOKENS = {"kuan", "ting", "lai", "kuan-ting"}


# ------------------------------------------------------------- small helpers
def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha12(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()[:12]


def image_digest(data: bytes) -> str:
    return "sha256:" + hashlib.sha256(data).hexdigest()[:16]


def esc(text) -> str:
    return html.escape(str(text), quote=True)


def inline(text: str) -> str:
    """Escape, then allow `code` and **bold** - the only markup in site.json."""
    out = esc(text)
    out = re.sub(r"`([^`]+)`", r"<code>\1</code>", out)
    out = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", out)
    return out


def walk_strings(obj, path="$"):
    """Yield (path, string) for every string value and every dict key."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield f"{path}.{k}", str(k)
            yield from walk_strings(v, f"{path}.{k}")
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from walk_strings(v, f"{path}[{i}]")
    elif isinstance(obj, str):
        yield path, obj


# ------------------------------------------------------------ policy checks
def check_allow_list(allow: dict) -> None:
    """Refuse an allow-list that would copy a disallowed field."""
    for name, spec in allow.items():
        _src, key, id_field, plain, text = spec
        fields = [key, id_field, *plain, *(text or ())]
        bad = sorted({f for f in fields if f and f in DISALLOWED_KEYS})
        if bad:
            raise PolicyViolation(
                f"allow-list entry {name!r} names disallowed field(s) {bad}")


def assert_no_disallowed_keys(obj, where: str) -> None:
    """Refuse any disallowed key at any depth."""
    def walk(o, path):
        if isinstance(o, dict):
            for k, v in o.items():
                if k in DISALLOWED_KEYS:
                    raise PolicyViolation(
                        f"{where}: disallowed field {k!r} at {path}.{k}")
                walk(v, f"{path}.{k}")
        elif isinstance(o, list):
            for i, v in enumerate(o):
                walk(v, f"{path}[{i}]")
    walk(obj, "$")


def git(*args: str) -> str | None:
    try:
        r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True,
                           text=True, encoding="utf-8", timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout if r.returncode == 0 else None


_LATIN_TOKEN = re.compile(r"[A-Za-z][A-Za-z'-]*[A-Za-z]|[A-Za-z]")
_CJK_RUN = re.compile(r"[㐀-䶿一-鿿豈-﫿]{2,}")


def name_parts(name: str) -> set[str]:
    """One person's name -> the lower-case forms the site must not contain:
    the whole name (matched with any whitespace between its words), each
    Latin word of three or more letters, and each run of two or more CJK
    characters ('王小明', 'Ian Lin' -> {'ian lin', 'ian', 'lin'}). The PI's
    name gives nothing."""
    name = " ".join(name.split())
    latin = [t.lower() for t in _LATIN_TOKEN.findall(name)]
    cjk = _CJK_RUN.findall(name)
    if not cjk and latin and all(t in PI_NAME_TOKENS for t in latin):
        return set()
    out = {t for t in latin if len(t) >= 3 and t not in PI_NAME_TOKENS}
    out.update(cjk)
    if len(name) >= 2 and (len(latin) + len(cjk) > 1 or cjk):
        out.add(name.lower())
    return out


def person_names(extra_file: Path | None = DENY_NAMES_FILE, *,
                 require_git: bool = True) -> set[str]:
    """Names of people other than the PI: commit authors, the owner in the
    remote URLs, and an optional local deny list (one name per line, kept out
    of git). Without git the author check cannot run, so that is an error
    unless the caller says so (require_git=False, `--no-git`)."""
    names: set[str] = set()
    log = git("log", "--format=%an%x00%ae")
    remote = git("remote", "-v")
    if log is None or remote is None:
        if require_git:
            raise BuildError("git is unavailable, so commit-author names cannot be checked; "
                             "build inside the repository, or pass --no-git to build without "
                             "that check")
        print("  ! git unavailable (--no-git): the commit-author name check is OFF")
    for line in (log or "").splitlines():
        name, _, mail = line.partition("\x00")
        names |= name_parts(name)
        local = mail.split("@", 1)[0]
        if "noreply" not in mail and len(local) >= 4 and local.lower() not in PI_NAME_TOKENS:
            names.add(local.lower())
    for owner in re.findall(r"github\.com[:/]([^/\s]+)/", remote or ""):
        names.add(owner.lower())
    if extra_file and extra_file.exists():
        for line in extra_file.read_text(encoding="utf-8").splitlines():
            names |= name_parts(line)
    return names


def _name_regex(names: set[str]) -> re.Pattern | None:
    """Latin boundaries only, so a name written straight after Chinese text
    ('由Alice撰寫') still matches; the words of a multi-word name may be
    separated by any whitespace."""
    if not names:
        return None
    alts = [r"\s+".join(re.escape(p) for p in n.split())
            for n in sorted(names, key=len, reverse=True)]
    return re.compile(r"(?i)(?<![A-Za-z0-9_])(" + "|".join(alts) + r")(?![A-Za-z0-9_])")


def scan_text(obj, names: set[str], where: str) -> None:
    """Refuse forbidden text, person names and Simplified characters."""
    name_re = _name_regex(names)
    for path, s in walk_strings(obj):
        for rid, pat, why in FORBIDDEN_TEXT:
            m = re.search(pat, s)
            if m:
                raise PolicyViolation(
                    f"{where}: {rid} text {m.group(0)!r} at {path} ({why})")
        if name_re:
            m = name_re.search(s)
            if m:
                raise PolicyViolation(
                    f"{where}: a person's name {m.group(0)!r} at {path}")
        if ".zh" in path or path.endswith("zh"):
            bad = sorted({c for c in s if c in SIMPLIFIED})
            if bad:
                raise PolicyViolation(
                    f"{where}: Simplified characters {''.join(bad)} at {path}")


def scan_page(rel: str, text: str, names: set[str]) -> None:
    """The last line of defence: scan the rendered file itself."""
    scan_text({"page": text}, names, rel)
    # Every page, not only zh/: the language chooser carries both languages.
    bad = sorted({c for c in text if c in SIMPLIFIED})
    if bad:
        raise PolicyViolation(f"{rel}: Simplified characters {''.join(bad)}")


# ------------------------------------------------------------------ numbers
_DATES = (
    r"\b\d{4}-\d{2}-\d{2}\b",
    r"\b\d{1,2}\s+(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Sept|Oct|Nov|Dec)[a-z]*\.?(?:\s+\d{4})?",
    r"\b(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Sept|Oct|Nov|Dec)[a-z]*\.?\s+\d{4}",
    r"(?:\d{4}\s*年\s*)?\d{1,2}\s*月(?:\s*\d{1,2}\s*日)?",
    r"\d{4}\s*年",
)
# Product names that contain a number but claim nothing.
_NAMES_WITH_NUMBERS = (
    r"\b(?:MAVROS|ROS)\s*2\b",
    r"\bUnreal Engine\s*5\b",
    r"\b\d-D\b",
)
# A digit glued to a Latin letter is part of a name (P0, WP1, Ed25519,
# OpenVLA-7B), not a quantity. CJK characters do not glue: "共7點" is 7.
_NUM = re.compile(r"(?<![A-Za-z0-9_.])(\d[\d,]*(?:\.\d+)?)(?![A-Za-z0-9])(\s*%)?")
_PCT_RANGE = re.compile(r"\s*[-–]\s*\d[\d.,]*\s*%")
_SRC_NUM = re.compile(r"(?<![\d.])(\d+(?:\.\d+)?)")


def numbers(text: str) -> dict[str, bool]:
    """Numeric tokens of a sentence -> whether each is a percentage (followed
    by %, or the first end of a range that ends in %). Code spans (paths,
    identifiers, versions) and dates are not claims and are skipped."""
    t = re.sub(r"`[^`]*`", " ", text or "")
    for pat in (*_DATES, *_NAMES_WITH_NUMBERS):
        t = re.sub(pat, " ", t)
    out: dict[str, bool] = {}
    for m in _NUM.finditer(t):
        tok = m.group(1).rstrip(",.").replace(",", "")
        if not tok:
            continue
        if "." not in tok:
            tok = tok.lstrip("0") or "0"
        pct = bool((m.group(2) or "").strip()) or bool(_PCT_RANGE.match(t, m.end()))
        out[tok] = out.get(tok, False) or pct
    return out


def source_values(text: str) -> set[float]:
    return {round(float(x), 9) for x in _SRC_NUM.findall(text)}


def json_values(node) -> set[float]:
    """Every number in a JSON subtree: number values, and numbers written
    inside string values ("29/29"). Keys are names, not values."""
    out: set[float] = set()
    if isinstance(node, bool) or node is None:
        return out
    if isinstance(node, (int, float)):
        out.add(round(float(node), 9))
    elif isinstance(node, str):
        out |= source_values(node)
    elif isinstance(node, dict):
        for v in node.values():
            out |= json_values(v)
    elif isinstance(node, list):
        for v in node:
            out |= json_values(v)
    return out


def number_in(tok: str, pct: bool, values: set[float],
              scoped: set[float] | frozenset = frozenset()) -> bool:
    """`values` come from a whole file or CHANGELOG section and must match
    exactly (a percentage may match its fraction: 98.5 % and 0.985).
    `scoped` values are the one value a JSON path names, so a rounded figure
    matches it: 0.6265 for 0.626506, 69.7 % for 0.6971."""
    v = float(tok)
    if round(v, 9) in values:
        return True
    if pct and round(v / 100.0, 9) in values:
        return True
    places = len(tok.split(".", 1)[1]) if "." in tok else 0
    tol = 0.5 * 10 ** -places + 1e-9
    return any(abs(s - v) <= tol or (pct and abs(s * 100.0 - v) <= tol) for s in scoped)


class Sources:
    """The files curated content cites. Three forms:

    * CHANGELOG.md#<anchor> - one CHANGELOG section, without its non-claim
      subsections;
    * path/file.json#a.b.0.c - the one value (or subtree) a JSON path names.
      A JSON file is always cited this way: a whole data file holds so many
      numbers that almost any figure would be found somewhere in it;
    * any other file path - the whole text of that file."""

    def __init__(self, root: Path = ROOT, changelog_text: str | None = None):
        self.root = root
        self._cache: dict[str, str] = {}
        self._json: dict[str, object] = {}
        self.changelog = parse_changelog(
            changelog_text if changelog_text is not None
            else (root / "CHANGELOG.md").read_text(encoding="utf-8"))

    def text(self, ref: str) -> str:
        if ref in self._cache:
            return self._cache[ref]
        if ref.startswith("CHANGELOG.md#"):
            anchor = ref.split("#", 1)[1]
            if anchor not in self.changelog:
                raise BuildError(f"CHANGELOG has no section {anchor!r} (cited as {ref})")
            # Never the Retracted / Found / Known limitations subsections.
            out = self.changelog[anchor]["claims_text"]
        else:
            p = self.root / ref
            if not p.is_file():
                raise BuildError(f"cited source {ref} does not exist")
            out = p.read_text(encoding="utf-8", errors="replace")
        self._cache[ref] = out
        return out

    def json_node(self, ref: str):
        """The value a 'file.json#dotted.path' reference names."""
        fname, _, path = ref.partition("#")
        if not path:
            raise BuildError(f"cited source {ref}: cite the value a number comes from, as "
                             f"{fname}#path.to.value, not the whole data file")
        if fname not in self._json:
            p = self.root / fname
            if not p.is_file():
                raise BuildError(f"cited source {fname} does not exist")
            self._json[fname] = read_json(p)
        node = self._json[fname]
        for part in path.split("."):
            if isinstance(node, dict) and part in node:
                node = node[part]
            elif isinstance(node, list) and part.isdigit() and int(part) < len(node):
                node = node[int(part)]
            else:
                raise BuildError(f"cited source {ref}: {fname} has nothing at {part!r}")
        return node

    def values(self, ref: str) -> tuple[set[float], set[float]]:
        """(exact values, scoped values) a reference supports."""
        if ref.split("#", 1)[0].endswith(".json"):
            return set(), json_values(self.json_node(ref))
        return source_values(self.text(ref)), set()


# Subsections of a CHANGELOG section that state no current claim: what was
# withdrawn or corrected, what was found and not fixed, what was not measured.
# A number that appears only there cannot support a published sentence.
NON_CLAIM_SUBSECTION = re.compile(
    r"(?i)retract|withdrawn|^correct(ed|ions?)\b|^found\b|^known\s+(limitations?|issues?)\b|"
    r"^not\s+measured\b")
_ISO_DATE = re.compile(r"\d{4}-\d{2}-\d{2}")


def parse_changelog(text: str) -> dict[str, dict]:
    """anchor -> {heading, text, claims_text, date, order}. The anchor is the
    heading up to the em dash: '2026-10-06', '2026-09-29 (evening)', '0.5.1'.

    `text` is the whole section. `claims_text` leaves out every subsection
    NON_CLAIM_SUBSECTION names (and any heading nested under one); numbers are
    checked against it, so a figure that survives only in a Retracted list
    supports nothing. `date` is the latest ISO date in the heading, or None."""
    out: dict[str, dict] = {}
    cur = None
    skip_level = None          # level of the non-claim heading being skipped
    for line in text.splitlines():
        m = re.match(r"^(#{2,4}) (.+)$", line)
        level = len(m.group(1)) if m else 0
        head = m.group(2).strip() if m else ""
        is_section = bool(m) and (level == 2 or (level == 3 and _ISO_DATE.match(head)))
        if is_section:
            anchor = head.split(" — ")[0].strip().strip("[]")
            anchor = re.sub(r"^\[([^\]]+)\]$", r"\1", anchor)
            cur = anchor
            skip_level = None
            dates = _ISO_DATE.findall(head)
            if cur in out:      # a repeated heading extends its section, never replaces it
                out[cur]["lines"].append(line)
                out[cur]["claim_lines"].append(line)
                if dates:
                    out[cur]["date"] = max([out[cur]["date"] or "", *dates])
            else:
                out[cur] = {"heading": head, "lines": [line], "claim_lines": [line],
                            "date": max(dates) if dates else None, "order": len(out)}
            continue
        if m and (skip_level is None or level <= skip_level):
            skip_level = level if NON_CLAIM_SUBSECTION.search(head) else None
        if cur is not None:
            out[cur]["lines"].append(line)
            if skip_level is None:
                out[cur]["claim_lines"].append(line)
    for v in out.values():
        v["text"] = "\n".join(v.pop("lines"))
        v["claims_text"] = "\n".join(v.pop("claim_lines"))
    return out


def sections_after(changelog: dict[str, dict], as_of: str) -> list[str]:
    """CHANGELOG sections dated after the site's as_of date: work the site
    cannot be showing yet. Non-empty means site.json needs re-curating."""
    return sorted(a for a, s in changelog.items() if s["date"] and s["date"] > as_of)


def cited_sections(content: dict) -> set[str]:
    """CHANGELOG anchors the curated content cites, directly or as sources."""
    out = {c["changelog"] for c in content.get("changes", [])}
    for _path, _unit, src in text_units(content):
        out |= {r.split("#", 1)[1] for r in src if r.startswith("CHANGELOG.md#")}
    for h in content.get("highlights", []):
        out |= {r.split("#", 1)[1] for r in h.get("src", ()) if r.startswith("CHANGELOG.md#")}
    return out


def watched_sections(changelog: dict[str, dict], content: dict) -> dict[str, str]:
    """anchor -> digest of the section text, for every section the site
    cites, every undated section and every section dated on or after as_of.
    Work is often added under an existing dated heading (the day's section
    grows), which sections_after cannot see; a changed digest can."""
    cited = cited_sections(content)
    as_of = content["as_of"]
    return {a: sha12(s["text"].encode("utf-8")) for a, s in changelog.items()
            if a in cited or s["date"] is None or s["date"] >= as_of}


def changelog_drift(recorded: dict | None, current: dict[str, str]) -> list[str]:
    """Watched CHANGELOG sections that changed, appeared or disappeared since
    the build that wrote `recorded` (build_manifest.json)."""
    if recorded is None:
        return ["(no build manifest with section digests)"]
    keys = set(recorded) | set(current)
    return sorted(k for k in keys if recorded.get(k) != current.get(k))


def text_units(obj, inherited_src=(), path="$"):
    """Yield (path, unit, src) for every {"en": str, "zh": str} pair in the
    curated content, with the nearest "src" list above it (or on it)."""
    if isinstance(obj, dict):
        src = tuple(obj.get("src", ())) or inherited_src
        if "changelog" in obj:
            src = (f"CHANGELOG.md#{obj['changelog']}", *src)
        if isinstance(obj.get("en"), str) and isinstance(obj.get("zh"), str):
            yield path, obj, src
        for k, v in obj.items():
            if k not in ("en", "zh", "src"):
                yield from text_units(v, src, f"{path}.{k}")
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from text_units(v, inherited_src, f"{path}[{i}]")


def check_number_words(path: str, unit: dict) -> None:
    """Quantities in curated text are written as digits (NUMBER_WORDS)."""
    for lang in LANGS:
        m = NUMBER_WORDS[lang].search(unit[lang])
        if m:
            raise BuildError(f"{path}.{lang}: write {m.group(0)!r} as digits, so the number "
                             f"is checked against its source and the other language")


def check_numbers(content: dict, sources: Sources) -> int:
    """Every number in curated text appears in a file it cites, and the two
    languages carry the same numbers. Returns how many numbers were checked."""
    checked = 0
    cache: dict[tuple, tuple[set[float], set[float]]] = {}

    def values_for(src: tuple) -> tuple[set[float], set[float]]:
        if src not in cache:
            exact, scoped = set(), set()
            for r in src:
                e, s = sources.values(r)
                exact |= e
                scoped |= s
            cache[src] = (exact, scoped)
        return cache[src]

    for path, unit, src in text_units(content):
        check_number_words(path, unit)
        en, zh = numbers(unit["en"]), numbers(unit["zh"])
        if set(en) != set(zh):
            raise BuildError(
                f"{path}: numbers differ between en {sorted(en)} and zh {sorted(zh)}")
        if not en:
            continue
        if not src:
            raise BuildError(f"{path}: numbers {sorted(en)} but no source cited")
        exact, scoped = values_for(src)
        for tok, pct in en.items():
            if not number_in(tok, pct or zh.get(tok, False), exact, scoped):
                raise BuildError(f"{path}: {tok} is not in its sources {list(src)}")
            checked += 1
    # Stat-tile values carry no language; they are checked against the
    # tile's own sources.
    for i, h in enumerate(content.get("highlights", [])):
        exact, scoped = values_for(tuple(h.get("src", ())))
        for tok, pct in numbers(h["value"]).items():
            if not number_in(tok, pct, exact, scoped):
                raise BuildError(f"highlights[{i}].value: {tok} is not in {h.get('src')}")
            checked += 1
    return checked


# The grant's acceptance-KPI names (Stress Testing p6, Safety Shield p6), in
# both languages. The grant measures every reported number on the hil
# topology, so an overview tile that names one must come from a hil run.
KPI_NAMES = re.compile(
    r"(?i)escape\s+rate|fail-?safe|mission\s+success|repair\s+(success|magnitude|count)|"
    r"time\s+to\s+safe|逃逸率|任務成功|修正成功|修正幅度|修正次數|回到安全")


def check_highlights(content: dict) -> None:
    """A headline tile that names an acceptance KPI must declare
    "topology": "hil"; development (dev) runs are shown on the work-package
    pages with their topology, never as a headline KPI figure."""
    for i, h in enumerate(content.get("highlights", [])):
        label = " ".join(h["label"][lg] for lg in LANGS)
        m = KPI_NAMES.search(label)
        if m and h.get("topology") != "hil":
            raise PolicyViolation(
                f"highlights[{i}]: the tile names the acceptance KPI {m.group(0)!r} but is not "
                f"from a hil run; show development runs on the work-package page instead")


_SCRIPT = re.compile(r"\.(py|sh|ps1)$")


def check_files(content: dict, root: Path = ROOT) -> int:
    """Every code path the site names exists in the repository, including
    the script of every 'Reproduce' command."""
    paths = []
    for key, wp in content.get("wps", {}).items():
        paths += [(f"wps.{key}.files", f) for f in wp.get("files", [])]
        for i, d in enumerate(wp.get("done", [])):
            paths += [(f"wps.{key}.done[{i}]", f) for f in d.get("files", [])]
        for i, s in enumerate(wp.get("shown", [])):
            if not s.get("command"):
                continue
            script = next((t for t in s["command"].split() if _SCRIPT.search(t)), None)
            if script is None:
                raise BuildError(f"wps.{key}.shown[{i}].command names no script: {s['command']!r}")
            paths.append((f"wps.{key}.shown[{i}].command", script))
    for where, f in paths:
        if not (root / f.rstrip("/")).exists():
            raise BuildError(f"{where}: {f} does not exist")
    return len(paths)


def check_parity(extract: dict) -> None:
    """Tracker text was reviewed in the tracker; here only the two languages
    must carry the same numbers."""
    for name, items in extract.items():
        for it in (items if isinstance(items, list) else [items]):
            for f in it.get("en", {}):
                en, zh = numbers(it["en"][f]), numbers(it["zh"][f])
                if set(en) != set(zh):
                    raise BuildError(
                        f"tracker {name}.{it.get('id', '')}.{f}: numbers differ "
                        f"between en {sorted(en)} and zh {sorted(zh)}")


# ------------------------------------------------------------ the extract
def extract_tracker(overview: dict, flow: dict, glossary_terms: list[str],
                    allow: dict = ALLOW_LIST) -> dict:
    """Copy exactly the allow-listed fields, in en and zh. A field the list
    names but the source lacks is an error, never a silent omission."""
    check_allow_list(allow)
    files = {"content_overview.json": overview, "content_flow.json": flow}
    out: dict = {}
    for name, (fname, key, id_field, plain, text) in allow.items():
        src = files[fname].get(key)
        if src is None:
            raise BuildError(f"{fname} has no {key!r}")
        if isinstance(src, dict):
            item = {}
            for lang in LANGS:
                tr = src["i18n"][lang]
                item[lang] = {f: _need(tr, f, f"{fname}:{key}.{lang}") for f in text}
            out[name] = item
            continue
        items = []
        for entry in src:
            ident = entry[id_field]
            if name == "glossary" and ident not in glossary_terms:
                continue
            item = {"id": ident}
            for p in plain:
                item[p] = _need(entry, p, f"{fname}:{key}[{ident}]")
            for lang in LANGS:
                tr = entry["i18n"][lang]
                if text is None:
                    item[lang] = {"text": tr}
                else:
                    item[lang] = {f: _need(tr, f, f"{fname}:{key}[{ident}].{lang}") for f in text}
            items.append(item)
        if name == "glossary":
            have = {i["id"] for i in items}
            lost = [t for t in glossary_terms if t not in have]
            if lost:
                raise BuildError(f"glossary terms not in the tracker: {lost}")
        out[name] = items
    assert_no_disallowed_keys(out, "tracker extract")
    return out


def _need(d: dict, field: str, where: str):
    if field not in d:
        raise BuildError(f"{where} has no field {field!r}")
    return d[field]


def apply_overrides(extract: dict, overrides: dict) -> dict:
    """site.json 'overrides' replace one copied field with curated public
    wording: {"section.item_id.field": {"en": .., "zh": ..}} for a text field,
    {"section.item_id.field": "..."} for a plain field such as grant_quote.
    The value's shape must match the field's, or the build stops."""
    for spec, val in overrides.items():
        section, _, rest = spec.partition(".")
        ident, _, field = rest.rpartition(".")
        items = extract.get(section)
        if items is None:
            raise BuildError(f"override {spec}: no section {section!r}")
        targets = [items] if isinstance(items, dict) else [i for i in items if i["id"] == ident]
        if not targets:
            raise BuildError(f"override {spec}: no such item")
        for t in targets:
            if field in t.get("en", {}):
                if not (isinstance(val, dict) and all(isinstance(val.get(lg), str) for lg in LANGS)):
                    raise BuildError(f"override {spec}: a text field takes {{'en': .., 'zh': ..}}")
                for lang in LANGS:
                    t[lang][field] = val[lang]
            elif field in t and field not in ("id", *LANGS):
                if not isinstance(val, str):
                    raise BuildError(f"override {spec}: a plain field takes one string")
                t[field] = val
            else:
                raise BuildError(f"override {spec}: no such field")
    return extract


def load_extract(content: dict, tracker_dir: Path, snapshot: Path) -> tuple[dict, str]:
    terms = content["glossary"]["from_tracker"]
    ov, fl = tracker_dir / "content_overview.json", tracker_dir / "content_flow.json"
    if ov.is_file() and fl.is_file():
        return extract_tracker(read_json(ov), read_json(fl), terms), "tracker"
    if snapshot.is_file():
        print(f"  ! tracker data not found in {tracker_dir}; building from the "
              f"committed snapshot {snapshot.relative_to(ROOT) if snapshot.is_relative_to(ROOT) else snapshot}")
        snap = read_json(snapshot)
        assert_no_disallowed_keys(snap, "tracker snapshot")
        return snap, "snapshot"
    raise BuildError("neither tracker/data nor the committed snapshot exists")


# ------------------------------------------------------------------- media
class Media:
    def __init__(self, root: Path = ROOT, tracked: set[str] | None = None, *,
                 require_git: bool = True):
        self.root = root
        if tracked is None:
            listing = git("ls-files", "-z", "--", "docs")
            tracked = set(listing.split("\0")) if listing is not None else None
            if tracked is None:
                if require_git:
                    raise BuildError("git is unavailable, so images cannot be checked for being "
                                     "committed; pass --no-git to build without that check")
                print("  ! git unavailable (--no-git): images are not checked for being "
                      "tracked, and no video is linked")
        self.tracked = tracked
        # path -> image_digest() of the file as its wording was last reviewed
        # (site.json "reviewed_images"); None skips the check (unit tests).
        self.reviewed: dict[str, str] | None = None
        self.used: set[str] = set()

    def image(self, rel: str) -> str:
        """An image under docs/img (committed by git) or under the site's own
        folder (docs/progress/assets/img, committed with the pages). The text
        scan cannot read an image, so each one must also be listed, with its
        digest, in site.json "reviewed_images": an image that changed since
        its content was last checked is refused until someone looks again."""
        p = self.root / rel
        if not p.is_file():
            raise BuildError(f"image {rel} does not exist")
        site_asset = rel.startswith(SITE_ASSETS)
        if not site_asset and self.tracked is not None and rel not in self.tracked:
            raise BuildError(f"image {rel} is not tracked by git, so it would not be on Pages")
        if self.reviewed is not None:
            want = self.reviewed.get(rel)
            got = image_digest(p.read_bytes())
            if want is None:
                raise PolicyViolation(
                    f"image {rel} is not in site.json reviewed_images: look at what it shows "
                    f"(the text checks cannot), then list it with digest {got}")
            if want != got:
                raise PolicyViolation(
                    f"image {rel} changed since it was reviewed ({want} -> {got}): look at it "
                    f"again before publishing, then update its digest in reviewed_images")
        self.used.add(rel)
        return rel

    def video(self, rel: str, recorded_bytes: int) -> dict:
        """{'embed': bool, 'bytes': int}. Embedded only when the file is
        committed and small; the size shown is the recorded one, so the page
        is the same on a clone that lacks the file."""
        p = self.root / rel
        if p.is_file() and p.stat().st_size != recorded_bytes:
            raise BuildError(f"video {rel}: site.json says {recorded_bytes} bytes, "
                             f"the file has {p.stat().st_size}")
        tracked = self.tracked is not None and rel in self.tracked
        return {"embed": tracked and recorded_bytes <= MAX_VIDEO_BYTES and p.is_file(),
                "bytes": recorded_bytes}


def mb(n: int) -> str:
    return f"{n / 1_000_000:.1f} MB"


# ------------------------------------------------------------------ dates
MONTHS_EN = ("Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")


def fmt_date(iso: str, lang: str) -> str:
    """2026-10-06 -> '6 Oct 2026' / '2026 年 10 月 6 日'; 2026-07 -> month only."""
    parts = [int(x) for x in iso.split("-")]
    if len(parts) == 2:
        y, m = parts
        return f"{MONTHS_EN[m - 1]} {y}" if lang == "en" else f"{y} 年 {m} 月"
    y, m, d = parts
    return f"{d} {MONTHS_EN[m - 1]} {y}" if lang == "en" else f"{y} 年 {m} 月 {d} 日"


# --------------------------------------------------------------- rendering
ICON = ("<svg viewBox='0 0 32 32' aria-hidden='true'><path d='M16 3l11 4v8c0 7-5 12-11 14"
        "C10 27 5 22 5 15V7z'/><path d='M11 16l4 4 7-8' class='tick'/></svg>")
FAVICON = ("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E"
           "%3Cpath d='M16 3l11 4v8c0 7-5 12-11 14C10 27 5 22 5 15V7z' fill='%23249DB2'/%3E"
           "%3Cpath d='M11 16l4 4 7-8' stroke='white' stroke-width='3' fill='none' "
           "stroke-linecap='round'/%3E%3C/svg%3E")


class Site:
    """Everything a page needs: curated content, the tracker extract, media."""

    def __init__(self, content: dict, extract: dict, media: Media, sources: Sources):
        self.c = content
        self.x = extract
        self.media = media
        self.sources = sources

    # -- small lookups
    def ui(self, lang: str, key: str, **kw) -> str:
        s = self.c["ui"][lang][key]
        return s.format(**kw) if kw else s

    def t(self, unit: dict, lang: str) -> str:
        return inline(unit[lang])

    def img_href(self, rel: str) -> str:
        rel = self.media.image(rel)
        if rel.startswith("docs/progress/"):
            return "../" + rel[len("docs/progress/"):]
        return "../../" + rel[len("docs/"):]

    def wp_page(self, key: str) -> str:
        return key.lower() + ".html"

    def chips(self, wps: list[str]) -> str:
        return " ".join(f'<a class="chip" href="{self.wp_page(w)}">{esc(w)}</a>' for w in wps)

    # -- layout
    def layout(self, lang: str, page: str, title: str, body: str) -> str:
        nav = []
        for p in PAGES:
            label = self.ui(lang, "nav_" + p)
            cls = ' class="on" aria-current="page"' if p == page else ""
            nav.append(f'<a href="{p}.html"{cls}>{esc(label)}</a>')
        langs = []
        for lg in LANGS:
            cls = ' class="on"' if lg == lang else ""
            langs.append(f'<a href="../{lg}/{page}.html" hreflang="{HTML_LANG[lg]}" '
                         f'lang="{HTML_LANG[lg]}"{cls}>{LANG_LABEL[lg]}</a>')
        alt = "".join(f'<link rel="alternate" hreflang="{HTML_LANG[lg]}" href="../{lg}/{page}.html">'
                      for lg in LANGS)
        g = self.c["grant"]
        foot = self.ui(lang, "footer", as_of=fmt_date(self.c["as_of"], lang))
        return f"""<!doctype html>
{GENERATED}
<html lang="{HTML_LANG[lang]}">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{esc(title)} · {esc(self.ui(lang, "site_name"))}</title>
<meta name="description" content="{esc(self.x['project'][lang]['one_liner'])}">
<link rel="icon" href="{FAVICON}">
<link rel="stylesheet" href="../assets/site.css">
{alt}
</head>
<body>
<a class="skip" href="#main">{esc(self.ui(lang, "skip"))}</a>
<header class="top">
<a class="brand" href="index.html">{ICON}<span>{esc(self.ui(lang, "site_name"))}</span></a>
<nav aria-label="{esc(self.ui(lang, "nav_label"))}">{"".join(nav)}</nav>
<div class="lang" aria-label="{esc(self.ui(lang, "lang_label"))}">{"".join(langs)}</div>
</header>
<main id="main">
{body}
</main>
<footer>
<p>{inline(foot)}</p>
<p class="muted">{esc(g["programme"][lang])} · {esc(g["title"])} · {esc(self.ui(lang, "pi"))}{esc(self.ui(lang, "colon"))}{esc(g["pi"][lang])}</p>
</footer>
</body>
</html>
"""

    # -- pieces
    def page_head(self, lang: str, kicker: str, title: str, lead: str = "") -> str:
        lead_html = f'<p class="lead">{lead}</p>' if lead else ""
        return (f'<section class="page-head"><p class="kicker">{esc(kicker)}</p>'
                f"<h1>{esc(title)}</h1>{lead_html}</section>")

    def roadmap_next(self, wp: str | None = None) -> list[dict]:
        out = []
        for win in self.c["roadmap"]["next"]:
            for it in win["items"]:
                if wp is None or wp in it.get("wp", []):
                    out.append({"window": win["window"], **it})
        return out

    def gallery_card(self, lang: str, g: dict) -> str:
        title = self.t(g["title"], lang)
        cap = self.t(g["caption"], lang)
        meta = esc(fmt_date(g["date"], lang)) if g.get("date") else ""
        wps = self.chips(g.get("wp", []))
        if g["kind"] == "image":
            href = self.img_href(g["file"])
            alt = esc(g["title"][lang])
            media = (f'<a class="shot" href="{href}"><img src="{href}" alt="{alt}" loading="lazy"></a>')
        else:
            v = self.media.video(g["file"], g["bytes"])
            poster = self.img_href(g["poster"]) if g.get("poster") else ""
            if v["embed"]:
                media = (f'<video controls preload="none" src="../../{esc(g["file"][5:])}"'
                         + (f' poster="{poster}"' if poster else "") + "></video>")
            else:
                name = Path(g["file"]).name
                note = self.ui(lang, "video_placeholder", name=name, size=mb(v["bytes"]))
                bg = f' style="background-image:url(\'{poster}\')"' if poster else ""
                media = (f'<div class="video-ph"{bg} role="img" aria-label="{esc(note)}">'
                         f'<span class="play" aria-hidden="true">&#9654;</span>'
                         f'<span class="ph-note">{inline(note)}</span></div>')
        return (f'<article class="card media" id="{esc(g["id"])}">{media}<div class="body">'
                f'<h3>{title}</h3><p class="meta">{meta} {wps}</p><p>{cap}</p></div></article>')

    # ---------------------------------------------------------------- pages
    def page_index(self, lang: str) -> str:
        c, x = self.c, self.x
        p = x["project"][lang]
        g = c["grant"]
        as_of = date.fromisoformat(c["as_of"])
        days = (date.fromisoformat(c["final_gate"]) - as_of).days
        facts = "".join(
            f"<div><dt>{esc(self.ui(lang, k))}</dt><dd>{v}</dd></div>" for k, v in (
                ("programme", esc(g["programme"][lang])),
                ("grant_title", esc(g["title"])),
                ("pi", esc(g["pi"][lang])),
                ("period", esc(fmt_date(g["start"], lang) + " – " + fmt_date(g["end"], lang))),
            ))
        tiles = "".join(
            f'<div class="stat"><div class="big">{esc(h["value"])}</div>'
            f'<div class="label">{self.t(h["label"], lang)}</div>'
            f'<div class="ctx">{self.t(h["context"], lang)}</div></div>'
            for h in c["highlights"])
        roles = "".join(
            f'<div class="card role"><span class="num">{i}</span><div>'
            f'<h3>{inline(r[lang]["analogy"])}</h3><p>{inline(r[lang]["in_project"])}</p></div></div>'
            for i, r in enumerate(x["roles"], 1))
        wps = []
        for wp in x["wps"]:
            k = wp["id"]
            n_done = len(c["wps"][k]["done"])
            n_next = len(self.roadmap_next(k))
            wps.append(
                f'<a class="card wp" href="{self.wp_page(k)}"><p class="kicker">{k}</p>'
                f'<h3>{inline(wp[lang]["name"])}</h3><p>{inline(wp[lang]["plain"])}</p>'
                f'<p class="counts"><span class="pill done">{esc(self.ui(lang, "n_built", n=n_done))}</span> '
                f'<span class="pill next">{esc(self.ui(lang, "n_next", n=n_next))}</span></p></a>')
        latest = "".join(
            f'<li><span class="date">{esc(fmt_date(ch["date"], lang))}</span> '
            f'{self.t(ch["title"], lang)}</li>' for ch in self.changes()[:3])
        nexts = "".join(
            f'<li><span class="date">{esc(it["window"][lang])}</span> {self.t(it, lang)}</li>'
            for it in self.roadmap_next()[:4])
        body = f"""
<section class="hero">
<div class="panel">
<p class="kicker">{esc(self.ui(lang, "kicker_index"))}</p>
<h1>{esc(self.ui(lang, "hero_title"))}</h1>
<p class="lead">{inline(p["one_liner"])}</p>
<p class="muted small">{esc(self.ui(lang, "as_of", as_of=fmt_date(c["as_of"], lang)))} · {esc(self.ui(lang, "days_left", days=days, gate=fmt_date(c["final_gate"], lang)))}</p>
</div>
<dl class="panel facts">{facts}</dl>
</section>
<section>
<h2>{esc(self.ui(lang, "h_numbers"))}</h2>
<div class="grid tiles">{tiles}</div>
</section>
<section class="grid g2">
<div class="panel"><h2>{esc(self.ui(lang, "h_problem"))}</h2><p>{inline(p["problem"])}</p></div>
<div class="panel"><h2>{esc(self.ui(lang, "h_approach"))}</h2><p>{inline(p["solution"])}</p>
<p><a href="flow.html">{esc(self.ui(lang, "see_flow"))} →</a></p></div>
</section>
<section>
<h2>{esc(self.ui(lang, "h_roles"))}</h2>
<div class="grid roles">{roles}</div>
</section>
<section>
<h2>{esc(self.ui(lang, "h_wps"))}</h2>
<div class="grid two">{"".join(wps)}</div>
</section>
<section class="grid g2">
<div class="panel"><h2>{esc(self.ui(lang, "h_latest"))}</h2><ul class="dated">{latest}</ul>
<p><a href="changes.html">{esc(self.ui(lang, "all_changes"))} →</a></p></div>
<div class="panel"><h2>{esc(self.ui(lang, "h_next"))}</h2><ul class="dated">{nexts}</ul>
<p><a href="roadmap.html">{esc(self.ui(lang, "full_roadmap"))} →</a></p></div>
</section>
"""
        return self.layout(lang, "index", self.ui(lang, "nav_index"), body)

    def architecture_svg(self, lang: str) -> str:
        # Fixed layout: three columns, four rows. Labels come from the
        # tracker extract; the numbers match the legend below the figure.
        pos = {
            "operator": (30, 30), "vla": (375, 30), "stress_harness": (720, 30),
            "prefix_compiler": (30, 150), "shield": (375, 150),
            "policy_bundle": (30, 270), "mavros": (375, 270), "gcs": (720, 270),
            "ardupilot": (375, 390), "geofence": (720, 390),
        }
        order = [a["id"] for a in self.x["architecture"]]
        missing = sorted(set(pos) - set(order))
        if missing:
            raise BuildError(f"architecture items missing from the tracker extract: {missing}")
        num = {k: i for i, k in enumerate(order, 1)}
        lab = {a["id"]: a[lang]["label"] for a in self.x["architecture"]}
        W, H = 250, 64
        long_cls = ("", ' class="long"')
        e = self.c["ui"][lang]
        edges = [
            ("M280,62 H375", e["edge_task"], (300, 54)),
            ("M155,270 V214", "", None),
            ("M280,182 H328 V82 H375", e["edge_csp"], (286, 174)),
            ("M500,94 V150", e["edge_action"], (508, 126)),
            ("M280,302 H340 V200 H375", e["edge_rules"], (286, 294)),
            ("M500,214 V270", "", None),
            ("M500,334 V390", e["edge_mavlink"], (508, 366)),
            ("M625,405 H670 V302 H720", e["edge_router"], (728, 358)),
            ("M625,438 H720", "", None),
            ("M845,94 V182 H625", e["edge_scenarios"], (650, 174)),
        ]
        out = [f'<svg class="arch" viewBox="0 0 1000 480" role="img" '
               f'aria-label="{esc(self.ui(lang, "arch_alt"))}">',
               '<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
               'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z"/></marker></defs>']
        for i, (d, label, at) in enumerate(edges):
            dash = " dash" if i in (8, 9) else ""
            out.append(f'<path class="edge{dash}" d="{d}"/>')
            if label and at:
                out.append(f'<text class="elabel" x="{at[0]}" y="{at[1]}">{esc(label)}</text>')
        for key, (x0, y0) in pos.items():
            href = self.c["architecture_links"].get(key, "flow.html")
            out.append(
                f'<a href="{href}"><g class="node"><rect x="{x0}" y="{y0}" width="{W}" height="{H}" rx="10"/>'
                f'<circle cx="{x0 + 18}" cy="{y0 + 18}" r="11"/>'
                f'<text class="num" x="{x0 + 18}" y="{y0 + 22}">{num[key]}</text>'
                f'<text{long_cls[len(lab[key]) > 26]} x="{x0 + W / 2 + 10}" '
                f'y="{y0 + 39}">{esc(lab[key])}</text></g></a>')
        out.append("</svg>")
        return "".join(out)

    def page_flow(self, lang: str) -> str:
        x, c = self.x, self.c
        legend = "".join(
            f'<li><strong>{inline(a[lang]["label"])}</strong> — {inline(a[lang]["short"])}</li>'
            for a in x["architecture"])
        steps = "".join(
            f'<li><strong>{inline(s[lang]["label"])}</strong><br>{self.t(c["shield_steps"][s["id"]], lang)}</li>'
            for s in x["shield_steps"])
        pipe = "".join(
            f'<li class="card"><h3>{inline(s[lang]["label"])}</h3><p>{inline(s[lang]["short"])}</p></li>'
            for s in x["pipeline"])
        hud_img = self.img_href(c["hud_image"])
        hots = "".join(
            f'<a class="hot" href="#hud-{esc(h["id"])}" style="left:{h["x"] / 12.8:.2f}%;top:{h["y"] / 7.2:.2f}%" '
            f'aria-label="{esc(h[lang]["label"])}">{i}</a>'
            for i, h in enumerate(x["hud_callouts"], 1))
        callouts = "".join(
            f'<li id="hud-{esc(h["id"])}"><strong>{inline(h[lang]["label"])}</strong> — {inline(h[lang]["detail"])}</li>'
            for h in x["hud_callouts"])
        statuses = "".join(
            f'<li><span class="sw" style="background:{esc(s["color"])}"></span>'
            f'<strong>{inline(s[lang]["label"])}</strong> — {inline(s[lang]["meaning"])}</li>'
            for s in x["statuses"])
        topo = "".join(
            f'<div class="card topo {esc(t["stage"])}"><p class="kicker">{esc(t["name"])}</p>'
            f'<h3>{self.t(t["where"], lang)}</h3><p>{self.t(t["purpose"], lang)}</p>'
            f'<p><span class="pill {esc(t["stage"])}">{esc(self.ui(lang, "stage_" + t["stage"]))}</span></p></div>'
            for t in c["topologies"])
        body = f"""
{self.page_head(lang, self.ui(lang, "kicker_flow"), self.ui(lang, "nav_flow"), inline(self.ui(lang, "flow_lead")))}
<section class="panel">
<h2>{esc(self.ui(lang, "h_arch"))}</h2>
<p>{inline(self.ui(lang, "arch_lead"))}</p>
<div class="scroll-x">{self.architecture_svg(lang)}</div>
<ol class="legend">{legend}</ol>
</section>
<section class="panel">
<h2>{esc(self.ui(lang, "h_tick"))}</h2>
<p>{inline(self.ui(lang, "tick_lead"))}</p>
<ol class="steps">{steps}</ol>
</section>
<section>
<h2>{esc(self.ui(lang, "h_pipeline"))}</h2>
<p>{inline(self.ui(lang, "pipeline_lead"))}</p>
<ol class="pipeline">{pipe}</ol>
</section>
<section class="panel">
<h2>{esc(self.ui(lang, "h_hud"))}</h2>
<p>{inline(self.ui(lang, "hud_lead"))}</p>
<div class="hud"><img src="{hud_img}" alt="{esc(self.ui(lang, "hud_alt"))}" loading="lazy">{hots}</div>
<ol class="callouts">{callouts}</ol>
<h3>{esc(self.ui(lang, "h_statuses"))}</h3>
<ul class="statuses">{statuses}</ul>
</section>
<section>
<h2>{esc(self.ui(lang, "h_topo"))}</h2>
<p>{inline(self.ui(lang, "topo_lead"))}</p>
<div class="grid g3">{topo}</div>
</section>
"""
        return self.layout(lang, "flow", self.ui(lang, "nav_flow"), body)

    def page_wp(self, lang: str, key: str) -> str:
        wp = next(w for w in self.x["wps"] if w["id"] == key)
        cw = self.c["wps"][key]
        done = "".join(
            f'<li><h3>{self.t(d["title"], lang)}</h3><p>{self.t(d["text"], lang)}</p>'
            + (f'<p class="files">{" ".join(f"<code>{esc(f)}</code>" for f in d.get("files", []))}</p>'
               if d.get("files") else "") + "</li>"
            for d in cw["done"])
        shown_items = []
        for s in cw["shown"]:
            fig = ""
            if s.get("image"):
                href = self.img_href(s["image"])
                fig = (f'<a class="shot" href="{href}"><img src="{href}" alt="{esc(s["title"][lang])}" '
                       f'loading="lazy"></a>')
            cmd = (f'<p class="files">{esc(self.ui(lang, "reproduce"))}{esc(self.ui(lang, "colon"))}<code>{esc(s["command"])}</code></p>'
                   if s.get("command") else "")
            shown_items.append(f'<li class="card">{fig}<h3>{self.t(s["title"], lang)}</h3>'
                               f'<p>{self.t(s["text"], lang)}</p>{cmd}</li>')
        nxt = "".join(
            f'<li><span class="pill next">{esc(self.ui(lang, "stage_next"))}</span> '
            f'<span class="date">{esc(it["window"][lang])}</span> {self.t(it, lang)}</li>'
            for it in self.roadmap_next(key))
        files = " ".join(f"<code>{esc(f)}</code>" for f in cw.get("files", []))
        i = WP_KEYS.index(key)
        prev_l = (f'<a href="{self.wp_page(WP_KEYS[i - 1])}">← {WP_KEYS[i - 1]}</a>' if i > 0 else "<span></span>")
        next_l = (f'<a href="{self.wp_page(WP_KEYS[i + 1])}">{WP_KEYS[i + 1]} →</a>' if i < 3 else "<span></span>")
        body = f"""
{self.page_head(lang, f"{key} · {self.ui(lang, 'kicker_wp')}", wp[lang]["name"], inline(wp[lang]["plain"]))}
<section class="panel">
<h2>{esc(self.ui(lang, "h_goal"))}</h2>
<blockquote>“{inline(wp["grant_quote"])}”<cite>{esc(self.ui(lang, "grant_page", page=wp["grant_page"]))}</cite></blockquote>
<p class="files">{esc(self.ui(lang, "main_files"))}{esc(self.ui(lang, "colon"))}{files}</p>
</section>
<section>
<h2>{esc(self.ui(lang, "h_built"))}</h2>
<ul class="built">{done}</ul>
</section>
<section>
<h2>{esc(self.ui(lang, "h_shown"))}</h2>
<ul class="grid g2 shown">{"".join(shown_items)}</ul>
</section>
<section class="panel">
<h2>{esc(self.ui(lang, "h_wp_next"))}</h2>
<ul class="next-list">{nxt}</ul>
<p><a href="roadmap.html">{esc(self.ui(lang, "full_roadmap"))} →</a></p>
</section>
<p class="pager">{prev_l}{next_l}</p>
"""
        return self.layout(lang, key.lower(), f"{key} · {wp[lang]['name']}", body)

    def page_gallery(self, lang: str) -> str:
        cards = "".join(self.gallery_card(lang, g) for g in self.c["gallery"])
        body = f"""
{self.page_head(lang, self.ui(lang, "kicker_gallery"), self.ui(lang, "nav_gallery"), inline(self.ui(lang, "gallery_lead")))}
<section class="grid g2 gallery">{cards}</section>
"""
        return self.layout(lang, "gallery", self.ui(lang, "nav_gallery"), body)

    def changes(self) -> list[dict]:
        """Curated change entries, newest first by CHANGELOG order."""
        out = []
        for ch in self.c["changes"]:
            sec = self.sources.changelog.get(ch["changelog"])
            if sec is None:
                raise BuildError(f"change entry anchors to missing CHANGELOG section {ch['changelog']!r}")
            out.append({**ch, "_order": sec["order"]})
        return sorted(out, key=lambda c: c["_order"])

    def page_changes(self, lang: str) -> str:
        items = []
        for ch in self.changes():
            pts = "".join(f"<li>{self.t(pt, lang)}</li>" for pt in ch["points"])
            ver = f'<span class="chip">{esc(ch["version"])}</span>' if ch.get("version") else ""
            when = fmt_date(ch["date"], lang)
            if ch.get("date_start"):
                when = fmt_date(ch["date_start"], lang) + " – " + when
            items.append(f'<li class="entry"><p class="when">{esc(when)} {ver}</p>'
                         f'<h3>{self.t(ch["title"], lang)}</h3><ul>{pts}</ul></li>')
        body = f"""
{self.page_head(lang, self.ui(lang, "kicker_changes"), self.ui(lang, "nav_changes"), inline(self.ui(lang, "changes_lead")))}
<ol class="timeline">{"".join(items)}</ol>
"""
        return self.layout(lang, "changes", self.ui(lang, "nav_changes"), body)

    def page_roadmap(self, lang: str) -> str:
        c = self.c
        start, end = date.fromisoformat(c["grant"]["start"]), date.fromisoformat(c["grant"]["end"])
        span = (end - start).days

        def pct(iso: str) -> float:
            d = date.fromisoformat(iso if len(iso) == 10 else iso + "-15")
            return max(0.0, min(100.0, (d - start).days / span * 100))

        months = []
        y, m = start.year, start.month
        while (y, m) <= (end.year, end.month):
            iso = f"{y}-{m:02d}-01"
            label = MONTHS_EN[m - 1] if lang == "en" else f"{m} 月"
            months.append(f'<span style="left:{pct(iso):.2f}%">{label}</span>')
            y, m = (y + 1, 1) if m == 12 else (y, m + 1)
        dots = "".join(
            f'<span class="dot" style="left:{pct(d["date"]):.2f}%" title="{esc(fmt_date(d["date"], lang))}{esc(self.ui(lang, "colon"))}'
            f'{esc(d["en"] if lang == "en" else d["zh"])}"></span>'
            for d in c["roadmap"]["done"])
        bar = (f'<div class="track" role="img" aria-label="{esc(self.ui(lang, "track_alt"))}">'
               f'<div class="filled" style="width:{pct(c["as_of"]):.2f}%"></div>{dots}'
               f'<span class="today" style="left:{pct(c["as_of"]):.2f}%"><b>{esc(self.ui(lang, "today"))}</b></span>'
               f'<span class="gate" style="left:100%"><b>{esc(fmt_date(c["final_gate"], lang))}</b></span>'
               f'</div><div class="months">{"".join(months)}</div>')
        done = "".join(
            f'<li><span class="date">{esc(fmt_date(d["date"], lang))}</span> '
            f'{inline(d[lang])} {self.chips(d.get("wp", []))}</li>'
            for d in reversed(c["roadmap"]["done"]))
        wins = []
        for win in c["roadmap"]["next"]:
            its = "".join(
                f'<li>{self.t(it, lang)} {self.chips(it.get("wp", []))}</li>'
                for it in win["items"])
            wins.append(f'<div class="card"><p class="kicker">{esc(win["window"][lang])}</p><ul>{its}</ul></div>')
        final = "".join(f"<li>{self.t(f, lang)}</li>" for f in c["roadmap"]["final_delivery"])
        body = f"""
{self.page_head(lang, self.ui(lang, "kicker_roadmap"), self.ui(lang, "nav_roadmap"), inline(self.ui(lang, "roadmap_lead")))}
<section class="panel">{bar}</section>
<section>
<h2><span class="pill next">{esc(self.ui(lang, "stage_next"))}</span> {esc(self.ui(lang, "h_next_windows"))}</h2>
<div class="grid two">{"".join(wins)}</div>
</section>
<section class="panel gate-box">
<h2>{esc(self.ui(lang, "h_final", gate=fmt_date(c["final_gate"], lang)))}</h2>
<ul>{final}</ul>
</section>
<section class="panel">
<h2><span class="pill done">{esc(self.ui(lang, "stage_done"))}</span> {esc(self.ui(lang, "h_done"))}</h2>
<ul class="dated">{done}</ul>
</section>
"""
        return self.layout(lang, "roadmap", self.ui(lang, "nav_roadmap"), body)

    def page_glossary(self, lang: str) -> str:
        entries = [(g["id"], g[lang]["text"]) for g in self.x["glossary"]]
        entries += [(g["term"], g[lang]) for g in self.c["glossary"]["extra"]]
        entries.sort(key=lambda e: e[0].lower())
        dl = "".join(f'<div><dt id="{esc(re.sub(r"[^a-z0-9]+", "-", t.lower()).strip("-"))}">{esc(t)}</dt>'
                     f"<dd>{inline(d)}</dd></div>" for t, d in entries)
        body = f"""
{self.page_head(lang, self.ui(lang, "kicker_glossary"), self.ui(lang, "nav_glossary"), inline(self.ui(lang, "glossary_lead")))}
<dl class="glossary">{dl}</dl>
"""
        return self.layout(lang, "glossary", self.ui(lang, "nav_glossary"), body)

    def page_root(self) -> str:
        cards = "".join(
            f'<a class="card choose" href="{lg}/index.html" hreflang="{HTML_LANG[lg]}" lang="{HTML_LANG[lg]}">'
            f'<p class="kicker">{LANG_LABEL[lg]}</p><h2>{esc(self.ui(lg, "hero_title"))}</h2>'
            f'<p>{inline(self.x["project"][lg]["one_liner"])}</p>'
            f'<p class="go">{esc(self.ui(lg, "enter"))} →</p></a>'
            for lg in LANGS)
        return f"""<!doctype html>
{GENERATED}
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{esc(self.ui("en", "site_name"))}</title>
<meta name="description" content="{esc(self.x['project']['en']['one_liner'])}">
<link rel="icon" href="{FAVICON}">
<link rel="stylesheet" href="assets/site.css">
</head>
<body class="root">
<main id="main">
<p class="brand">{ICON}<span>{esc(self.ui("en", "site_name"))}</span></p>
<div class="grid g2">{cards}</div>
<p class="muted small">{esc(self.c["grant"]["programme"]["en"])} · {esc(self.c["grant"]["title"])}</p>
</main>
</body>
</html>
"""

    def render(self) -> dict[str, str]:
        files = {"index.html": self.page_root()}
        for lang in LANGS:
            files[f"{lang}/index.html"] = self.page_index(lang)
            files[f"{lang}/flow.html"] = self.page_flow(lang)
            for k in WP_KEYS:
                files[f"{lang}/{k.lower()}.html"] = self.page_wp(lang, k)
            files[f"{lang}/gallery.html"] = self.page_gallery(lang)
            files[f"{lang}/changes.html"] = self.page_changes(lang)
            files[f"{lang}/roadmap.html"] = self.page_roadmap(lang)
            files[f"{lang}/glossary.html"] = self.page_glossary(lang)
        return files


# ------------------------------------------------------------------- build
def build(content: dict, extract: dict, *, sources: Sources | None = None,
          media: Media | None = None, names: set[str] | None = None,
          require_git: bool = True) -> tuple[dict[str, str], dict]:
    """Check everything, render every page, scan every page. Returns
    (relative path -> HTML, stats). Writes nothing."""
    sources = sources or Sources()
    media = media or Media(require_git=require_git)
    names = person_names(require_git=require_git) if names is None else names
    assert_no_disallowed_keys(content, "site.json")
    assert_no_disallowed_keys(extract, "tracker extract")
    extract = public_extract(extract, content)
    scan_text(content, names, "site.json")
    scan_text(extract, names, "tracker extract")
    check_highlights(content)
    check_parity(extract)
    check_files(content, sources.root)
    n_numbers = check_numbers(content, sources)
    if "reviewed_images" not in content:
        raise BuildError("site.json has no reviewed_images list")
    media.reviewed = content["reviewed_images"]
    media.used = set()
    site = Site(content, extract, media, sources)
    pages = site.render()
    unused = sorted(set(media.reviewed) - media.used)
    if unused:
        raise BuildError(f"reviewed_images lists images no page shows: {unused}")
    for rel, text in pages.items():
        scan_page(rel, text, names)
    placeholders = [Path(g["file"]).name for g in content["gallery"]
                    if g["kind"] == "video" and not media.video(g["file"], g["bytes"])["embed"]]
    stats = {"page_count": len(pages), "numbers_checked": n_numbers,
             "images_reviewed": len(media.used),
             "video_placeholders": placeholders,
             "changelog_sections_published": sorted(c["changelog"] for c in content["changes"]),
             "changelog_sections_after_as_of": sections_after(sources.changelog, content["as_of"]),
             "changelog_watch": watched_sections(sources.changelog, content)}
    return pages, stats


def public_extract(extract: dict, content: dict) -> dict:
    """The tracker extract with site.json's public overrides applied, as a
    copy. Both the pages and the committed snapshot are made from this."""
    return apply_overrides(json.loads(json.dumps(extract)), content.get("overrides", {}))


def write_pages(pages: dict[str, str], out: Path) -> list[str]:
    """Write the pages; remove generated pages that no longer exist."""
    written = []
    for rel, text in pages.items():
        p = out / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
        written.append(rel)
    for lang in LANGS:
        d = out / lang
        if d.is_dir():
            for f in d.glob("*.html"):
                if f"{lang}/{f.name}" not in pages:
                    f.unlink()
    return written


def write_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(obj, ensure_ascii=False, indent=1) + "\n")


def read_manifest(out: Path) -> dict | None:
    p = out / "_src" / MANIFEST_FILE.name
    try:
        return read_json(p) if p.is_file() else None
    except (OSError, ValueError):
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=OUT_DIR, help="output folder (default docs/progress)")
    ap.add_argument("--tracker-dir", type=Path, default=TRACKER_DIR)
    ap.add_argument("--changelog", type=Path, default=CHANGELOG,
                    help="the CHANGELOG to read (default CHANGELOG.md; for tests)")
    ap.add_argument("--check", action="store_true",
                    help="build in memory and exit 1 if any page on disk differs, if a CHANGELOG "
                         "section the site depends on changed since the last build, or if the "
                         "CHANGELOG has sections dated after site.json's as_of")
    ap.add_argument("--no-git", action="store_true",
                    help="build without git: no commit-author name check, no check that "
                         "images are committed, no video embedded (not for publishing)")
    a = ap.parse_args(argv)
    try:
        content = read_json(CONTENT_FILE)
        extract, origin = load_extract(content, a.tracker_dir, SNAPSHOT_FILE)
        sources = Sources(changelog_text=a.changelog.read_text(encoding="utf-8"))
        pages, stats = build(content, extract, sources=sources, require_git=not a.no_git)
    except (PolicyViolation, BuildError) as e:
        print(f"REFUSED: {type(e).__name__}: {e}")
        return 2
    newer = stats["changelog_sections_after_as_of"]
    if newer:
        print(f"  ! site.json is as of {content['as_of']}, but the CHANGELOG has later "
              f"sections {newer}: re-curate site.json and move as_of before publishing "
              f"(docs/DESIGN-public-site.md, How to update)")
    previous = read_manifest(a.out)
    drift = changelog_drift((previous or {}).get("changelog_watch"), stats["changelog_watch"])
    for anchor in drift:
        print(f"  changelog changed: {anchor}")
    if drift:
        print("  ! these CHANGELOG sections changed since the last build: read the Built, "
              "Shown and Next items in site.json against them before publishing "
              "(docs/DESIGN-public-site.md, How to update, step 1)")
    if a.check:
        stale = [rel for rel, text in pages.items()
                 if not (a.out / rel).is_file() or (a.out / rel).read_text(encoding="utf-8") != text]
        print(f"{len(pages) - len(stale)}/{len(pages)} pages up to date")
        for rel in stale:
            print(f"  stale: {rel}")
        return 1 if stale or newer or drift else 0
    written = write_pages(pages, a.out)
    if origin == "tracker":
        # Stored with the public overrides applied, so the committed snapshot
        # holds exactly the wording the site shows. Overrides are idempotent,
        # so a build from the snapshot renders the same pages.
        write_json(a.out / "_src" / SNAPSHOT_FILE.name, public_extract(extract, content))
    manifest = {
        "generator": "tools/build_public_site.py",
        "as_of": content["as_of"],
        "tracker_text_from": origin,
        "sources": {"CHANGELOG.md": sha12(a.changelog.read_bytes()),
                    **{rel: sha12((ROOT / rel).read_bytes()) for rel in (
                        "docs/progress/_src/site.json", "docs/progress/_src/tracker_extract.json")
                       if (ROOT / rel).is_file()}},
        "pages": sorted(written),
        **stats,
    }
    write_json(a.out / "_src" / MANIFEST_FILE.name, manifest)
    print(f"Wrote {len(written)} pages to {a.out} (tracker text from {origin}); "
          f"{stats['numbers_checked']} numbers checked against their sources; "
          f"{stats['images_reviewed']} reviewed images; "
          f"{len(stats['video_placeholders'])} video placeholders.")
    print("Preview: python -m http.server -d docs 8000, then open "
          "http://127.0.0.1:8000/progress/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
