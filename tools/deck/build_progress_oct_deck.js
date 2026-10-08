/*
 * Guardrail progress report for ITRI's monthly report, October 2026.
 * nathan-deck design system (tools/deck/nathan_theme.js), same conventions as
 * tools/deck/build_progress_0930_deck.js.
 *
 * Every measured number is read at build time from docs/data/progress_oct.json
 * (tools/deck/build_progress_oct_data.py), which reads each artefact or re-runs
 * the code that produced it. Plan dates and the grant's own terms are named
 * constants below, each with its source.
 *
 * Audience and rule (PI, 6 Oct): this pack goes to ITRI. It reports progress
 * and has no slide of obstacles or shortcomings, but every statement on it is
 * true, uses the corrected numbers, and never repeats a retracted claim.
 *
 * Order of the build, each step refusing on failure:
 *   1. `build_progress_oct_data.py --verify`: the stored numbers are still
 *      what the repository produces, and the Shield timing measured the code
 *      in the tree now.
 *   2. No stray copy of the deck in the share folder.
 *   3. The text of every slide and speaker note is dumped to
 *      _build/slides_text.json and the README (cover note included) is
 *      drafted from the same numbers; `--check-deck ... --cover ...` refuses a
 *      withdrawn claim or a number printed as 'undefined' / 'NaN'.
 *   4. Write, repair, audit, embed fonts, set the package's last-saved-by to
 *      the lab, audit again; then README.md is written beside the deck.
 *
 *   C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_oct_data.py
 *   node tools/deck/build_progress_oct_deck.js [--allow-dirty]
 *   powershell -File tools/office_to_pdf.ps1 -Path docs/share/2026-10-ITRI/Guardrail-Progress-Oct2026.pptx
 *
 * The deck step refuses a working tree with uncommitted changes under the
 * number sources, and Shield timing of other code than the tree's (--verify
 * exit 3; the data step keeps such timing only with --keep-stale-bench).
 * --allow-dirty builds a review draft instead, marked REVIEW DRAFT on its
 * cover and in its README, which lists what made it a draft.
 * Each build writes _build/build_stamp.json (hashes of this builder, the data
 * file and the written deck), so a pack older than its builder is detectable
 * (tests/test_build_progress_oct_data.py).
 *
 * No video is embedded (the build refuses a deck over 10 MB). After the audit,
 * PowerPoint re-saves the deck with its fonts embedded, as the 30 Sept build does.
 */
const fs = require("fs");
const path = require("path");
const { execFileSync } = require("child_process");
const pptxgen = require("pptxgenjs");
const sharp = require("sharp");
const {
  FaFileSignature, FaShieldAlt, FaListOl, FaFingerprint, FaRandom, FaClipboardList,
  FaCheckCircle, FaServer, FaMicrochip, FaPlane, FaCogs, FaLayerGroup, FaTachometerAlt,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "..", "..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const DATA_PY = path.join(REPO, "tools/deck/build_progress_oct_data.py");
const readJson = (rel) => JSON.parse(fs.readFileSync(path.join(REPO, rel), "utf8"));
const SHARE = path.join(REPO, "docs/share/2026-10-ITRI");
const BUILD = path.join(SHARE, "_build");
const DECK_NAME = "Guardrail-Progress-Oct2026";
const OUT = path.join(SHARE, `${DECK_NAME}.pptx`);
const README = path.join(SHARE, "README.md");
const README_DRAFT = path.join(BUILD, "README.draft.md");
const TEXT_DUMP = path.join(BUILD, "slides_text.json");
const STAMP = path.join(BUILD, "build_stamp.json");
const DATA_JSON = path.join(REPO, "docs/data/progress_oct.json");
const sha256 = (f) => require("crypto").createHash("sha256").update(fs.readFileSync(f)).digest("hex");
// Hashed at start: on 6 Oct this file was saved again a minute after a build,
// and the pack that would have been sent did not match its builder. The stamp
// records what this run read; the end of the build refuses if either changed.
const BUILDER_SHA = sha256(__filename);
const DATA_SHA = sha256(DATA_JSON);
const MAX_MB = 10;
// Who the package says last saved it: PowerPoint's font-embedding save writes
// the Windows user's name; ITRI material names the lab, as `pres.author` does.
const LAB = "NTUT AIoT Lab";

/* A deck is never built from numbers the repository no longer produces: the
 * data step recomputes every section and checks that the stored Shield timing
 * measured the code in the tree now (`--verify`; it writes nothing). Exit 3
 * means only the Shield timing is of other code: on 7 Oct the bench refused
 * the tree's own code, and a review draft was still needed. --allow-dirty
 * then builds a DRAFT that says so on its cover and in its README; without it
 * the build stops, as for any other difference. */
const ALLOW_DIRTY = process.argv.includes("--allow-dirty");
let BENCH_STALE = false;
try {
  execFileSync(PY, [DATA_PY, "--verify"], { stdio: "inherit" });
} catch (e) {
  if (e.status === 3 && ALLOW_DIRTY) {
    BENCH_STALE = true;
    console.warn("DRAFT: the stored Shield timing measured other code than the tree's (--allow-dirty).");
  } else {
    console.error(e.status === 3
      ? "REFUSED: the stored Shield timing measured other code. Re-run tools/deck/build_progress_oct_data.py (the bench runs again); --allow-dirty builds a DRAFT only."
      : "REFUSED: docs/data/progress_oct.json is stale. Re-run tools/deck/build_progress_oct_data.py first.");
    process.exit(1);
  }
}
const D = readJson("docs/data/progress_oct.json");

/* --verify proves the numbers match the working tree, not that anyone else
 * can reproduce them: on 6 Oct the broken-policy figure (35/35, 14/25 at the
 * last commit) came from other people's uncommitted edits. A pack built from
 * uncommitted number sources is refused, unless --allow-dirty is given; then
 * the README is headed DRAFT and lists the files, so it is not sent as is.
 * `git status` with pathspecs: tracked edits and untracked files under the
 * paths the numbers are read from (gitignored flight output is not listed). */
const SOURCE_PATHS = ["CHANGELOG.md", "docs/data", "guardrail", "policies", "experiments", "tools", "demo", "deploy/evidence"];
function uncommittedSources() {
  const out = execFileSync("git", ["status", "--porcelain", "--untracked-files=all", "--", ...SOURCE_PATHS],
    { cwd: REPO, encoding: "utf8" });
  return out.split(/\r?\n/).filter((l) => l.trim());
}
const DIRTY = uncommittedSources();
if (DIRTY.length && !ALLOW_DIRTY) {
  console.error(`REFUSED: ${DIRTY.length} uncommitted change(s) under the number sources (${SOURCE_PATHS.join(", ")}):`);
  DIRTY.slice(0, 20).forEach((l) => console.error("  " + l));
  if (DIRTY.length > 20) console.error(`  ... and ${DIRTY.length - 20} more`);
  console.error("Commit first, then rebuild (data step, deck, PDF). --allow-dirty builds a DRAFT for review only.");
  process.exit(1);
}
if (DIRTY.length) console.warn(`DRAFT: building from ${DIRTY.length} uncommitted change(s) under the number sources (--allow-dirty).`);
// A draft is marked on the cover, in the README and in the build stamp.
const DRAFT = DIRTY.length > 0 || BENCH_STALE;

/* Copies such as "Guardrail-Progress-Oct2026 (1).pptx" beside the deck (one
 * was a pre-embedding build with no fonts) invite uploading the wrong file. */
function strayCopies() {
  if (!fs.existsSync(SHARE)) return [];
  return fs.readdirSync(SHARE).filter((f) => f.startsWith(DECK_NAME) && f !== `${DECK_NAME}.pptx` && f !== `${DECK_NAME}.pdf`);
}
{
  const stray = strayCopies();
  if (stray.length) {
    console.error(`REFUSED: other copies of the deck in ${SHARE}: ${stray.join(", ")}. Move them out first.`);
    process.exit(1);
  }
}

const IMG = {
  hud: path.join(REPO, "docs/img/policy_hud_nfz2.jpg"),
  track: path.join(REPO, D.chart.path),
};

/* ---- Constants with no key in progress_oct.json, each with its source ---- */
// docs/img/policy_hud_nfz2.jpg: frame of citylife_redcar_nfz2 at the HUD's t = 39.0 s.
const HUD_T_S = 39;
// The still's layout (1280 x 720 px), read off the image: banner x 340-940,
// y 10-46; rule panel x 800-1270, y 55-230; map x 1050-1270, y 490-710. Each
// numbered callout sits just outside its element, over the street scene.
const HUD_CALLOUTS = [[0.5, 0.115], [0.595, 0.19], [0.79, 0.75]];
// demo/policy_hud.py status words, in its own order (module docstring).
const HUD_STATES = ["OK", "NEAR", "AVOID", "HOLD", "ACTING", "BREACH", "BRAKE"];
// The share pack's copy of the 3 Oct flight video (docs/share/2026-09-30-ITRI).
const NFZ_VIDEO = "citylife_redcar_nfz_policy_hud.mp4";
// Grant page "Grant overview": the five acceptance KPIs, as listed.
const ACCEPTANCE_KPIS = ["mission success rate", "P0 violation escape rate (target 0)",
  "fail-safe trigger correctness", "mean repair magnitude", "mean time to safe"];
// Grant page "Architecture constraints", three deployment topologies.
const TOPOLOGIES = [
  // "Development and testing", not "and CI": the CI workflow has not run on
  // GitHub yet (CHANGELOG 2026-10-06).
  ["dev", "One desktop", "Simulators, ArduPilot SITL, MAVROS 2, Shield and VLA on one PC.", "Development and testing"],
  ["hil", "Desktop + Jetson Orin", "Desktop: Project AirSim, ArduPilot SITL, MAVROS 2, Mission Planner. Orin: VLA and Safety Shield, over the network.", "The grant's KPI configuration"],
  ["flight", "Orin on the drone", "The Orin as companion computer beside the real ArduPilot flight controller.", "Final demos"],
];
// Project AirSim (IAMAI fork) client/python/example_user_scripts/ardupilot/:
// sim_config/robot_ardu_quadrotor.jsonc "controller": {"type": "ardupilot-api",
// "ardupilot-udp-port": 9003, "local-host-udp-port": 9002}; project-airsim-quad.param
// header: sim_vehicle.py -v ArduCopter -f airsim-copter; ardupilot_quadrotor.py
// docstring: "Mission Planner can be used to control the drone."
const PAS_CONTROLLER = "ardupilot-api", PAS_PORTS = "9002 / 9003";
const SITL_CMD = "sim_vehicle.py -v ArduCopter -f airsim-copter";
// The status date is the newest dated CHANGELOG section from 6 Oct on
// (build_progress_oct_data.release_facts): work committed on 7 Oct is not
// reported as "delivered by 6 October". The period starts at the 30 Sept pack.
const STATUS = D.status_date, PERIOD_FROM = "2026-09-30";
const MONTHS = ["January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"];
const dayMonth = (iso) => { const [, m, d] = iso.split("-").map(Number); return `${d} ${MONTHS[m - 1]}`; };
const longDate = (iso) => `${dayMonth(iso)} ${iso.slice(0, 4)}`;
// The plan to 30 Nov, as of 6 Oct (critical path of the 6 Oct plan; final
// delivery date from the grant). [label, start, end] in 2026, inclusive.
const PLAN = [
  ["Deliveries across WP1-WP4 (this report)", PERIOD_FROM, STATUS, "done"],
  ["Jetson Orin bring-up (hil)", "2026-10-07", "2026-10-24"],
  ["Project AirSim + ArduPilot + Mission Planner", "2026-10-07", "2026-11-07"],
  ["FSM and GeoFence backstop in the Shield node", "2026-10-13", "2026-10-31"],
  ["VLA reading the CSP on the ArduPilot rail", "2026-10-27", "2026-11-13"],
  ["hil KPI stress campaign", "2026-11-16", "2026-11-23"],
  ["KPI report on dynamic-scenario stress", "2026-11-23", "2026-11-26"],
  ["Final demo and signed final report", "2026-11-26", "2026-11-30"],
];
const PLAN_FROM = "2026-10-05", PLAN_TO = "2026-11-30", TODAY = STATUS;
// Poppins has no U+2192: arrows are runs in Segoe UI (as in the 30 Sept deck).
const ARROW_FONT = "Segoe UI";

/* ---- Formatting ---------------------------------------------------------- */
/* Every formatter refuses a missing number: Math.round(null) is 0, and a slide
 * must not print "0.0 %" for a figure the data never had. */
const num = (v) => {
  if (typeof v !== "number" || !Number.isFinite(v)) throw new Error(`a slide number is missing or not finite: ${JSON.stringify(v)}`);
  return v;
};
const pct1 = (v) => (Math.round(num(v) * 1000 + 1e-9) / 10).toFixed(1) + " %";
const pctN = (v) => { const x = Math.round(num(v) * 1000 + 1e-9) / 10; return (Number.isInteger(x) ? String(x) : x.toFixed(1)) + " %"; };
const nf = (n) => num(n).toLocaleString("en-US");
const ms = (v) => (num(v) >= 10 ? String(Math.round(v)) : v >= 1 ? v.toFixed(1) : v.toFixed(2)) + " ms";
const m1 = (v) => (Math.round(num(v) * 10) / 10).toFixed(1);

async function photo(file) {
  const buf = fs.readFileSync(file);
  const m = await sharp(buf).metadata();
  return { data: "image/jpeg;base64," + buf.toString("base64"), aspect: m.width / m.height };
}

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "Guardrail - Progress Report, October 2026";
  pres.author = LAB;
  pres.company = LAB;
  pres.subject = "VLA-UAV Guardrail, ITRI ICL subcontract: progress for ITRI's monthly report";
  const H = helpers(pres);
  const { TEAL, TEAL_DK, TEAL_DEEP, TEAL_TINT, TEAL_TINT2, WHITE, INK, GREY, LINE, FF, CHARCOAL } = T;

  const ic = {
    sign: await iconPng(FaFileSignature), shield: await iconPng(FaShieldAlt), list: await iconPng(FaListOl),
    id: await iconPng(FaFingerprint), random: await iconPng(FaRandom), clip: await iconPng(FaClipboardList),
    ok: await iconPng(FaCheckCircle), server: await iconPng(FaServer), chip: await iconPng(FaMicrochip),
    plane: await iconPng(FaPlane), cogs: await iconPng(FaCogs), layers: await iconPng(FaLayerGroup),
    speed: await iconPng(FaTachometerAlt),
  };

  /* ---- Text record: every slide's text and notes go through the guard ---- */
  const SLIDES = [];
  let page = 0;
  function track(s, title) {
    const rec = { n: page, title, text: [], notes: "" };
    SLIDES.push(rec);
    const addText = s.addText.bind(s);
    s.addText = (t, o) => {
      rec.text.push(Array.isArray(t) ? t.map((r) => r.text).join("") : String(t));
      return addText(t, o);
    };
    const addNotes = s.addNotes.bind(s);
    s.addNotes = (n) => { rec.notes += n; return addNotes(n); };
    return s;
  }
  const content = (eyebrow, title) => {
    page++;
    const s = track(pres.addSlide(), title);
    s.background = { color: WHITE };
    H.heading(s, eyebrow, title);
    return s;
  };

  /* ---- Local helpers (30 Sept deck archetypes) ----------------------------- */
  const cardBox = (s, x, y, w, h, fill = WHITE, line = LINE) => s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x, y, w, h, fill: { color: fill }, line: { color: line, width: 1 }, rectRadius: 0.08,
    shadow: { type: "outer", blur: 6, offset: 2, angle: 90, color: "D9DEE3", opacity: 0.4 },
  });
  function source(s, txt) {
    s.addText(txt, { x: 0.55, y: 4.98, w: 8.6, h: 0.28, fontSize: 9.5, color: GREY, fontFace: FF, italic: true });
  }
  const arrow = (a, b) => [{ text: `${a} ` }, { text: "\u2192", options: { fontFace: ARROW_FONT } }, { text: ` ${b}` }];
  function arrowR(s, x1, y, x2, color = CHARCOAL) {
    s.addShape(pres.shapes.LINE, { x: x1, y, w: x2 - x1, h: 0, line: { color, width: 1.25, endArrowType: "triangle" } });
  }
  /* A vertical arrow from y1 to y2, either direction. A negative height is a
   * package PowerPoint refuses (tools/audit_pptx.py), so an upward arrow is the
   * downward shape flipped: flipV makes the line run bottom to top. */
  function arrowD(s, x, y1, y2, color = CHARCOAL) {
    const up = y2 < y1;
    s.addShape(pres.shapes.LINE, { x, y: Math.min(y1, y2), w: 0, h: Math.abs(y2 - y1), flipV: up, line: { color, width: 1.25, endArrowType: "triangle" } });
  }
  function hline(s, x1, y, x2, color = CHARCOAL, width = 1.25) {
    s.addShape(pres.shapes.LINE, { x: x1, y, w: x2 - x1, h: 0, line: { color, width } });
  }
  function vline(s, x, y1, y2, color = CHARCOAL, width = 1.25) {
    s.addShape(pres.shapes.LINE, { x, y: Math.min(y1, y2), w: 0, h: Math.abs(y2 - y1), line: { color, width } });
  }
  /* A diagram node: title and a small grey line under it, optional WP pill. */
  function node(s, x, y, w, h, title, sub, { fill = WHITE, line = LINE, tag = null, titleSize = 12, subSize = 9.5 } = {}) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill }, line: { color: line, width: 1.25 }, rectRadius: 0.07 });
    s.addText(title, { x: x + 0.08, y: y + 0.08, w: w - 0.16, h: 0.34, fontSize: titleSize, color: INK, fontFace: FF, bold: true, align: "center", valign: "middle" });
    if (sub) s.addText(sub, { x: x + 0.08, y: y + 0.42, w: w - 0.16, h: h - 0.5, fontSize: subSize, color: GREY, fontFace: FF, align: "center", valign: "top", lineSpacingMultiple: 1.05 });
    if (tag) {
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: x + 0.1, y: y - 0.14, w: 0.52, h: 0.26, fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.05 });
      s.addText(tag, { x: x + 0.1, y: y - 0.14, w: 0.52, h: 0.26, fontSize: 9, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
    }
  }
  /* A tag card: icon, small teal headline, body in ink (30 Sept deck). */
  function tagCard(s, x, y, w, h, icon, tag, body, bodySize = 10.5) {
    cardBox(s, x, y, w, h);
    s.addImage({ data: icon, x: x + 0.22, y: y + 0.2, w: 0.26, h: 0.26 });
    s.addText(tag.toUpperCase(), { x: x + 0.56, y: y + 0.17, w: w - 0.76, h: 0.32, fontSize: 9, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
    s.addText(body, { x: x + 0.22, y: y + 0.56, w: w - 0.44, h: h - 0.68, fontSize: bodySize, color: INK, fontFace: FF, lineSpacingMultiple: 1.15, valign: "top" });
  }
  /* A big number over a label. */
  function bigStat(s, x, y, w, value, label, { size = 26, color = TEAL, labelSize = 10.5, labelH = 0.55 } = {}) {
    s.addText(value, { x, y, w, h: 0.55, fontSize: size, color, fontFace: FF, bold: true, valign: "bottom" });
    s.addText(label, { x, y: y + 0.57, w, h: labelH, fontSize: labelSize, color: GREY, fontFace: FF, lineSpacingMultiple: 1.1, valign: "top" });
  }
  /* A table with a height per row (30 Sept deck tableV). */
  function tableV(s, rows, { x = 0.55, y, w = 8.9, colFrac, rhs, fontSize = 10, emph = [], align }) {
    const colW = colFrac.map((f) => w * f);
    let ry = y;
    rows.forEach((r, ri) => {
      const header = ri === 0, zebra = !header && ri % 2 === 0;
      let cx = x;
      r.forEach((cell, ci) => {
        s.addShape(pres.shapes.RECTANGLE, { x: cx, y: ry, w: colW[ci], h: rhs[ri], fill: { color: header ? TEAL : zebra ? TEAL_TINT : WHITE }, line: { color: LINE, width: 1 } });
        const e = !header && emph.some(([er, ec]) => er === ri && ec === ci);
        s.addText(String(cell), {
          x: cx + 0.1, y: ry, w: colW[ci] - 0.2, h: rhs[ri], fontSize: header ? fontSize + 0.5 : fontSize,
          color: header ? WHITE : e ? TEAL_DK : INK, bold: header || e, fontFace: FF,
          align: align ? align[ci] : "left", valign: "middle", lineSpacingMultiple: 1.0,
        });
        cx += colW[ci];
      });
      ry += rhs[ri];
    });
    return ry;
  }

  /* ---- Derived numbers ------------------------------------------------------ */
  const W1 = D.wp1, W2 = D.wp2, F = D.wp3_fsm, W4 = D.wp4, PP = D.paraphraser, NFZ = D.nfz_flight;
  const G = D.wp3_bench.grant_load;
  const chk = G["near/_check"], tick = G["near/filter"];
  // The replay of real flight ticks (performance cores), when it is on file
  // and its own checks passed (build_progress_oct_data.profile_facts).
  const PC = (D.wp3_profile || {}).performance_cores, EC = (D.wp3_profile || {}).efficiency_cores;
  const PROF = PC && PC.quotable ? PC : null;
  const ECO = EC && EC.quotable ? EC : null;
  const suite = D.release.suite;
  const [tokLo, tokHi] = W2.tokens_exact_range || [null, null];
  const tokText = W2.tokens_exact_range ? `${tokLo}-${tokHi}` : `up to ${W2.tokens_table_max}`;
  const tokLabel = W2.tokens_exact_range ? "OpenVLA tokens" : "tokens (upper-bound counter)";
  const smoke = W4.profiles.smoke, nightly = W4.profiles.nightly;
  const banner = NFZ.banner_ticks || {};
  const hudImg = await photo(IMG.hud);
  const trackImg = await img(IMG.track);

  /* 01 · COVER ------------------------------------------------------------- */
  {
    page++;
    const s = track(pres.addSlide(), "Progress Report, October 2026");
    H.darkBase(s, "ITRI ICL subcontract · VLA-UAV Guardrail · NTUT AIoT Lab");
    s.addText("Progress Report\nOctober 2026", { x: 0.55, y: 1.15, w: 8.4, h: 1.9, fontSize: 40, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1 });
    s.addText("All four work packages advanced toward the grant's locked specification", { x: 0.55, y: 3.15, w: 8.6, h: 0.6, fontSize: 15, color: TEAL_TINT, fontFace: FF });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.15, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    // A draft says so on its first page, so it cannot be sent by mistake.
    s.addText([
      { text: `Status as of ${longDate(STATUS)}` },
      ...(DRAFT ? [{ text: "  ·  REVIEW DRAFT, not for distribution", options: { bold: true, color: WHITE } }] : []),
    ], { x: 0.55, y: 4.3, w: 8.6, h: 0.35, fontSize: 12, color: TEAL_TINT, fontFace: FF });
    s.addNotes(`This pack is for ITRI's monthly report and covers the work from ${dayMonth(PERIOD_FROM)} to ${longDate(STATUS)}. ` +
      "Each slide names, at the bottom, the file its numbers come from, so any figure can be traced and re-run.");
  }

  /* 02 · THE SYSTEM IN ONE LINE --------------------------------------------- */
  {
    const s = content("Semantic-spatial translation and safety-constrained VLA for ArduPilot", "The System in One Line");
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.55, y: 1.6, w: 8.9, h: 0.62, fill: { color: TEAL_TINT }, line: { color: TEAL_TINT }, rectRadius: 0.08 });
    // Labelled as the grant's design: PDF readers see no notes, and no flight
    // has yet flown a VLA with the compiled rules in its prompt.
    s.addText("The grant's design: a declarative flight policy is compiled into the VLA's prompt, and a Safety Shield checks every 10 Hz command against the same policy before ArduPilot flies it.", {
      x: 0.75, y: 1.6, w: 8.5, h: 0.62, fontSize: 12.5, color: TEAL_DEEP, fontFace: FF, bold: true, valign: "middle",
    });
    const y = 2.55, h = 0.92, gap = 0.24, w = (8.9 - 4 * gap) / 5;
    const nodes = [
      ["Policy DSL", "rules, signed bundle", "WP1"],
      ["Prefix Compiler", "rule summary in the prompt", "WP2"],
      ["VLA backend", "4-D action, 10 Hz", null],
      ["Safety Shield", "check, repair, escalate", "WP3"],
      ["ArduPilot", "inner loop, GeoFence", null],
    ];
    nodes.forEach(([t, sub, tag], i) => {
      const x = 0.55 + i * (w + gap);
      node(s, x, y + 0.04, w, h, t, sub, tag ? { fill: TEAL_TINT, line: TEAL, tag, titleSize: 11, subSize: 9 } : { titleSize: 11, subSize: 9 });
      if (i < nodes.length - 1) arrowR(s, x + w + 0.02, y + 0.04 + h / 2, x + w + gap - 0.02);
    });
    // The Shield reads the same policy: an elbow from Policy DSL to the Shield.
    const yt = y + 0.04 + h;
    const xa = 0.55 + w / 2, xb = 0.55 + 3 * (w + gap) + w / 2, yb = yt + 0.24;
    vline(s, xa, yt, yb, TEAL); hline(s, xa, yb, xb, TEAL); arrowD(s, xb, yb, yt + 0.02, TEAL);
    s.addText("the same rules, enforced", { x: xa + 1.6, y: yb - 0.01, w: 3.0, h: 0.26, fontSize: 9.5, color: TEAL_DK, fontFace: FF, italic: true, align: "center" });
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.55, y: 4.0, w: 8.9, h: 0.38, fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.07 });
    s.addText([
      { text: "WP4  ", options: { bold: true } },
      { text: "Stress testing of the whole chain: scenarios, seeds, paraphrased instructions, KPI report, replay" },
    ], { x: 0.7, y: 4.0, w: 8.6, h: 0.38, fontSize: 10, color: WHITE, fontFace: FF, valign: "middle" });
    s.addText([
      { text: "Acceptance KPIs:  ", options: { bold: true, color: INK } },
      { text: ACCEPTANCE_KPIS.join("  ·  "), options: { color: GREY } },
    ], { x: 0.55, y: 4.44, w: 8.9, h: 0.48, fontSize: 9.5, fontFace: FF, valign: "top" });
    source(s, "Source: grant pages 'Grant overview' and 'Architecture constraints' (ITRI ICL subcontract, 2026-02-01 to 2026-11-30).");
    H.badge(s, page);
    s.addNotes("This is the grant's architecture. The rules an operator writes become a signed policy (WP1). The Prefix Compiler turns them " +
      "into a short summary placed in front of the VLA's instruction (WP2). The Safety Shield checks every 4-D command against the same rules " +
      "ten times a second and repairs or stops it before ArduPilot flies it; the escalation chain on slide 7 adds Loiter, RTL and Land (WP3). " +
      "WP4 stress-tests the whole chain and produces the KPI report. " +
      "The five acceptance KPIs at the bottom are the grant's own list.");
  }

  /* 03 · DELIVERED THIS PERIOD ---------------------------------------------- */
  {
    const s = content(`${dayMonth(PERIOD_FROM)} - ${longDate(STATUS)}`, "Delivered This Period");
    const rows = [
      ["Work package", "Grant output", `Delivered by ${dayMonth(STATUS)}`],
      ["WP1 Policy DSL", "Signed policy bundle (hash + semver)",
        `Ed25519-signed bundles; ${W1.round_trip_passed}/${W1.policies} policies round-trip; ${W1.tamper_refused}/${W1.tamper_run} tampered bundles refused`],
      ["WP2 Prefix Compiler", "Constraint Summary Pack in the VLA prefix",
        `Typed ${W2.csp_fields_locked}-field CSP; ${W2.p0_covered}/${W2.p0_in_scope} P0 rules carried within a ${W2.budget_tokens}-token budget; OpenVLA prompt path`],
      ["WP3 Safety Shield", "Monitor, filter, projection; fail-safe escalation",
        `Rule check at the 50-rule load (desktop): ${ms(chk.legacy.median_ms)} to ${ms(chk.current.median_ms)}; ${F.states.join(" / ")} state machine`],
      ["WP4 Stress testing", "Scenario library, KPI rollups, replay, paraphraser",
        `${smoke.episodes}-episode smoke and ${nf(nightly.episodes)}-episode nightly profiles; ${PP.paraphrases} stored paraphrases; KPI report generator`],
      ["Demonstration", "ITRI request, 30 September", "Policy indicator on screen; city no-fly-zone flight"],
    ];
    const tb = tableV(s, rows, { y: 1.62, fontSize: 10, colFrac: [0.21, 0.27, 0.52], rhs: [0.34, 0.5, 0.5, 0.5, 0.5, 0.42], emph: [[1, 0], [2, 0], [3, 0], [4, 0]] });
    if (suite) {
      // The interpreter of the full run stays in progress_oct.json; the slide
      // names 3.11, the grant's locked version, only where the changelog does.
      const py311 = suite.guardrail_passes_on_311 ? "; the guardrail package's tests also pass on Python 3.11" : "";
      s.addText(`Test suite: ${nf(suite.passed)} of ${nf(suite.total)} tests pass over ${nf(suite.files)} test files${py311}.`, {
        x: 0.55, y: tb + 0.1, w: 8.9, h: 0.32, fontSize: 10.5, color: TEAL_DK, fontFace: FF, bold: true,
      });
    }
    source(s, `Source: CHANGELOG.md, 2026-10-03 to ${STATUS}; all figures from docs/data/progress_oct.json.`);
    H.badge(s, page);
    s.addNotes("One row per work package, each against the output the grant names for it. The following slides take them one at a time. " +
      "The last row answers ITRI's request of 30 September to see on screen when a rule acts.");
  }

  /* 04 · WP1 ------------------------------------------------------------------ */
  {
    const s = content("WP1 · Policy DSL, IR and signed bundle", "Policies With a Verifiable Identity");
    const steps = [
      ["Policy file (YAML)", "No-fly zones, envelopes, corridors, time windows"],
      ["Canonical IR and hash", "64-hex SHA-256 over the canonical policy"],
      ["Manifest", "Policy id, semantic version, issue time"],
      ["Ed25519 signature", "A bundle with a wrong hash or signature is refused"],
    ];
    const x0 = 0.55, bw = 4.5, bh = 0.62, dy = 0.78;
    steps.forEach(([t, b], i) => {
      const y = 1.62 + i * dy;
      cardBox(s, x0, y, bw, bh);
      s.addShape(pres.shapes.OVAL, { x: x0 + 0.16, y: y + 0.14, w: 0.34, h: 0.34, fill: { color: TEAL }, line: { color: TEAL } });
      s.addText(String(i + 1), { x: x0 + 0.16, y: y + 0.14, w: 0.34, h: 0.34, fontSize: 11, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(t, { x: x0 + 0.62, y: y + 0.04, w: bw - 0.75, h: 0.28, fontSize: 11.5, color: INK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(b, { x: x0 + 0.62, y: y + 0.31, w: bw - 0.75, h: 0.28, fontSize: 9.5, color: GREY, fontFace: FF, valign: "middle" });
      if (i < steps.length - 1) arrowD(s, x0 + 0.33, y + bh, y + dy - 0.01, TEAL);
    });
    const sx = 5.45, sw = 4.0;
    bigStat(s, sx, 1.55, sw, `${W1.round_trip_passed} / ${W1.policies}`, "policies round-trip: same hash, same IR, signature verified");
    bigStat(s, sx, 2.6, sw, `${W1.tamper_refused} / ${W1.tamper_run}`, `tampered bundles refused (a reader that checks nothing refuses ${W1.null_tamper_refused} / ${W1.null_tamper_run})`);
    // The broken-policy corpus is in newer wp1_roundtrip.json files only;
    // without it the third figure is the pure-Python verifier, not a zero.
    const neg = W1.negative_run != null;
    if (neg) {
      bigStat(s, sx, 3.65, sw, `${nf(W1.negative_refused)} / ${nf(W1.negative_run)}`, `deliberately broken policies refused (a reader that checks nothing: ${nf(W1.null_negative_refused)} / ${nf(W1.negative_run)})`);
    } else {
      bigStat(s, sx, 3.65, sw, "Pure Python", "signature check that runs where the cryptography library is absent (the 3.11 and SITL environments)", { size: 20 });
    }
    source(s, `Source: docs/data/wp1_roundtrip.json (${W1.command}); guardrail/bundle.py; policies/policy.lock.json.`);
    H.badge(s, page);
    s.addNotes("The grant asks for a signed policy bundle with a hash and a semantic version. Each policy now has one 64-character hash " +
      "over its canonical form, a manifest with its version and issue time, and an Ed25519 signature. All " + W1.policies + " policies in the " +
      "repository survive a write and read-back unchanged, and every deliberately tampered copy is refused. The null beside each figure: a " +
      "reader that verifies nothing also passes the round trip, so only the refusal counts tell a real verifier apart. " +
      (neg ? `The policy language also refuses each of the ${W1.negative_run} deliberately broken policies in the test corpus, such as a misspelled key, ` +
        "a self-crossing polygon or a duplicate rule id. " : "") +
      "Where the cryptography library is not installed, the signature is checked by a pure-Python verifier (RFC 8032), so the check can run in every environment. " +
      "The Shield node, the SITL demo and the city runner load bundles with --bundle.");
  }

  /* 05 · WP2 ------------------------------------------------------------------ */
  {
    const s = content("WP2 · Prefix Constraint Compiler", "A Typed Rule Summary for the VLA Prompt");
    const ex = W2.example;
    const lx = 0.55, lw = 5.1;
    s.addText(`Compiled for the VLA prompt: the city no-fly-zone policy`, {
      x: lx, y: 1.58, w: lw, h: 0.3, fontSize: 10.5, color: INK, fontFace: FF, bold: true,
    });
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: lx, y: 1.92, w: lw, h: 1.66, fill: { color: TEAL_TINT }, line: { color: TEAL_TINT2 }, rectRadius: 0.06 });
    s.addText(ex.prompt, { x: lx + 0.18, y: 1.98, w: lw - 0.36, h: 1.54, fontSize: 10.5, color: TEAL_DEEP, fontFace: FF, italic: true, valign: "middle", lineSpacingMultiple: 1.15 });
    const exTok = ex.tokens_exact ? `${ex.rules} rules, ${ex.tokens_exact} OpenVLA tokens. ` : `${ex.rules} rules. `;
    s.addText(exTok + "real_vla_demo.py --csp on places this text before OpenVLA's instruction, and logs the prompt, its token count and the CSP and policy hashes at every inference.", {
      x: lx, y: 3.68, w: lw, h: 0.75, fontSize: 10, color: GREY, fontFace: FF, valign: "top", lineSpacingMultiple: 1.1,
    });
    const sx = 6.0, sw = 3.45;
    // The null that can tell a compiler apart: the old by-type cut to the same
    // budget. While every policy fits whole it also carries every P0 rule, and
    // the label says so instead of quoting only the empty-CSP null.
    const p0Label = W2.kpi_discriminates === false
      ? `P0 rules carried over ${W2.policies} policies; each fits the budget whole, so no rule was cut`
      : `P0 rules carried, over ${W2.policies} policies (the old by-type cut carries ${nf(W2.baseline_p0_covered)} / ${nf(W2.p0_in_scope)})`;
    bigStat(s, sx, 1.5, sw, `${nf(W2.p0_covered)} / ${nf(W2.p0_in_scope)}`, p0Label, { size: 24, labelH: 0.5 });
    bigStat(s, sx, 2.55, sw, tokText, `${tokLabel} for every rule of a policy; budget ${W2.budget_tokens}`, { size: 24, labelH: 0.5 });
    bigStat(s, sx, 3.6, sw, String(W2.csp_fields_locked), `locked CSP fields; P1 / P2 rules ranked by risk (weights ${W2.risk_weights.join(" / ")})`, { size: 24, labelH: 0.5 });
    source(s, `Source: ${W2.command.replace(" --policies policies", "")}; guardrail/csp.py; demo/real_vla_demo.py.`);
    H.badge(s, page);
    s.addNotes("The Constraint Summary Pack is now a typed object with the grant's " + W2.csp_fields_locked + " locked fields. P0 rules are always kept; " +
      "P1 and P2 rules are ranked by the grant's risk score inside a token budget, and every rule carries the reason it was kept. Time-windowed " +
      "rules carry their window, for example 'in force Mon-Fri 07:30-17:30'. The left box is the compiler's actual output for the city " +
      "no-fly-zone policy. " +
      (W2.kpi_discriminates === false
        ? "Every policy in the repository fits whole in the " + W2.budget_tokens + "-token budget, so nothing had to be cut, and the P0 " +
          "figure would be the same for a simpler cut; the ranking is what matters once a mission's rules exceed the budget. "
        : `A simpler cut by rule type to the same budget carries ${W2.baseline_p0_covered} of the ${W2.p0_in_scope} P0 rules. `) +
      "The OpenVLA runner can now put this text in front of its instruction; flights with it on are part of the plan to November.");
  }

  /* 06 · WP3 SHIELD TIMING ----------------------------------------------------- */
  {
    // Not "Well Inside the Budget" (title until 7 Oct): a replay of real
    // flight ticks at the same load measured the check's p99 at 3.4 ms on the
    // desktop's performance cores (deploy/evidence/dev/shield_tick_profile.json),
    // a margin the synthetic benchmark's 1.1 ms overstates. The title states
    // the speed-up, which both measurements support, and only when the bench
    // shows it in both columns.
    const fold = Math.min(chk.legacy.median_ms / chk.current.median_ms, tick.legacy.median_ms / tick.current.median_ms);
    const title = fold >= 100 ? "Rule Checks Over 100 Times Faster" : fold >= 10 ? "Rule Checks Over 10 Times Faster" : "A Faster Rule Check";
    const s = content("WP3 · Safety Shield at the grant's 50-rule load", title);
    const cw = 4.35, ch = 1.52, y0 = 1.58;
    const blocks = [
      ["Rule check near a no-fly zone", chk, G.budgets_ms._check, "check budget"],
      ["Whole 10 Hz tick near a zone", tick, G.budgets_ms.filter, "tick budget"],
    ];
    blocks.forEach(([t, st, budget, bl], i) => {
      const x = 0.55 + i * (cw + 0.2);
      cardBox(s, x, y0, cw, ch);
      s.addText(t, { x: x + 0.24, y: y0 + 0.14, w: cw - 0.48, h: 0.32, fontSize: 11.5, color: INK, fontFace: FF, bold: true });
      s.addText(arrow(ms(st.legacy.median_ms), ms(st.current.median_ms)), { x: x + 0.24, y: y0 + 0.5, w: cw - 0.48, h: 0.55, fontSize: 26, color: TEAL, fontFace: FF, bold: true, valign: "middle" });
      s.addText(`median on benchmark samples, old loop and new; p99 now ${ms(st.current.p99_ms)}, ${bl} ${budget} ms`, {
        x: x + 0.24, y: y0 + 1.08, w: cw - 0.48, h: 0.42, fontSize: 10, color: GREY, fontFace: FF, valign: "top",
      });
    });
    const fy = 3.26, fw = (8.9 - 2 * 0.2) / 3, fh = 1.3;
    tagCard(s, 0.55, fy, fw, fh, ic.layers, "The grant's load", `${G.rules} rules, ${G.fences} no-fly polygons, ${G.poses_per_check} future poses per check`, 9.5);
    tagCard(s, 0.55 + fw + 0.2, fy, fw, fh, ic.ok, "Same decisions", "Identical to the old per-fence loop on every sample both ran", 9.5);
    // Real flight ticks at the same load, when the profile is on file and
    // trusts itself; otherwise the card says how the speed-up was made.
    if (PROF) {
      tagCard(s, 0.55 + 2 * (fw + 0.2), fy, fw, fh, ic.speed, "Real flight ticks",
        `${nf(PROF.check.n)} replayed at the same load: check p99 ${ms(PROF.check.p99_ms)}, max ${ms(PROF.check.max_ms)} (desktop performance cores)`, 9.5);
    } else {
      tagCard(s, 0.55 + 2 * (fw + 0.2), fy, fw, fh, ic.speed, "How", "Fence rings built once per policy; a spatial index picks the fences in reach", 9.5);
    }
    s.addText(`Desktop CPU. The benchmark${PROF ? " and the replay" : ""} will be repeated on the Jetson Orin in the hil campaign.`, {
      x: 0.55, y: 4.66, w: 8.9, h: 0.28, fontSize: 10.5, color: TEAL_DK, fontFace: FF, bold: true,
    });
    source(s, PROF ? `Source: ${G.command.replace(/^python /, "").replace(/ --legacy-cap \d+/, "")}; ${PROF.source}.`
      : `Source: ${G.command} (desktop); guardrail/ir.py.`);
    H.badge(s, page);
    const d3 = D.wp3_bench.default_3s;
    // Runs of the same code set aside because the machine was busy (pick_bench).
    const loaded = (G.load_check || []).filter((r) => /under load/.test(r.why));
    const lc = loaded[loaded.length - 1];
    const second = lc ? `A run of the same code with other jobs on the machine measured ${ms(lc.near_check_ms.legacy)} to ` +
      `${ms(lc.near_check_ms.current)}, slower in both columns; the slide quotes the run on the quieter machine. ` : "";
    // "N/N in each scenario" only when every scenario compared the same count.
    const agreeSet = [...new Set(Object.values(G.agreement))];
    const agreeText = agreeSet.length === 1 ? `${agreeSet[0]} in each scenario`
      : Object.entries(G.agreement).map(([k, v]) => `${k} ${v}`).join(", ");
    const replay = PROF ? `The benchmark's samples are synthetic. A replay of ${nf(PROF.check.n)} real flight ticks at the same load, with no-fly zones added ` +
      `along each flight path up to 50 rules and the process pinned to the desktop's performance cores, measured the check at ${ms(PROF.check.median_ms)} median ` +
      `and ${ms(PROF.check.p99_ms)} p99, at most ${ms(PROF.check.max_ms)}, against the ${PROF.check.budget_ms} ms budget, and the whole tick at ${ms(PROF.filter.p99_ms)} p99. ` : "";
    s.addNotes(`The grant sets the budget at about 50 active rules and 50 future-pose checks per tick: 5 ms for the check, inside a 100 ms tick. ` +
      `At exactly that load, a rule check near a no-fly zone went from ${ms(chk.legacy.median_ms)} to ${ms(chk.current.median_ms)} median, and a whole tick ` +
      `from ${ms(tick.legacy.median_ms)} to ${ms(tick.current.median_ms)}. The 'before' column is the old per-fence loop, timed on ${chk.legacy.n} samples; the new ` +
      `Shield reaches the same decision on every sample both ran (${agreeText}). The speed-up comes from building each fence's ring once per policy ` +
      `and letting a spatial index pick the fences in reach. With the shorter 3 s lookahead the check takes ` +
      `${ms(d3["near/_check"].current.median_ms)}. The Shield also keeps its last 50 ticks, the sliding-window buffer of the grant's Shield design. ` +
      second + replay + "These are desktop numbers; both measurements are scripts, so they will be re-run unchanged on the Orin, where the grant's hil topology runs the Shield.");
  }

  /* 07 · WP3 ESCALATION FSM ----------------------------------------------------- */
  {
    const s = content("WP3 · the grant's fail-safe chain", "Escalation State Machine");
    const states = F.states;
    const why = ["rules satisfied", "a violation", `${F.n_violations} violations in ${F.window_s} s, or a repair larger than theta`, "violations persist, or loiter timeout", "RTL cannot complete"];
    const y = 1.98, h = 0.56, gap = 0.36, w = (8.9 - 4 * gap) / 5;
    states.forEach((st, i) => {
      const x = 0.55 + i * (w + gap);
      const fill = i === 0 ? TEAL_TINT : i >= 3 ? TEAL_DK : TEAL;
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill }, line: { color: i === 0 ? TEAL : fill, width: 1.25 }, rectRadius: 0.08 });
      s.addText(st, { x, y, w, h, fontSize: 15, color: i === 0 ? TEAL_DEEP : WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(why[i], { x: x - 0.1, y: y + h + 0.06, w: w + 0.2, h: 0.62, fontSize: 9.5, color: GREY, fontFace: FF, align: "center", valign: "top", lineSpacingMultiple: 1.05 });
      if (i < states.length - 1) arrowR(s, x + w + 0.04, y + h / 2, x + w + gap - 0.04);
    });
    // Back to Normal when the violation clears for T_recover (Brake and Loiter).
    const xn = 0.55 + w / 2, xl = 0.55 + 2 * (w + gap) + w / 2, yr = y - 0.2;
    const xbk = 0.55 + (w + gap) + w / 2;
    vline(s, xl, y, yr, TEAL); vline(s, xbk, y, yr, TEAL); hline(s, xn, yr, xl, TEAL); arrowD(s, xn, yr, y - 0.01, TEAL);
    s.addText(`clear for ${F.t_recover_s} s: back to Normal`, { x: xl + 0.1, y: yr - 0.13, w: 2.8, h: 0.25, fontSize: 9.5, color: TEAL_DK, fontFace: FF, italic: true, align: "left" });
    const cy = 3.22, cw = (8.9 - 2 * 0.2) / 3, chh = 1.34;
    tagCard(s, 0.55, cy, cw, chh, ic.cogs, "Grant defaults",
      `N = ${F.n_violations} in T = ${F.window_s} s; theta ${F.theta_lateral_m} m lateral, ${F.theta_vertical_m} m vertical; T_recover ${F.t_recover_s} s. Set per mission in YAML.`, 9.5);
    tagCard(s, 0.55 + cw + 0.2, cy, cw, chh, ic.list, "Driven by the policy",
      "Each rule's violation action, type and priority pick the step; RTL and Land rules go there directly.", 9.5);
    tagCard(s, 0.55 + 2 * (cw + 0.2), cy, cw, chh, ic.clip, "Scored with nulls",
      `Trigger correctness beside always- and never-trigger baselines; target at least ${Math.round(F.failsafe_target * 100)} %.`, 9.5);
    s.addText("Next: in the ROS 2 Shield node, with ArduPilot's GeoFence as the final backstop.", {
      x: 0.55, y: 4.66, w: 8.9, h: 0.28, fontSize: 10.5, color: TEAL_DK, fontFace: FF, bold: true,
    });
    source(s, "Source: guardrail/fsm.py (FSMConfig, FSMState); grant page 'Safety Shield', escalation FSM; docs/DESIGN-escalation-fsm.md.");
    H.badge(s, page);
    s.addNotes("This is the grant's escalation chain, with its own default thresholds. Small, trusted repairs keep the drone in guided flight; " +
      "repeated violations or a repair larger than theta step up to Loiter, then RTL, then Land, and the drone returns to Normal once the " +
      "violation has been clear for two seconds. Rules whose action is RTL or Land go straight there. It is built as a pure state machine, " +
      "so every transition can be unit-tested without a simulator. The scorer reports fail-safe trigger correctness beside the two trivial " +
      "baselines, so a number that does not beat both is visible as such.");
  }

  /* 08 · WP4 STRESS PROFILES ---------------------------------------------------- */
  {
    const s = content("WP4 · dynamic-scenario stress testing", "Stress Profiles at the Grant's Scale");
    // 1.56, not 1.5: at 1.5 the 30 pt numbers touched the heading rule (7 Oct render).
    const y = 1.56, sw = 2.85;
    const gs = (g) => g.replace(" scenarios", "").replace(" seeds", "").replace(" seed each", "").replace(" seed", "");
    // "per build" is the grant's cadence for this profile, not a CI fact: the
    // CI workflow has not run on GitHub yet (CHANGELOG 2026-10-06).
    bigStat(s, 0.55, y, sw, nf(smoke.episodes), `smoke episodes: ${nf(smoke.cells)} x ${smoke.seeds.length} seed, the grant's per-build set (${gs(smoke.grant_scope)})`, { size: 30, labelSize: 10, labelH: 0.48 });
    bigStat(s, 0.55 + sw + 0.18, y, sw, nf(nightly.episodes), `nightly episodes: ${nf(nightly.cells)} x ${nightly.seeds.length} seeds, the grant's nightly set (${gs(nightly.grant_scope)})`, { size: 30, labelSize: 10, labelH: 0.48 });
    bigStat(s, 0.55 + 2 * (sw + 0.18), y, sw, String(W4.n_templates), "scenario templates, from static no-fly zones to wind and GPS loss", { size: 30, labelSize: 10, labelH: 0.48 });
    const cy = 2.66, cw = (8.9 - 2 * 0.2) / 3, chh = 1.9;
    tagCard(s, 0.55, cy, cw, chh, ic.random, "Stressors",
      "The Shield test matrix: high speed near an edge, sudden no-fly zone, time-window switch, three violations at once, GPS noise and latency. Plus wind, gusts, start jitter, GPS dropout.", 9.5);
    tagCard(s, 0.55 + cw + 0.2, cy, cw, chh, ic.id, "Traceable episodes",
      `${W4.manifest_fields.length}-field manifest: code revision, VLA model hash, policy hash, seed, sim speed-up, topology. Expected outcome and fail-safe labelled per scenario.`, 9.5);
    tagCard(s, 0.55 + 2 * (cw + 0.2), cy, cw, chh, ic.clip, "KPI report",
      "One row per scenario family and Shield arm: acceptance KPIs, repair success and false triggers, each beside its null. 'Not measurable' is never printed as zero.", 9.5);
    s.addText("Run headless today for regression; the hil KPI campaign runs the same profiles.", {
      x: 0.55, y: 4.66, w: 8.9, h: 0.28, fontSize: 10.5, color: TEAL_DK, fontFace: FF, bold: true,
    });
    source(s, "Source: experiments/profiles/{smoke,nightly}.yaml expanded by guardrail/scenario_spec.py; tools/kpi_report.py.");
    H.badge(s, page);
    s.addNotes(`The grant's test schedule asks for about 50 scenarios on one seed per build, and about 200 scenarios on three seeds nightly. ` +
      `Our two profiles expand to ${smoke.episodes} and ${nf(nightly.episodes)} episodes over ${W4.n_templates} templates, and they include all five rows of the Shield test matrix the grant asks every release to run. ` +
      "Every episode records the grant's six manifest fields, so each number in the final KPI report can be traced to the code, model, " +
      "policy and seed that produced it. The KPI report generator prints 'not measurable' where a log cannot answer, rather than a zero.");
  }

  /* 09 · WP4 PARAPHRASER -------------------------------------------------------- */
  {
    const s = content("WP4 · paraphraser for per-paraphrase robustness", "Paraphrased Instructions");
    const ex = PP.example;
    const lx = 0.55, lw = 4.75;
    s.addText([{ text: "Instruction:  ", options: { color: GREY } }, { text: `"${ex.source}"`, options: { color: INK, bold: true } }], {
      x: lx, y: 1.6, w: lw, h: 0.32, fontSize: 11.5, fontFace: FF,
    });
    const picks = [2, 4, 5, 7].map((i) => ex.paraphrases[i]).filter(Boolean);
    picks.forEach((p, i) => {
      const y = 2.0 + i * 0.56;
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: lx, y, w: lw, h: 0.46, fill: { color: TEAL_TINT }, line: { color: TEAL_TINT2 }, rectRadius: 0.06 });
      s.addText(`"${p}"`, { x: lx + 0.15, y, w: lw - 0.3, h: 0.46, fontSize: 10.5, color: TEAL_DEEP, fontFace: FF, italic: true, valign: "middle" });
    });
    s.addText(`${picks.length} of the ${PP.per_source} stored paraphrases of this instruction.`, { x: lx, y: 2.0 + picks.length * 0.56, w: lw, h: 0.28, fontSize: 9.5, color: GREY, fontFace: FF });
    const sx = 5.65, sw = 3.8;
    const k = PP.source_kinds;
    const o = { size: 26, labelSize: 10, labelH: 0.48 };
    bigStat(s, sx, 1.45, sw, String(PP.paraphrases), `stored paraphrases: ${PP.per_source} each of ${k.task} task instructions and ${k.csp_nl} rule summaries`, o);
    // "Generated variants": the slot checker's record on held-out paraphrases
    // is not perfect (CHANGELOG 2026-10-06), so the label names the test set.
    bigStat(s, sx, 2.55, sw, `${nf(PP.mutants_refused)} / ${nf(PP.mutants)}`, `generated variants with a changed slot (${PP.mutation_classes} kinds) refused; a copy-only check refuses ${PP.null_copy_only_refused}`, o);
    bigStat(s, sx, 3.65, sw, "Same set every run", "generated once by a large language model, prompt recorded; seeded template fallback", { ...o, size: 18 });
    source(s, `Source: ${PP.source}; guardrail/paraphraser.py.`);
    H.badge(s, page);
    s.addNotes("Per-paraphrase robustness is one of WP4's KPIs: the same mission, worded differently, must give the same safe behaviour. " +
      `We store ${PP.paraphrases} paraphrases, ${PP.per_source} for each of ${PP.sources} instructions and rule summaries, generated once with the prompt recorded so ` +
      "every run reads exactly the same set. A slot checker is built to refuse a paraphrase that changes a number, a unit, a zone or a direction; " +
      `on ${nf(PP.mutants)} generated variants with one slot changed, of ${PP.mutation_classes} kinds, it refused every one, where a check that only rejects exact copies refuses ${PP.null_copy_only_refused === 0 ? "none" : nf(PP.null_copy_only_refused)}.`);
  }

  /* 10 · POLICY INDICATOR ------------------------------------------------------- */
  {
    const s = content("Requested by ITRI, 30 September", "The Policy on Screen");
    const fw = 5.75, fh = fw / hudImg.aspect, fx = 0.55, fy = 1.58;
    const f = H.figure(s, hudImg, fx, fy, fw, fh);
    HUD_CALLOUTS.forEach(([u, v], i) => {
      const d = 0.3, cx = f.x + u * f.w - d / 2, cy = f.y + v * f.h - d / 2;
      s.addShape(pres.shapes.OVAL, { x: cx, y: cy, w: d, h: d, fill: { color: TEAL }, line: { color: WHITE, width: 1.5 } });
      s.addText(String(i + 1), { x: cx, y: cy, w: d, h: d, fontSize: 11, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
    });
    const rx = 6.55, rw = 2.9;
    const items = [
      ["Banner", "the most important rule event of the last 1.5 s"],
      ["Rule panel", "every rule of the loaded policy: limit, live value, status"],
      ["Map", "zones, the drone's track and the target estimate"],
    ];
    items.forEach(([t, b], i) => {
      const y = 1.56 + i * 0.7;
      s.addShape(pres.shapes.OVAL, { x: rx, y: y + 0.02, w: 0.3, h: 0.3, fill: { color: TEAL }, line: { color: TEAL } });
      s.addText(String(i + 1), { x: rx, y: y + 0.02, w: 0.3, h: 0.3, fontSize: 11, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(t, { x: rx + 0.4, y, w: rw - 0.4, h: 0.32, fontSize: 12, color: INK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(b, { x: rx + 0.4, y: y + 0.3, w: rw - 0.4, h: 0.4, fontSize: 9.5, color: GREY, fontFace: FF, valign: "top" });
    });
    s.addText("Status words", { x: rx, y: 3.66, w: rw, h: 0.26, fontSize: 10, color: TEAL, fontFace: FF, bold: true });
    s.addText([
      { text: HUD_STATES.join(" · "), options: { color: INK, bold: true, breakLine: true } },
      { text: "AVOID: the pilot routes round a rule", options: { color: GREY, breakLine: true } },
      { text: "ACTING: the Shield repairs a command", options: { color: GREY } },
    ], { x: rx, y: 3.9, w: rw, h: 0.95, fontSize: 9, fontFace: FF, valign: "top", lineSpacingMultiple: 1.05 });
    source(s, `Source: demo/policy_hud.py; frame of ${NFZ.tag} at t = ${HUD_T_S} s (docs/img/policy_hud_nfz2.jpg).`);
    H.badge(s, page);
    s.addNotes("ITRI asked on 30 September to see on screen when a guardrail rule acts. The demo now draws three things over the camera view: " +
      "a banner with the most important rule event of the last second and a half, a panel with every rule of the loaded policy and its live " +
      "value, and a north-up map with the zones. The status words separate the pilot steering round a rule (AVOID) from the Shield " +
      "correcting a command (ACTING), so the viewer can see which layer acted.");
  }

  /* 11 · CITY NO-FLY-ZONE FLIGHT -------------------------------------------------- */
  {
    const s = content(`${NFZ.sim} · 3 October 2026`, "City No-Fly-Zone Flight");
    // Layout (7 Oct render): with five items the strip wrapped onto the
    // description; the altitude band now sits in the description, as the
    // measured band, and the chart box is 0.17 in shorter.
    const f = H.figure(s, trackImg, 0.55, 1.5, 8.9, 2.45);
    const y = f.y + f.h + 0.07;
    const strip = [
      ["zone entered", `${NFZ.zone_ticks_inside} ticks`],
      ["closest", `${m1(NFZ.zone_min_distance_m)} m`],
      ["car within 30 m", `${pct1(NFZ.within_30m)}`, ` (hover ${pct1(NFZ.within_30m_null_hover)})`],
      ["estimate on car", pctN(NFZ.estimate_on_car), ` (fixed ${pct1(NFZ.estimate_null_fixed_start)})`],
    ];
    const runs = [];
    strip.forEach(([l, v, nul], i) => {
      runs.push({ text: l + " ", options: { color: GREY, fontSize: 9.5 } });
      runs.push({ text: v, options: { color: TEAL_DK, bold: true, fontSize: 11.5 } });
      if (nul) runs.push({ text: nul, options: { color: GREY, fontSize: 9.5 } });
      if (i < strip.length - 1) runs.push({ text: "  ·  ", options: { color: GREY, fontSize: 9.5 } });
    });
    s.addText(runs, { x: 0.55, y, w: 8.9, h: 0.3, fontFace: FF, valign: "middle" });
    // 10 m is the cruise ALTITUDE (params.cruise_alt); the horizontal distance
    // to the car averaged sep_mean_m. "Following at 10 m" read as a distance.
    s.addText(`${Math.round(NFZ.duration_s)} s following the red car at ${m1(NFZ.alt_min_m)}-${m1(NFZ.alt_max_m)} m altitude, ${m1(NFZ.sep_mean_m)} m from it on average. The follow controller routed beside the zone (banner: AVOID) while the Safety Shield checked every command. ` +
      `Nulls: hover, a drone left at its start; fixed, an estimate left where the car started. Video: ${NFZ_VIDEO}.`, {
      x: 0.55, y: y + 0.33, w: 8.9, h: 0.6, fontSize: 9.5, color: GREY, fontFace: FF, valign: "top", lineSpacingMultiple: 1.05,
    });
    source(s, `Source: demo/out/${NFZ.tag}/metrics.json and flight_log.jsonl; ${NFZ.policy_id} policy.`);
    H.badge(s, page);
    s.addNotes(`The policy for this flight adds a no-fly zone over the frontage of the car's first street (${NFZ.policy_id}, ${NFZ.policy_rules} rules). ` +
      `Over ${Math.round(NFZ.duration_s)} seconds the drone never entered the zone, came no closer than ${m1(NFZ.zone_min_distance_m)} m to it, and kept the red car within ` +
      `30 m ${pct1(NFZ.within_30m)} of the time, against ${pct1(NFZ.within_30m_null_hover)} for a drone that simply hovered at the start. It flew at ` +
      `${m1(NFZ.cruise_alt_m)} m cruise altitude, between ${m1(NFZ.alt_min_m)} and ${m1(NFZ.alt_max_m)} m, inside the ${NFZ.alt_band_m[0]}-${NFZ.alt_band_m[1]} m band, ` +
      `and ${m1(NFZ.sep_mean_m)} m from the car on average. The pilot here is our follow controller; it re-aimed ` +
      `beside the zone on ${NFZ.fence_aim_ticks} ticks, and the banner shows such steering, round the zone or round a building, as AVOID; the Safety Shield checked every command. 'Estimate on car' is the share ` +
      `of ticks with a target estimate on which that estimate was within ${NFZ.estimate_near_m} m of the car's true position; an estimate that stayed where the car ` +
      `started scores ${pct1(NFZ.estimate_null_fixed_start)}. The full video is ${NFZ_VIDEO}.`);
  }

  /* 12 · TOPOLOGIES -------------------------------------------------------------- */
  {
    const s = content("Plan · Jetson Orin now, the drone next", "One Code Base, Three Topologies");
    const cw = 2.62, gap = 0.52, y0 = 1.58, ch = 2.4;
    const icons = [ic.server, ic.chip, ic.plane];
    const chips = ["In use", "Orin in bring-up now", "When the airframe is ready"];
    TOPOLOGIES.forEach(([name, where, what, purpose], i) => {
      const x = 0.55 + i * (cw + gap);
      const hl = i === 1;
      cardBox(s, x, y0, cw, ch, hl ? TEAL_TINT : WHITE, hl ? TEAL : LINE);
      s.addImage({ data: icons[i], x: x + 0.22, y: y0 + 0.22, w: 0.34, h: 0.34 });
      s.addText(name, { x: x + 0.66, y: y0 + 0.17, w: cw - 0.8, h: 0.42, fontSize: 18, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(where, { x: x + 0.22, y: y0 + 0.66, w: cw - 0.44, h: 0.3, fontSize: 11.5, color: INK, fontFace: FF, bold: true });
      s.addText(what, { x: x + 0.22, y: y0 + 0.98, w: cw - 0.44, h: 0.95, fontSize: 9.5, color: GREY, fontFace: FF, valign: "top", lineSpacingMultiple: 1.05 });
      s.addText(purpose, { x: x + 0.22, y: y0 + 2.0, w: cw - 0.44, h: 0.28, fontSize: 9.5, color: INK, fontFace: FF, italic: true });
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: y0 + ch + 0.1, w: cw, h: 0.32, fill: { color: hl ? TEAL : TEAL_TINT }, line: { color: hl ? TEAL : TEAL_TINT }, rectRadius: 0.06 });
      s.addText(chips[i], { x, y: y0 + ch + 0.1, w: cw, h: 0.32, fontSize: 10, color: hl ? WHITE : TEAL_DK, fontFace: FF, bold: true, align: "center", valign: "middle" });
      if (i < 2) arrowR(s, x + cw + 0.08, y0 + ch / 2, x + cw + gap - 0.08, TEAL);
    });
    s.addText("Design rule: the topology is configuration, not code; every run records its topology in its manifest.", {
      x: 0.55, y: 4.52, w: 8.9, h: 0.38, fontSize: 10.5, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle",
    });
    source(s, "Source: grant page 'Architecture constraints', three deployment topologies.");
    H.badge(s, page);
    s.addNotes("The grant requires one code base to run in three topologies. Until now everything ran on one desktop, which the grant calls dev. " +
      "A Jetson Orin is now being set up, so the KPI runs will use the grant's hil topology: the desktop keeps Project AirSim, ArduPilot SITL, " +
      "MAVROS 2 and Mission Planner, and the Orin runs the VLA and the Safety Shield over the network. Project AirSim stays on the desktop in " +
      "every topology, because Unreal Engine 5 needs a discrete GPU. When the " +
      "airframe is ready, the same Orin moves onto the drone beside the real flight controller; the goal is that only addresses and the " +
      "autopilot link change, not the guardrail code.");
  }

  /* 13 · PROJECT AIRSIM + ARDUPILOT + MISSION PLANNER --------------------------- */
  {
    const s = content("Plan · perception-rail integration, October-November", "Project AirSim, ArduPilot and Mission Planner");
    const bw = 2.3, bh = 0.92, r1 = 1.62, r2 = 3.02;
    const xA = 0.55, xB = 3.85, xC = 7.15;
    node(s, xA, r1, bw, bh, "Project AirSim", "desktop, UE5: CityLife city, cameras, physics", { fill: TEAL_TINT, line: TEAL });
    node(s, xB, r1, bw, bh, "ArduPilot SITL", "ArduCopter, airsim-copter frame", {});
    node(s, xC, r1, bw, bh, "mavlink-router", "one MAVLink link, fanned out", {});
    node(s, xA, r2, bw, bh, "VLA + Safety Shield", "Jetson Orin (hil, planned)", { fill: TEAL_TINT, line: TEAL });
    node(s, xB, r2, bw, bh, "MAVROS 2", "safe setpoints to ArduPilot", {});
    node(s, xC, r2, bw, bh, "Mission Planner", "ground station: map, modes, missions", {});
    const lbl = (t, x, y, w) => s.addText(t, { x, y, w, h: 0.24, fontSize: 9, color: TEAL_DK, fontFace: FF, italic: true, align: "center" });
    // Row 1: Project AirSim <-> SITL <-> router (two-way links).
    const two = (x1, y, x2) => s.addShape(pres.shapes.LINE, { x: x1, y, w: x2 - x1, h: 0, line: { color: CHARCOAL, width: 1.25, beginArrowType: "triangle", endArrowType: "triangle" } });
    two(xA + bw + 0.04, r1 + bh / 2, xB - 0.04);
    s.addText([{ text: PAS_CONTROLLER, options: { breakLine: true } }, { text: `UDP ${PAS_PORTS.replace(/ /g, "")}` }], {
      x: xA + bw, y: r1 + bh / 2 - 0.44, w: xB - xA - bw, h: 0.38, fontSize: 8.5, color: TEAL_DK, fontFace: FF, italic: true, align: "center", valign: "bottom", margin: 0,
    });
    two(xB + bw + 0.04, r1 + bh / 2, xC - 0.04); lbl("MAVLink", xB + bw, r1 + bh / 2 - 0.28, xC - xB - bw);
    // Camera frames down to the Orin; setpoints Orin -> MAVROS 2; MAVROS 2 and Mission Planner up to the router.
    arrowD(s, xA + bw / 2, r1 + bh + 0.02, r2 - 0.02, TEAL); lbl("camera frames", xA + bw / 2 + 0.05, r1 + bh + 0.1, 1.3);
    arrowR(s, xA + bw + 0.04, r2 + bh / 2, xB - 0.04, TEAL); lbl("4-D action, 10 Hz", xA + bw, r2 + bh / 2 - 0.28, xB - xA - bw);
    const xm = xB + bw / 2, xr = xC + bw / 2;
    s.addShape(pres.shapes.LINE, { x: xm, y: r1 + bh + 0.02, w: 0, h: r2 - r1 - bh - 0.04, line: { color: CHARCOAL, width: 1.25, beginArrowType: "triangle", endArrowType: "triangle" } });
    s.addShape(pres.shapes.LINE, { x: xr, y: r1 + bh + 0.02, w: 0, h: r2 - r1 - bh - 0.04, line: { color: CHARCOAL, width: 1.25, beginArrowType: "triangle", endArrowType: "triangle" } });
    lbl("via the router", xm + 0.05, r1 + bh + 0.1, 1.3); lbl("in parallel", xr + 0.05, r1 + bh + 0.1, 1.1);
    // The answer to "can Project AirSim connect to Mission Planner?"
    const ay = 4.12;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.55, y: ay, w: 8.9, h: 0.78, fill: { color: TEAL_TINT }, line: { color: TEAL_TINT }, rectRadius: 0.08 });
    s.addText([
      { text: "Mission Planner can connect through ArduPilot. ", options: { bold: true, color: TEAL_DEEP } },
      { text: `Project AirSim's ${PAS_CONTROLLER} controller lets ArduPilot SITL fly on its physics (${SITL_CMD}), and Mission Planner joins over MAVLink beside MAVROS 2, as the grant specifies. `, options: { color: INK } },
      { text: (D.g1 && D.g1.passed)
          ? `Gate G1 closed-loop check passed on ${D.g1.date}: ${D.g1.runs_ok}/${D.g1.runs_expected} square missions flown by ArduPilot, EKF error p95 ${D.g1.ekf_p95_m[0].toFixed(2)}-${D.g1.ekf_p95_m[1].toFixed(2)} m.`
          : "Being connected now.", options: { bold: true, color: TEAL_DEEP } },
    ], { x: 0.72, y: ay, w: 8.56, h: 0.78, fontSize: 10, fontFace: FF, valign: "middle", lineSpacingMultiple: 1.08 });
    source(s, "Source: Project AirSim (IAMAI fork) example_user_scripts/ardupilot/; grant page 'Architecture constraints'.");
    H.badge(s, page);
    s.addNotes("This is the integration planned for October to early November; work on it started on 6 October. It answers the question whether Project AirSim can work with Mission Planner. " +
      "Project AirSim ships an ArduPilot controller: the simulator exchanges sensor and motor data with ArduPilot SITL over UDP, and SITL is started with " +
      "the airsim-copter frame. Mission Planner never talks to the simulator directly; it talks MAVLink to ArduPilot, exactly as it would to a " +
      "real drone, and Project AirSim's own ArduPilot example notes that Mission Planner can control the drone. mavlink-router gives Mission " +
      "Planner its own link in parallel to MAVROS 2, which is the ground-station layout the grant locks. In the hil topology the camera frames " +
      "will go to the VLA on the Orin, and its commands will pass through the Safety Shield and MAVROS 2 to ArduPilot.");
  }

  /* 14 · TIMELINE ------------------------------------------------------------------ */
  {
    const s = content("Plan to the final delivery", "Timeline to 30 November");
    const day = (iso) => Date.parse(iso + "T00:00:00Z") / 86400000;
    const d0 = day(PLAN_FROM), d1 = day(PLAN_TO) + 1;
    const lx = 0.55, lw = 3.35, gx = 4.0, gw = 5.3, top = 1.86, rowH = 0.3;
    const X = (iso) => gx + (Math.max(day(iso), d0) - d0) / (d1 - d0) * gw;
    // Week ticks every Monday.
    for (let d = d0; d <= d1 - 1; d += 7) {
      const iso = new Date(d * 86400000).toISOString().slice(0, 10);
      const x = X(iso);
      vline(s, x, top - 0.08, top + PLAN.length * rowH, LINE, 0.75);
      const dt = new Date(d * 86400000);
      const lab = `${dt.getUTCDate()} ${dt.toLocaleString("en-US", { month: "short", timeZone: "UTC" })}`;
      s.addText(lab, { x: x - 0.4, y: top - 0.34, w: 0.8, h: 0.24, fontSize: 8.5, color: GREY, fontFace: FF, align: "center" });
    }
    PLAN.forEach(([label, a, b, state], i) => {
      const y = top + i * rowH;
      if (i % 2 === 0) s.addShape(pres.shapes.RECTANGLE, { x: lx, y, w: gx + gw + 0.15 - lx, h: rowH, fill: { color: "F6FAFB" }, line: { color: "F6FAFB" } });
      s.addText(label, { x: lx + 0.05, y, w: lw, h: rowH, fontSize: 9.5, color: INK, fontFace: FF, bold: state === "done", valign: "middle" });
      const xa = X(a), xb = X(new Date((day(b) + 1) * 86400000).toISOString().slice(0, 10));
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: xa, y: y + 0.07, w: Math.max(0.08, xb - xa), h: rowH - 0.14, fill: { color: state === "done" ? TEAL_DK : TEAL }, line: { color: state === "done" ? TEAL_DK : TEAL }, rectRadius: 0.04 });
    });
    // Today, and the final delivery.
    const xt = X(TODAY) + gw / (d1 - d0) / 2, yb = top + PLAN.length * rowH;
    s.addShape(pres.shapes.LINE, { x: xt, y: top - 0.08, w: 0, h: yb - top + 0.16, line: { color: CHARCOAL, width: 1, dashType: "dash" } });
    s.addText("today", { x: xt - 0.3, y: yb + 0.06, w: 0.6, h: 0.22, fontSize: 8.5, color: CHARCOAL, fontFace: FF, align: "center", italic: true });
    const xf = X(PLAN_TO) + gw / (d1 - d0);
    s.addShape(pres.shapes.DIAMOND, { x: xf - 0.11, y: yb + 0.05, w: 0.22, h: 0.22, fill: { color: TEAL_DK }, line: { color: TEAL_DK } });
    s.addText("Final delivery, 30 Nov", { x: xf - 1.9, y: yb + 0.04, w: 1.72, h: 0.24, fontSize: 9, color: TEAL_DK, fontFace: FF, bold: true, align: "right" });
    s.addText("Afterwards: the same stack moves onto the drone (flight topology) when the airframe is ready.", {
      x: 0.55, y: 4.66, w: 8.9, h: 0.28, fontSize: 10.5, color: TEAL_DK, fontFace: FF, bold: true,
    });
    source(s, "Source: project plan as of 6 October 2026; final delivery date and items from the grant ('Grant overview', delivery dates).");
    H.badge(s, page);
    s.addNotes("The plan to the final delivery on 30 November. The Orin and the Project AirSim and ArduPilot rail come up in parallel through October. " +
      "The escalation state machine and ArduPilot's GeoFence go into the Shield node, and then the VLA, with the rule summary in its prompt, will fly the ArduPilot rail. " +
      "The KPI stress campaign runs in the hil topology in the third week of November, followed by the KPI report on dynamic-scenario stress, " +
      "the final demo and the signed final report. The grant's final delivery items are exactly these: the four components complete, the KPI " +
      "report, perception-rail integration and the signed final report.");
  }

  /* ---- README with the cover note, drafted from the same numbers ----------- */
  const BT = "`";
  const code = (t) => BT + t + BT;
  const video = path.join(SHARE, NFZ_VIDEO);
  const videoMB = fs.existsSync(video) ? fs.statSync(video).size / 1048576 : null;
  function readmeText(deckMB) {
    const negLine = W1.negative_run != null
      ? `, and so is each of the ${W1.negative_run} deliberately broken policies in the test corpus` : "";
    const negRow = W1.negative_run != null
      ? `, ${W1.negative_refused}/${W1.negative_run} broken policies refused (null ${W1.null_negative_refused}/${W1.negative_run})` : "";
    const p0Null = W2.kpi_discriminates === false
      ? "every policy fits whole, so the naive by-type cut also carries all" : `naive cut ${W2.baseline_p0_covered}/${W2.p0_in_scope}`;
    const BR = D.bench_refused;
    const draft = DRAFT ? [
      `> **DRAFT for review, not for sending.** Built with ${code("--allow-dirty")}; the cover says`,
      "> REVIEW DRAFT.",
      ...(DIRTY.length ? [
        `> - The working tree has ${DIRTY.length} uncommitted change(s) under the number sources`,
        `>   (${SOURCE_PATHS.map(code).join(", ")}).`,
      ] : []),
      ...(BENCH_STALE ? [
        `> - The Shield timing on slides 3 and 6 measured guardrail code ${code(G.code_sha)}`,
        `>   (${G.measured}), not the code in the tree now${BR ? ` (${code(BR.tree_code)})` : ""}.`,
        ...(BR ? [`>   A fresh bench run on ${BR.measured} refused itself: ${BR.why}.`] : []),
      ] : []),
      "> Fix what is listed, commit, then rebuild (data step, deck, PDF) before",
      "> sending; without --allow-dirty the deck step refuses each of these.",
      "",
    ] : [];
    const L = [
      "# Share pack: October 2026 progress report (for ITRI's monthly report)",
      "",
      ...draft,
      `Written by ${code("tools/deck/build_progress_oct_deck.js")} on every build, from`,
      `${code("docs/data/progress_oct.json")} (generated ${D.generated}, code revision`,
      `${code(D.code_revision)}). Status date on the slides: ${longDate(STATUS)}. Audience: ITRI.`,
      "PI's rule (6 Oct): progress-focused, no obstacles or shortcomings slide, every",
      "statement true, corrected numbers only. Do not edit this file by hand: the next",
      "build rewrites it.",
      "",
      "## What to send",
      "",
      "Upload these three files to OneDrive and send the link. This README and the",
      `${code("_build/")} folder stay local.`,
      "",
      "| File | What it is |",
      "|---|---|",
      `| ${code(DECK_NAME + ".pptx")} | ${page} slides, speaker note on every slide, Poppins embedded (${deckMB.toFixed(1)} MB). |`,
      `| ${code(DECK_NAME + ".pdf")} | The same deck as PDF, exported by PowerPoint (${page} pages). |`,
      `| ${code(NFZ_VIDEO)} | The 3 Oct city flight named on slide 11${videoMB ? ` (${Math.round(videoMB)} MB)` : ""}: the red car followed from ${m1(NFZ.cruise_alt_m).replace(/\.0$/, "")} m altitude past a no-fly zone, with the on-screen policy indicator. Same file as in the 30 Sept pack${videoMB ? "" : "; copy it from " + code("docs/share/2026-09-30-ITRI/")}. |`,
      "",
      "## Before sending (sender's checklist)",
      "",
      "1. Prof. Lai reviews the deck (his Q5 answer asked for it). Speaker notes are in",
      "   the .pptx; delete them first if ITRI should get slides only.",
      "2. Decide with Prof. Lai how this pack relates to the 30 Sept pack",
      `   (${code("docs/share/2026-09-30-ITRI/")}), which is still unsent and has its own`,
      "   pre-send steps in its README. This pack does not depend on it.",
      "3. Rebuild after the last commit (the three commands below). The deck step",
      `   refuses to run if ${code("progress_oct.json")} no longer matches the repository,`,
      "   or if the number sources have uncommitted changes.",
      "4. Paste the cover note into the email and fill in [name], [link] and [sender]",
      "   there.",
      "",
      "## Cover note (paste into the email)",
      "",
      "> Dear [name],",
      ">",
      "> For ITRI's monthly report, please find our October progress report (slides",
      `> and PDF) and one flight video at [link]. It covers the work from ${dayMonth(PERIOD_FROM)}`,
      `> to ${longDate(STATUS)}.`,
      ">",
      `> - **WP1 Policy DSL.** Policy bundles are now signed (Ed25519) and verified on`,
      `>   load. All ${W1.policies} policies in the repository round-trip with the same hash, and`,
      `>   each of the ${W1.tamper_run} tampered bundles in the test is refused${negLine}.`,
      `> - **WP2 Prefix Compiler.** The Constraint Summary Pack is a typed object with`,
      `>   the grant's ${W2.csp_fields_locked} locked fields. Every policy fits the ${W2.budget_tokens}-token budget whole`,
      `>   (${tokText} ${W2.tokens_exact_range ? "OpenVLA tokens" : "tokens"}), and the OpenVLA runner can place it in the prompt.`,
      `> - **WP3 Safety Shield.** At the grant's load of ${G.rules} rules and ${G.poses_per_check} future poses,`,
      `>   a rule check near a no-fly zone now takes ${ms(chk.current.median_ms)} median on a desktop CPU,`,
      `>   down from ${ms(chk.legacy.median_ms)}${PROF ? `; replayed over ${nf(PROF.check.n)} real flight ticks at the same load,` : "."}`,
      ...(PROF ? [`>   its p99 is ${ms(PROF.check.p99_ms)} against the grant's ${PROF.check.budget_ms} ms budget (desktop performance cores).`] : []),
      `>   Both measurements will be repeated on the Jetson Orin. The ${F.states.join(" / ")}`,
      ">   escalation state machine is built with the grant's default thresholds.",
      `> - **WP4 Stress testing.** Smoke (${smoke.episodes} episodes) and nightly (${nightly.episodes} episodes)`,
      `>   profiles at the grant's scale, a KPI report generator, and ${PP.paraphrases} stored`,
      ">   paraphrases for the per-paraphrase robustness KPI.",
      "> - **Policy indicator.** The on-screen indicator requested on 30 September,",
      ">   shown in a city flight with a no-fly zone (video attached).",
      ">",
      "> Next: a Jetson Orin is being set up for the grant's hil topology, where the",
      "> KPI campaign runs in November, and Project AirSim is being connected to",
      "> ArduPilot SITL with Mission Planner as the ground station. The final",
      "> delivery is on 30 November.",
      ">",
      "> Best regards,",
      "> [sender]",
      "",
      "## How it was built",
      "",
      "```",
      "C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_oct_data.py",
      "node tools/deck/build_progress_oct_deck.js",
      "powershell -File tools/office_to_pdf.ps1 -Path docs/share/2026-10-ITRI/Guardrail-Progress-Oct2026.pptx",
      "```",
      "",
      "- The data step runs the Shield benchmark itself (about two minutes) and stores",
      `  with each run ${code("code_sha")}, a hash of the code it timed. A stored run of other`,
      "  code is never quoted. Of two runs of the same code the fresh one is quoted,",
      `  unless its fence-free ${code("floor")} is more than ${num(D.bench_pick_rule.load_tolerance)} times the stored run's (a`,
      `  busy machine); the run not quoted is kept in ${code("load_check")} (${code("pick_bench")}).`,
      `  ${code("--bench-replace")} drops the stored runs. A bench that refuses itself (its new`,
      "  and legacy decisions disagree, or a sanity check fails) stops the data step;",
      `  ${code("--keep-stale-bench")} keeps the stored runs instead and records why, for a draft.`,
      `- The deck step first runs ${code("build_progress_oct_data.py --verify")}, which recomputes`,
      "  every other section and refuses a deck built from numbers the repository no",
      "  longer produces, or from Shield timing of other code, and refuses uncommitted",
      `  changes under the number sources (${code("--allow-dirty")} builds a draft marked REVIEW`,
      "  DRAFT on its cover and in this README). It then dumps every slide's text and notes to",
      `  ${code("_build/slides_text.json")}, drafts this README, and refuses to write anything`,
      "  if either contains a withdrawn claim or a number printed as 'undefined'.",
      `  It repairs and audits the package (${code("tools/fix_pptx_package.py")},`,
      `  ${code("tools/audit_pptx.py")}), embeds the fonts through PowerPoint and sets the`,
      `  package's last-saved-by to "${LAB}". Last, it writes ${code("_build/build_stamp.json")}`,
      "  with hashes of the builder, the data file and the deck, so a pack older than",
      "  its builder fails the test below.",
      `- Test: ${code("python tests/test_build_progress_oct_data.py")}.`,
      "",
      "## Where each number comes from",
      "",
      "| Slide | Numbers | Source |",
      "|---|---|---|",
      `| 3, 4 | ${W1.round_trip_passed}/${W1.policies} round trip, ${W1.tamper_refused}/${W1.tamper_run} tampered refused (null ${W1.null_tamper_refused}/${W1.null_tamper_run})${negRow} | ${code(W1.source)} |`,
      ...(suite ? [`| 3 | ${nf(suite.passed)} of ${nf(suite.total)} tests, ${suite.files} files | ${code("CHANGELOG.md")}, ${D.release.suite_section} section |`] : []),
      `| 3, 5 | ${W2.p0_covered}/${W2.p0_in_scope} P0 rules (empty CSP ${W2.null_p0_covered}/${W2.p0_in_scope}; ${p0Null}), ${tokText} tokens, ${W2.example.tokens_exact != null ? W2.example.tokens_exact + "-token example, " : ""}${W2.csp_fields_locked} fields | ${code("python -m guardrail.compiler report --exact")}, run by the data step |`,
      `| 3, 6 | ${ms(chk.legacy.median_ms)} to ${ms(chk.current.median_ms)} (check), ${ms(tick.legacy.median_ms)} to ${ms(tick.current.median_ms)} (tick), p99 ${ms(chk.current.p99_ms)} / ${ms(tick.current.p99_ms)} | ${code(G.command)}, desktop, measured ${G.measured}, code ${code(G.code_sha)}${BENCH_STALE ? " (not the tree's: DRAFT)" : ""} |`,
      ...(PROF ? [`| 6 | ${nf(PROF.check.n)} replayed flight ticks: check median ${ms(PROF.check.median_ms)}, p99 ${ms(PROF.check.p99_ms)}, max ${ms(PROF.check.max_ms)}, ${PROF.check.over_budget} over ${PROF.check.budget_ms} ms; tick p99 ${ms(PROF.filter.p99_ms)} | ${code(PROF.source)} (${code("tools/profile_shield_tick.py")}, performance cores ${PROF.cpu_affinity ? code(PROF.cpu_affinity.join(",")) : ""}), created ${PROF.created} |`] : []),
      `| 7 | N = ${F.n_violations}, T = ${F.window_s} s, theta ${F.theta_lateral_m} / ${F.theta_vertical_m} m, T_recover ${F.t_recover_s} s, target ${Math.round(F.failsafe_target * 100)} % | ${code("guardrail/fsm.py")} ${code("FSMConfig")} |`,
      `| 3, 8 | ${smoke.episodes} and ${nightly.episodes} episodes, ${nightly.cells} x ${nightly.seeds.length}, ${W4.n_templates} templates | profiles expanded by ${code("guardrail/scenario_spec.py")} (counted, not run) |`,
      `| 3, 9 | ${PP.paraphrases} paraphrases, ${nf(PP.mutants_refused)}/${nf(PP.mutants)} generated variants with a changed slot refused (copy-only null ${PP.null_copy_only_refused}) | ${code(PP.source)} |`,
      `| 11 | ${NFZ.zone_ticks_inside} ticks in the zone, closest ${m1(NFZ.zone_min_distance_m)} m, ${pct1(NFZ.within_30m)} within 30 m (hover null ${pct1(NFZ.within_30m_null_hover)}), ${pctN(NFZ.estimate_on_car)} estimate on car (fixed-at-start null ${pct1(NFZ.estimate_null_fixed_start)}), ${m1(NFZ.alt_min_m)}-${m1(NFZ.alt_max_m)} m altitude, ${m1(NFZ.sep_mean_m)} m mean distance | ${code("demo/out/" + NFZ.tag + "/")} metrics and flight log |`,
      `| 13 | ${code(PAS_CONTROLLER)}, UDP ${PAS_PORTS.replace(/ /g, "")}, ${code(SITL_CMD)} | Project AirSim ${code("example_user_scripts/ardupilot/")} |`,
      "| 14 | plan dates | plan of 6 Oct; final delivery date from the grant |",
      "",
      "## Wording the build enforces",
      "",
      `The guard in ${code("tools/deck/build_progress_oct_data.py")} (${code("WITHDRAWN")}) refuses the`,
      "claims this project has retracted, on the slides, in the notes and in the",
      "cover note above. The deck therefore says:",
      "",
      "- the desktop rail is the grant's **dev** topology; KPI figures come from the",
      "  Orin **hil** campaign in November;",
      "- the city flight was flown by the **follow controller**, which routed beside",
      "  the zone (banner AVOID), with the Safety Shield checking every command;",
      "- the CSP is **placed in the OpenVLA prompt by the runner**; flights with it",
      "  on are in the November plan;",
      "- Shield timings are **desktop** figures, to be repeated on the Orin;",
      "- the Project AirSim + ArduPilot + Mission Planner chain is **being connected**",
      "  (slide 13 is marked as plan).",
      "",
      "If asked in a meeting:",
      "",
      "- *Are these the contract KPIs?* The KPI campaign runs on the Orin in the hil",
      "  topology in the third week of November; the numbers here are development",
      "  measurements of the components.",
      // The replay on the desktop's efficiency cores is not on the slides
      // (the PI's rule: no shortcomings slide); the sender gets the true answer.
      ...(PROF && ECO ? [
        `- *Does the rule check fit ${ECO.check.budget_ms} ms on every CPU?* Not on every one: on the desktop's slower`,
        `  efficiency cores the same replay measured p99 ${ms(ECO.check.p99_ms)}, with ${nf(ECO.check.over_budget)} of`,
        `  ${nf(ECO.check.n)} ticks over ${ECO.check.budget_ms} ms (${code(ECO.source)}). In the grant's hil`,
        "  topology the Shield runs on the Orin, and the grant measures every reported",
        "  number there; the Orin profile in the hil campaign answers this.",
      ] : []),
      "- *Can Project AirSim work with Mission Planner?* Yes, through ArduPilot:",
      `  Project AirSim's ${code(PAS_CONTROLLER)} controller lets ArduPilot SITL fly on its physics, and`,
      "  Mission Planner talks MAVLink to ArduPilot via mavlink-router (slide 13).",
      "",
    ];
    return L.join("\n");
  }

  /* ---- Guard, write, repair, audit, embed fonts ----------------------------- */
  fs.mkdirSync(BUILD, { recursive: true });
  const dump = SLIDES.map((r) => ({ n: r.n, title: r.title, text: r.text.join("\n"), notes: r.notes }));
  fs.writeFileSync(TEXT_DUMP, JSON.stringify(dump, null, 1) + "\n", "utf8");
  const guard = (readmeFile) => {
    try {
      execFileSync(PY, [DATA_PY, "--check-deck", TEXT_DUMP, "--cover", readmeFile], { stdio: "inherit" });
    } catch (e) {
      console.error("REFUSED: the deck text or the cover note failed the claims check; nothing written.");
      process.exit(1);
    }
  };
  // The README's deck size is only known after the build; the draft's text is
  // otherwise identical, and the final file is checked again before it is written.
  fs.writeFileSync(README_DRAFT, readmeText(0), "utf8");
  guard(README_DRAFT);
  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  await embedFonts(OUT);
  await setLastModifiedBy(OUT, LAB);
  const mb = fs.statSync(OUT).size / 1048576;
  console.log(`wrote ${OUT}`);
  console.log(`slides: ${page}`);
  console.log(`size  : ${mb.toFixed(2)} MB`);
  if (mb > MAX_MB) {
    console.error(`REFUSED: deck exceeds ${MAX_MB} MB. Something is embedding media.`);
    process.exit(1);
  }
  fs.writeFileSync(README_DRAFT, readmeText(mb), "utf8");
  guard(README_DRAFT);
  fs.copyFileSync(README_DRAFT, README);
  console.log(`wrote ${README}`);
  const stray = strayCopies();
  if (stray.length) {
    console.error(`REFUSED: other copies of the deck appeared in ${SHARE} during the build: ${stray.join(", ")}`);
    process.exit(1);
  }
  if (sha256(__filename) !== BUILDER_SHA || sha256(DATA_JSON) !== DATA_SHA) {
    fs.rmSync(STAMP, { force: true });
    console.error("REFUSED: the builder or progress_oct.json changed while the deck was built; build again.");
    process.exit(1);
  }
  fs.writeFileSync(STAMP, JSON.stringify({
    built: new Date().toISOString().replace(/\.\d+Z$/, "Z"),
    builder: "tools/deck/build_progress_oct_deck.js", builder_sha256: BUILDER_SHA,
    data: "docs/data/progress_oct.json", data_sha256: DATA_SHA,
    deck: path.basename(OUT), deck_sha256: sha256(OUT), slides: page,
    draft: DRAFT, uncommitted_sources: DIRTY.length, bench_of_other_code: BENCH_STALE,
  }, null, 1) + "\n", "utf8");
  console.log(`wrote ${STAMP}${DRAFT ? " (DRAFT)" : ""}`);
}

/* PowerPoint's font-embedding save records the Windows user as the last
 * author. ITRI material names the lab, so core.xml's lastModifiedBy (and the
 * creator, if PowerPoint changed it) is set to `who`, and the package is
 * audited again. */
async function setLastModifiedBy(pptx, who) {
  const JSZip = require("jszip");
  const zip = await JSZip.loadAsync(fs.readFileSync(pptx));
  const f = zip.file("docProps/core.xml");
  if (!f) throw new Error("docProps/core.xml missing");
  const xml = await f.async("string");
  const esc = who.replace(/&/g, "&amp;").replace(/</g, "&lt;");
  let out = xml.replace(/<cp:lastModifiedBy>[^<]*<\/cp:lastModifiedBy>/, `<cp:lastModifiedBy>${esc}</cp:lastModifiedBy>`)
    .replace(/<dc:creator>[^<]*<\/dc:creator>/, `<dc:creator>${esc}</dc:creator>`);
  if (!/<cp:lastModifiedBy>/.test(out)) out = out.replace("</cp:coreProperties>", `<cp:lastModifiedBy>${esc}</cp:lastModifiedBy></cp:coreProperties>`);
  // createFolders: false - JSZip otherwise adds a "docProps/" directory
  // entry, which is not an OPC part (tools/audit_pptx.py refuses it).
  zip.file("docProps/core.xml", out, { createFolders: false });
  const buf = await zip.generateAsync({ type: "nodebuffer", compression: "DEFLATE", compressionOptions: { level: 6 } });
  fs.writeFileSync(pptx, buf);
  const check = await (await JSZip.loadAsync(fs.readFileSync(pptx))).file("docProps/core.xml").async("string");
  const by = /<cp:lastModifiedBy>([^<]*)<\/cp:lastModifiedBy>/.exec(check);
  if (!by || by[1] !== esc) throw new Error(`lastModifiedBy is ${by ? by[1] : "absent"} after rewrite`);
  execFileSync(PY, [path.join(REPO, "tools/audit_pptx.py"), pptx], { stdio: "inherit" });
  console.log(`saved by: ${who}`);
}

/* Embed the fonts (copied from build_progress_0930_deck.js): Poppins is
 * installed per user on this PC only, so PowerPoint re-saves the audited deck
 * with EmbedTrueTypeFonts, then the result is checked and re-audited. */
async function embedFonts(pptx) {
  const os = require("os");
  const JSZip = require("jszip");
  const tmp = path.join(os.tmpdir(), `embed_${process.pid}_${path.basename(pptx)}`);
  const started = Date.now();
  const ps = [
    "$ErrorActionPreference = 'Stop'",
    "$before = @(Get-Process POWERPNT -ErrorAction SilentlyContinue)",
    "$app = New-Object -ComObject PowerPoint.Application",
    "$pres = $null",
    "try { $pres = $app.Presentations.Open($env:EMBED_IN, $true, $false, $false); $pres.SaveAs($env:EMBED_OUT, 24, -1) }" +
      " finally { if ($pres) { $pres.Close() }; if ($before.Count -eq 0) { try { $app.Quit() } catch {} };" +
      " [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app) }",
  ].join("; ");
  execFileSync("powershell", ["-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps],
    { stdio: "inherit", env: { ...process.env, EMBED_IN: pptx, EMBED_OUT: tmp } });
  if (!fs.existsSync(tmp) || fs.statSync(tmp).mtimeMs < started - 2000) {
    throw new Error(`font embedding wrote no new file at ${tmp}`);
  }
  const zip = await JSZip.loadAsync(fs.readFileSync(tmp));
  const presXml = await zip.file("ppt/presentation.xml").async("string");
  const lst = /<p:embeddedFontLst>([\s\S]*?)<\/p:embeddedFontLst>/.exec(presXml);
  const faces = lst ? [...lst[1].matchAll(/typeface="([^"]+)"/g)].map((m) => m[1]) : [];
  const parts = Object.keys(zip.files).filter((f) => f.startsWith("ppt/fonts/"));
  if (!faces.includes(T.FF) || parts.length === 0) {
    throw new Error(`fonts not embedded (faces: ${faces.join(", ") || "none"}; parts: ${parts.length})`);
  }
  fs.copyFileSync(tmp, pptx);
  fs.unlinkSync(tmp);
  execFileSync(PY, [path.join(REPO, "tools/audit_pptx.py"), pptx], { stdio: "inherit" });
  console.log(`fonts : embedded ${faces.join(", ")} (${parts.length} parts)`);
}

main().catch((e) => { console.error(e); process.exit(1); });
