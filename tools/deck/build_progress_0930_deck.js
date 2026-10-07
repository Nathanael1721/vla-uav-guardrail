/*
 * Guardrail progress since 16 September — lab meeting, 30 September 2026.
 * nathan-deck design system (tools/deck/nathan_theme.js).
 *
 * Flight, replay, identity-gate and Simulate numbers are read at build time from
 * docs/data/progress_0930.json (tools/deck/build_progress_0930_data.py). The
 * identity thresholds come from demo/identity_thresholds.json, the scenario
 * counts from docs/data/scenario_sweep.json, the replay's reproduced-flight
 * numbers from demo/out/identity_replay/pipeline.json and the pedestrian ring
 * score from demo/out/citylife_ped_0930/metrics.json. A number with no key in
 * any of those files is a named constant below, with a comment naming its source.
 *
 * Shape follows how the 16 September deck was actually presented: no name
 * block on the cover, no timeline, no conclusion, and "Limits" as a backup
 * slide after "Thank You". No video is embedded; the build refuses a deck
 * over 10 MB. After the audit PowerPoint re-saves the deck with its fonts
 * embedded (embedFonts), so it needs PowerPoint, as the PDF export does.
 *
 *   C:/Users/natha/.conda/envs/pas/python.exe tools/deck/progress_0930_charts.py
 *   node tools/deck/build_progress_0930_deck.js
 */
const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const sharp = require("sharp");
const {
  FaCheckCircle, FaCrosshairs, FaLock, FaArrowsAltV, FaExpandArrowsAlt, FaBan, FaAdjust,
  FaTrafficLight, FaWalking, FaBug, FaEyeSlash, FaFlask, FaPlaneArrival, FaExclamationTriangle,
  FaUserFriends, FaLocationArrow, FaTachometerAlt, FaVideo, FaClipboardCheck, FaRoute,
  FaHourglassHalf, FaRoad, FaUndo, FaQuestionCircle, FaLightbulb,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "..", "..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const readJson = (rel) => JSON.parse(fs.readFileSync(path.join(REPO, rel), "utf8"));
const P = readJson("docs/data/progress_0930.json");
const TH = readJson("demo/identity_thresholds.json");
const SWEEP = readJson("docs/data/scenario_sweep.json");
// The replay itself, for the four flights its old arm reproduces row for row
// ("pooled, reproduced flights" in demo/out/identity_replay/pipeline.md).
const PIPE = readJson("demo/out/identity_replay/pipeline.json");
// The pedestrian flight's own scoring of the 10 m ring against ground truth.
const PED_M = readJson("demo/out/citylife_ped_0930/metrics.json");
const OUT = path.join(REPO, "docs/Guardrail-Progress-30Sep2026.pptx");
const MAX_MB = 10;

const IMG = {
  before: path.join(REPO, "docs/img/progress_0930_before_fpv.jpg"),
  after: path.join(REPO, "docs/img/progress_0930_after_fpv.jpg"),
  estChart: path.join(REPO, "docs/img/progress_0930_estimate_on_car.png"),
  w30Chart: path.join(REPO, "docs/img/progress_0930_within_30m.png"),
  junction: path.join(REPO, "docs/img/citylife_signals_junction_4100_4100_low.png"),
};

/* ---- Constants with no key in the JSON files, each with its source ---------- */
// docs/img/progress_0930_before_fpv.jpg HUD (citylife_redcar_trail fpv/03600.jpg).
const BEFORE_T_S = 180, BEFORE_SEP_M = 110;
// demo/out/citylife_redcar_trail/{metrics,manifest,kpi}.json and fpv/03600.jpg
// are dated 2026-09-24 21:06-21:07, code_revision 82295b1717ce-dirty. It was
// flown on 24 Sept; 29 Sept is when the FINDING analysed it.
const BEFORE_DATE = "24 September", BEFORE_DATE_SHORT = "24 Sept";
// docs/img/progress_0930_after_fpv.jpg HUD (citylife_redcar_id1 fpv/03000.jpg).
const AFTER_SEP_M = 19.9;
// docs/FINDING-the-lock-that-could-not-let-go.md, causes (1) and (3).
const OLD_GROUND_SIGNAL_M = "2-3", OLD_GROUND_PASS = "~95 %", OLD_GATE_M = "75-190";
// demo/follow_vlm.py argparse defaults: --lapse-s 3.0, Reacquirer(need=4, max_range_m=45.0).
const LAPSE_S = 3, REACQ_NEED = 4, REACQ_MAX_M = 45;
// demo/landing.py SITE_MARGIN_M.
const SITE_MARGIN_M = 1;
// CHANGELOG.md [Unreleased] 2026-09-30: the replay's old arm is the 1d09786
// estimator, never reset. pipeline.md "Is the old arm the flown one?": it matches
// the flown `served` flag row for row only on PIPE._reproduced_flights (final1..4).
const OLD_ARM_COMMIT = "1d09786";
// docs/WORKLOG.md 2026-09-30, id3 notes: sharp turn window, car at the light.
const ID3_TURN = "110-114", ID3_LIGHT_M = 51.8;
// docs/WORKLOG.md 2026-09-30, replay of id3 candidates through Reacquirer.far_lead.
const FAR_LEAD_T_S = 126, FAR_LEAD_OFF_M = 4.3;
// CHANGELOG.md 2026-09-30: docs/video/citylife_redcar_identity.mp4 length.
const VIDEO_S = 262;
// CHANGELOG.md 2026-09-21 "Fixed": car paint material, eight colours; six
// confirmed by measured hue (the white truck and red sports car in shadow).
const PAINT_COLOURS = 8, PAINT_HUE_CONFIRMED = 6;
// CHANGELOG.md 2026-09-22 / 09-29: 24 cars on 3 keep-left loops.
const CARS = 24, LOOPS = 3;
// tools/citylife_signals.py fixed-time plan: 50 s cycle.
const SIGNAL_CYCLE_S = 50;
// Project detector gate (docs/MID-EVALUATION-REPORT-Sep2026.md s.6).
const DET_GATE_HZ = 4.0;
// CHANGELOG.md 2026-09-30 "Known limitations": ped_0930 ticks, facade ranges.
const PED_TICKS = 1788, FACADE_RANGE_M = "40-130";
// CHANGELOG.md 2026-09-30 / FINDING-citylife-level.md: signal-head and zebra coverage.
const LOOP_JUNCTIONS = 13, LOOP_JUNCTIONS_WITH_VEH_HEADS = 0, USABLE_ZEBRAS = 5;
// CHANGELOG.md [Unreleased] 2026-09-29 "Retracted" and 2026-09-30 "Retracted / corrected".
const RETRACTED = {
  // midEvalDet is printed as the quoted, retracted claim (CORRECTION row A14).
  detOld: "6.9-7.5", detNew: "3.5-4.0", midEvalDet: '"3.76-5.22 Hz"', midEvalDetNew: "2.77-4.31",
  replayOldOn: "22.4", replayOldNew: "95.5", replayOldSeeds: "0/18", replayOldWrong: "260/293",
  pinholeBelowChance: 3,
};
// demo/out/ros2_*/kpi.json are dated 2026-09-01; this deck is 2026-09-30.
const KPI_DATE = "1 Sept", KPI_AGE_D = 29;
// docs/data/scenario_sweep.json file date (the sweep has not been rerun since).
const SWEEP_DATE = "9 Sept";
// Gazebo: asked for in the meeting notes of 2026-08-19, 09-02 and 09-16; no
// gazebo folder in demo/out; docs/MID-EVALUATION-REPORT-Sep2026.md s.7 "Open".
const GAZEBO_ASKED = "19 Aug, 2 and 16 Sept";
// demo/out/citylife_ped_0930/flight_log.jsonl: truth.pts holds all 40 level
// figures on every row, and the separation is to the NEAREST of them
// (demo/follow_vlm.py _sep), not to one tracked person.
const PED_TRUTH_FIGURES = 40;
// The arrow glyph: Poppins has no U+2192, so PowerPoint would substitute Times
// New Roman. Segoe UI has it and matches the sans weight.
const ARROW_FONT = "Segoe UI";

/* ---- Formatting ---------------------------------------------------------- */
const r1 = (v) => (Math.round(v * 10 + 1e-9) / 10).toFixed(1);           // 3.05 -> "3.1"
const pct1 = (v) => (Math.round(v * 1000 + 1e-9) / 10).toFixed(1) + " %"; // 0.992 -> "99.2 %"
const pctN = (v) => {                                                      // 1.0 -> "100", 0.996 -> "99.6"
  const x = Math.round(v * 1000 + 1e-9) / 10;
  return Number.isInteger(x) ? String(x) : x.toFixed(1);
};
const pct0 = (v) => Math.round(v * 100 + 1e-9) + " %";                    // 0.15 -> "15 %"
const nf = (n) => Number(n).toLocaleString("en-US");                      // 1502 -> "1,502"
const minmax = (a) => [Math.min(...a), Math.max(...a)];
const range1 = (a) => { const [lo, hi] = minmax(a).map(r1); return lo === hi ? lo : `${lo}-${hi}`; };
const short = (tag) => tag.replace("citylife_redcar_", "");

/* A JPEG from the repository kept as JPEG (img() would re-encode a photo as a
 * much larger PNG). */
async function photo(file) {
  const buf = fs.readFileSync(file);
  const m = await sharp(buf).metadata();
  return { data: "image/jpeg;base64," + buf.toString("base64"), aspect: m.width / m.height };
}

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "Guardrail — Progress Since 16 September";
  const H = helpers(pres);
  const { TEAL, TEAL_DK, TEAL_TINT, TEAL_TINT2, WHITE, INK, GREY, LGREY, LINE, FF, CHARCOAL } = T;

  const ic = {
    ok: await iconPng(FaCheckCircle), aim: await iconPng(FaCrosshairs), lock: await iconPng(FaLock),
    ground: await iconPng(FaArrowsAltV), gate: await iconPng(FaExpandArrowsAlt), ban: await iconPng(FaBan),
    soft: await iconPng(FaAdjust), light: await iconPng(FaTrafficLight), walk: await iconPng(FaWalking),
    bug: await iconPng(FaBug), blind: await iconPng(FaEyeSlash), flask: await iconPng(FaFlask),
    land: await iconPng(FaPlaneArrival), warn: await iconPng(FaExclamationTriangle),
    people: await iconPng(FaUserFriends), lead: await iconPng(FaLocationArrow),
    speed: await iconPng(FaTachometerAlt), video: await iconPng(FaVideo), clip: await iconPng(FaClipboardCheck),
    route: await iconPng(FaRoute), wait: await iconPng(FaHourglassHalf), road: await iconPng(FaRoad),
    undo: await iconPng(FaUndo), ask: await iconPng(FaQuestionCircle), idea: await iconPng(FaLightbulb),
  };

  /* ---- Derived numbers ---------------------------------------------------- */
  const NEW = P.red_new, OLD = Object.values(P.red_old), RP = P.replay, IG = P.identity_gate, SIM = P.simulate;
  const id3 = NEW.find((r) => r.tag === "citylife_redcar_id3");
  const id4 = NEW.find((r) => r.tag === "citylife_redcar_id4");
  const id3Reacq = id3.reacq_events.find((e) => e.event === "reacquired");
  const id3Lapse = id3.reacq_events.find((e) => e.event === "lapse");
  // "obstacle_6to14: 6-14 m obstacle 2.94 m away, needs 3.0 m"
  const landMatch = /([\d.]+) m away, needs ([\d.]+) m/.exec(id3.landing_reasons[0]);
  const landGot = Number(landMatch[1]), landNeed = Number(landMatch[2]);
  const landShortCm = Math.round((landNeed - landGot) * 100);
  const detRange = range1(NEW.map((r) => r.det_hz));
  const loopRange = range1(NEW.map((r) => r.loop_hz));
  const p0Max = Math.max(...NEW.map((r) => r.p0));
  const collisions = NEW.reduce((a, r) => a + r.collisions_mission, 0);
  const notLandable = NEW.filter((r) => !r.landing_final_landable).map((r) => short(r.tag));
  const [oldOnLo, oldOnHi] = minmax(OLD.map((r) => r.estimate_on_car_as_flown));
  const [newOnLo, newOnHi] = minmax(NEW.map((r) => r.estimate_on_subject));
  const simMin = (s) => Math.round(s / 60);
  const simLongS = Math.max(...SIM.after_fix_t_s);
  const ped = P.other.citylife_ped_0930, pedId = P.other.citylife_ped_id, demoId = P.other.demo_follow_identity;
  const demoAcq = demoId.reacq_events.find((e) => e.event === "reacquired");
  const sweepTotal = SWEEP._counts.pass + SWEEP._counts.fail + SWEEP._counts.known_failure + SWEEP._counts.skipped;

  let page = 1;
  const content = (eyebrow, title) => {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, eyebrow, title);
    return s;
  };
  /* The card() frame alone, for the compact variants below. */
  const cardBox = (s, x, y, w, h) => s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x, y, w, h, fill: { color: WHITE }, line: { color: LINE, width: 1 }, rectRadius: 0.08,
    shadow: { type: "outer", blur: 6, offset: 2, angle: 90, color: "D9DEE3", opacity: 0.4 },
  });
  /* A numbered card: teal disc, title and body side by side (compact). */
  function numCard(s, x, y, w, h, n, title, body, bodySize = 10.5) {
    cardBox(s, x, y, w, h);
    s.addShape(pres.shapes.OVAL, { x: x + 0.2, y: y + 0.18, w: 0.38, h: 0.38, fill: { color: TEAL }, line: { color: TEAL } });
    s.addText(String(n), { x: x + 0.2, y: y + 0.18, w: 0.38, h: 0.38, fontSize: 12, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
    s.addText(title, { x: x + 0.72, y: y + 0.1, w: w - 0.9, h: 0.36, fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle" });
    s.addText(body, { x: x + 0.72, y: y + 0.45, w: w - 0.9, h: h - 0.5, fontSize: bodySize, color: GREY, fontFace: FF, lineSpacingMultiple: 1.15, valign: "top" });
  }
  /* A card whose headline is a small teal tag, body in ink. */
  function tagCard(s, x, y, w, h, icon, tag, body, bodySize = 11) {
    cardBox(s, x, y, w, h);
    s.addImage({ data: icon, x: x + 0.22, y: y + 0.2, w: 0.26, h: 0.26 });
    s.addText(tag.toUpperCase(), { x: x + 0.56, y: y + 0.17, w: w - 0.76, h: 0.32, fontSize: 9, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
    s.addText(body, { x: x + 0.22, y: y + 0.56, w: w - 0.44, h: h - 0.7, fontSize: bodySize, color: INK, fontFace: FF, lineSpacingMultiple: 1.2, valign: "top" });
  }
  /* Numbered steps (build_updates_deck.js "Next Steps" pattern). */
  function steps(s, items, { y0 = 1.72, dy = 0.64, rowH = 0.56, tx = 1.2, tw = 3.5, bx = 4.7, bw = 4.7, bodySize = 11 } = {}) {
    items.forEach(([t, b], i) => {
      const y = y0 + i * dy;
      s.addShape(pres.shapes.OVAL, { x: 0.6, y: y + (rowH - 0.42) / 2, w: 0.42, h: 0.42, fill: { color: TEAL }, line: { color: TEAL } });
      s.addText(String(i + 1), { x: 0.6, y: y + (rowH - 0.42) / 2, w: 0.42, h: 0.42, fontSize: 12, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(t, { x: tx, y, w: tw, h: rowH, fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(b, { x: bx, y, w: bw, h: rowH, fontSize: bodySize, color: GREY, fontFace: FF, valign: "middle", lineSpacingMultiple: 1.15 });
      if (i < items.length - 1) s.addShape(pres.shapes.LINE, { x: tx, y: y + dy - 0.04, w: 9.4 - tx, h: 0, line: { color: LINE, width: 1 } });
    });
  }
  function arrowR(s, x1, y, x2) {
    s.addShape(pres.shapes.LINE, { x: x1, y, w: x2 - x1, h: 0, line: { color: CHARCOAL, width: 1.25, endArrowType: "triangle" } });
  }
  /* "a → b" as text runs: the arrow in ARROW_FONT, the rest inherits Poppins. */
  const arrow = (a, b) => [{ text: `${a} ` }, { text: "→", options: { fontFace: ARROW_FONT } }, { text: ` ${b}` }];
  /* The source caption. nathan_theme's source() is 8.5 pt LGREY, about 2.5:1 on
   * white; this deck uses 9.5 pt GREY (about 4.8:1) so it survives a projector. */
  function source(s, txt) {
    s.addText(txt, { x: 0.55, y: 4.98, w: 8.6, h: 0.28, fontSize: 9.5, color: GREY, fontFace: FF, italic: true });
  }
  /* H.table with a height per row, for a table whose cells wrap unevenly. Same
   * archetype: teal header, TEAL_TINT zebra, LINE hairlines, teal `emph` cells. */
  function tableV(s, rows, { x = 0.55, y, w = 8.9, colFrac, rhs, fontSize = 10, emph = [], align }) {
    const colW = colFrac.map((f) => w * f);
    let ry = y;
    rows.forEach((r, ri) => {
      const header = ri === 0, zebra = !header && ri % 2 === 0;
      let cx = x;
      r.forEach((cell, ci) => {
        s.addShape(pres.shapes.RECTANGLE, {
          x: cx, y: ry, w: colW[ci], h: rhs[ri],
          fill: { color: header ? TEAL : zebra ? TEAL_TINT : WHITE }, line: { color: LINE, width: 1 },
        });
        const e = !header && emph.some(([er, ec]) => er === ri && ec === ci);
        s.addText(String(cell), {
          x: cx + 0.1, y: ry, w: colW[ci] - 0.2, h: rhs[ri],
          fontSize: header ? fontSize + 0.5 : fontSize, color: header ? WHITE : e ? TEAL : INK,
          bold: header || e, fontFace: FF, align: align ? align[ci] : ci === 0 ? "left" : "center", valign: "middle",
        });
        cx += colW[ci];
      });
      ry += rhs[ri];
    });
    return ry;
  }

  const beforeImg = await photo(IMG.before);
  const afterImg = await photo(IMG.after);

  /* 01 · COVER ------------------------------------------------------------- */
  {
    const s = pres.addSlide();
    H.darkBase(s, "ITRI VLA-UAV · Guardrail · 30 September 2026");
    s.addText("Progress Since\n16 September", {
      x: 0.55, y: 1.15, w: 8.4, h: 1.9, fontSize: 40, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1,
    });
    s.addText("Following the right car, in a city that behaves", {
      x: 0.55, y: 3.15, w: 8.4, h: 0.6, fontSize: 15, color: TEAL_TINT, fontFace: FF,
    });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.15, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
  }

  /* 02 · ACTION ITEMS ------------------------------------------------------ */
  {
    const s = content("Since 16 September", "Action Items");
    const rows = [
      ["Action item", "Status", "Evidence"],
      ["Better pedestrian models", "Done", `${SIM.before_fix_figures} City Sample Crowd figures, hands and hair`],
      ["Pedestrians that walk", "Done", "Routes; wait at the kerb; cross on the walk phase"],
      ["Car colour variety", "Done", `Paint fixed (${PAINT_COLOURS} colours, ${PAINT_HUE_CONFIRMED} confirmed by hue); exactly one red car`],
      ["Resolution vs 10 Hz", "Loop done", `Loop ${loopRange} Hz; detector ${detRange} Hz (gate ${r1(DET_GATE_HZ)})`],
      // 16 Sept item: try a non-MetaHuman model and check its cost. The crowd
      // is MetaHuman-derived (metahuman_base_skel, CHANGELOG 09-22) and its
      // frame-rate cost "is unknown" (CHANGELOG 09-21 "Not measured").
      ["Lighter than MetaHuman", "Open", "MetaHuman-derived crowd; no other model tried; cost not measured"],
      ["Gazebo visualisation", "Open", "Not started"],
      ["Pipeline write-up + demo", "Partial", `Lock/identity write-up + ${VIDEO_S} s video`],
      // Crowd, intersection, NFZ, corridor, square route: none added since; the
      // sweep is dated 9 Sept and the CityLife policies have no fence or corridor.
      ["Several demo scenarios", "Open", `None added; NFZ/corridor only headless (${SWEEP_DATE}), not in CityLife`],
    ];
    const emph = rows.map((r, i) => (i > 0 && r[1].startsWith("Done") ? [i, 1] : null)).filter(Boolean);
    H.table(s, rows, { y: 1.66, rh: 0.36, fontSize: 10, colFrac: [0.255, 0.135, 0.61], emph, align: ["left", "center", "left"] });
    source(s, "Source: action items in the 16 September seminar minutes.");
    H.badge(s, page);
  }

  /* 03 · THE PROBLEM WE SAW ------------------------------------------------ */
  {
    const s = content(`Reference flight, ${BEFORE_DATE}`, "The Problem We Saw");
    const f = H.figure(s, beforeImg, 0.55, 1.66, 4.4, 4.4 / beforeImg.aspect);
    s.addText([
      { text: `t = ${BEFORE_T_S} s · HUD: TARGET LOCKED · car ${BEFORE_SEP_M} m away`, options: { bold: true, color: INK, breakLine: true } },
      { text: "'NFZ AHEAD' with no NFZ in the policy", options: { color: GREY } },
    ], { x: 0.55, y: f.y + f.h + 0.06, w: 4.4, h: 0.52, fontSize: 10, fontFace: FF, lineSpacingMultiple: 1.15, valign: "top" });
    const cx = 5.2, cw = 4.25, ch = 0.98, gap = 0.12;
    numCard(s, cx, 1.66, cw, ch, 1, "Ground check", `A ${OLD_GROUND_SIGNAL_M} m signal passed ${OLD_GROUND_PASS} of the time`);
    numCard(s, cx, 1.66 + ch + gap, cw, ch, 2, "Lock", "Could not say 'none';\ntook the best-scoring box");
    numCard(s, cx, 1.66 + 2 * (ch + gap), cw, ch, 3, "Estimator gate", `Grew to ${OLD_GATE_M} m after a loss;\nfar box re-seeded it`);
    source(s, "Source: docs/FINDING-the-lock-that-could-not-let-go.md; frame citylife_redcar_trail/view/fpv/03600.jpg.");
    H.badge(s, page);
  }

  /* 04 · JUDGE EVERY CANDIDATE --------------------------------------------- */
  {
    const s = content("Fix 1 · physical identity", "Judge Every Candidate");
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.48, y0 = 1.66;
    // REJECT / DOUBTFUL, not HARD / SOFT: the grant's DSL already uses hard/soft
    // for policy rules (demo/identity.py, TIER_LABEL). Renamed 2026-10-03.
    H.card(s, 0.55, y0, cw, ch, ic.ban, "REJECT",
      `Impossible for a car: bottom > ${TH.bottom_max} m up, > ${TH.off_street_hard} m off street, > ${TH.width_max} m wide. Never steers.`, 10);
    H.card(s, 0.55 + cw + 0.2, y0, cw, ch, ic.soft, "DOUBTFUL",
      "A real car can look like this (half behind a truck). May continue, never start.", 10);
    H.card(s, 0.55 + 2 * (cw + 0.2), y0, cw, ch, ic.ok, "OK", "The rest.", 10);
    s.addText("Judged at each frame's own pose and depth.", {
      x: 0.55, y: y0 + ch + 0.08, w: 8.9, h: 0.3, fontSize: 11, color: INK, fontFace: FF, bold: true,
    });
    const ho = IG.held_out, g = IG.gate;
    const rows = [
      ["Leave-one-flight-out", "Wrong boxes not OK", "True boxes not OK", "True boxes REJECT"],
      ["Held out", pct1(ho.off_notok_rate), pct1(ho.on_notok_rate), pct1(ho.on_hard_rate)],
      ["Gate", `≥ ${pct0(g.off_notok_min)}`, `≤ ${pct0(g.on_notok_max)}`, `≤ ${pct0(g.on_hard_max)}`],
    ];
    const tb = H.table(s, rows, { y: 3.58, rh: 0.34, fontSize: 10.5, colFrac: [0.25, 0.25, 0.25, 0.25], emph: [[1, 1], [1, 3]] });
    s.addText(`Near miss on the true-box gate (${pctN(ho.on_notok_rate)} vs ${pct0(g.on_notok_max)}), not a pass.`, {
      x: 0.55, y: tb + 0.06, w: 8.9, h: 0.3, fontSize: 11, color: TEAL, fontFace: FF, bold: true,
    });
    source(s, `Source: tools/replay_identity.py, ${TH.tuned_on.length} recorded flights, ${nf(IG.n_wrong_boxes)} wrong / ${nf(IG.n_true_boxes)} true boxes; thresholds ${IG.thresholds_hash}.`);
    H.badge(s, page);
  }

  /* 05 · LOSE IT, THEN TAKE IT BACK ---------------------------------------- */
  {
    const s = content("Fix 2 · strict lock and re-acquisition", "Lose It, Then Take It Back on Evidence");
    steps(s, [
      ["Strict lock", "Can answer 'none'. Seeded only by the start gate or a re-acquisition."],
      [`Lapse after ${LAPSE_S} s`, `No accepted measurement for ${LAPSE_S} s: the estimate is dropped.`],
      ["Re-acquire", `${REACQ_NEED} OK sightings in a row, ≤ ${REACQ_MAX_M} m, within reach of the last position.`],
      // FINDING-the-lock, "The motion rule came from the replay": the FIRST
      // Reacquirer re-seeded twice on a still sign in citylife_redcar_far.
      ["Seen moving", "When nothing anchors it: must move. In replay, the first Reacquirer re-seeded on a still fire-hydrant sign."],
      ["Far lead (new, 30 Sept)", `Car seen beyond ${REACQ_MAX_M} m: fly toward it; the gate still decides.`],
    ], { y0: 1.66, dy: 0.64, tw: 2.9, bx: 4.15, bw: 5.25, bodySize: 10.5 });
    source(s, "Source: demo/follow_vlm.py TargetLock.select_strict, Reacquirer; docs/FINDING-the-lock-that-could-not-let-go.md.");
    H.badge(s, page);
  }

  /* 06 · REPLAYED ON WHAT WAS FLOWN ---------------------------------------- */
  {
    const s = content(`Open-loop replay, ${RP.flights} flights, same boxes`, "Replayed on What Was Flown");
    const y = 1.7, sz = 24;
    H.stat(s, 0.55, y, 2.95, arrow(pct1(RP.as_flown_on_car_of_served), pct1(RP.new_on_car_of_served)),
      "Estimate on the car (≤ 6 m), share of served ticks", TEAL, sz);
    H.stat(s, 3.55, y, 2.95, arrow(`${RP.as_flown_false_post_gap} / ${RP.as_flown_post_gap}`, `${RP.new_false_seeds} / ${RP.new_seeds}`),
      "Wrong first accepts after a gap (old) vs wrong seeds (new)", TEAL, sz);
    // Not "12" next to "0 / 12": the two stats read as one string.
    H.stat(s, 6.8, y, 2.65, `${RP.new_seeds_start} + ${RP.new_seeds_reacq}`,
      `Seeds: starts + re-acquisitions (${RP.new_seeds})`, TEAL, sz);
    H.card(s, 0.55, 3.0, 8.9, 1.1, ic.warn, "Caveat",
      `Served on ${pct1(RP.new_served_frac)} of ticks: the replay only has the boxes the old lock picked.`, 11);
    const repro = PIPE._pooled_reproduced, reproTags = PIPE._reproduced_flights.map(short);
    const reproSpan = `${reproTags[0]}..${reproTags[reproTags.length - 1].replace(/^\D+/, "")}`;   // "final1..4"
    s.addText([
      { text: `Old arm = the ${OLD_ARM_COMMIT} estimator, never reset; it reproduces what flew on ${reproSpan} only (there: ` },
      ...arrow(pct1(repro.old_as_flown.on_frac_of_served), pct1(repro.new.on_frac_of_served)),
      { text: ")." },
    ], { x: 0.55, y: 4.3, w: 8.9, h: 0.32, fontSize: 11, color: GREY, fontFace: FF });
    source(s, `Source: tools/replay_pipeline.py, demo/out/identity_replay/pipeline.json and pipeline.md (${nf(RP.ticks)} ticks).`);
    H.badge(s, page);
  }

  /* 07 · BEFORE AND AFTER -------------------------------------------------- */
  {
    const s = content("Same mission, same car", "Before and After");
    const bw = 4.3, bh = bw / beforeImg.aspect, by = 1.62;
    H.figure(s, beforeImg, 0.55, by, bw, bh);
    H.figure(s, afterImg, 5.15, by, bw, bh);
    s.addText([
      { text: `Before · ${BEFORE_DATE_SHORT}`, options: { bold: true, color: INK, breakLine: true } },
      { text: `TARGET LOCKED on a pedestrian signal, car ${BEFORE_SEP_M} m away`, options: { color: GREY } },
    ], { x: 0.55, y: by + bh + 0.06, w: bw, h: 0.56, fontSize: 10.5, fontFace: FF, valign: "top" });
    s.addText([
      { text: `After · 30 Sept (${short(NEW[0].tag)})`, options: { bold: true, color: INK, breakLine: true } },
      { text: `TARGET LOCKED on the red car, ${AFTER_SEP_M} m, centred`, options: { color: GREY } },
    ], { x: 5.15, y: by + bh + 0.06, w: bw, h: 0.56, fontSize: 10.5, fontFace: FF, valign: "top" });
    // Measured differently: "before" is the 1d09786 estimator replayed on each
    // old flight's boxes (pipeline.json old_as_flown), "after" is in flight.
    s.addText(`Estimate on the car: ${Math.round(oldOnLo * 100)}-${Math.round(oldOnHi * 100)} % before (replayed, ${OLD_ARM_COMMIT} estimator), ${pctN(newOnLo)}-${pctN(newOnHi)} % after (flown).`, {
      x: 0.55, y: 4.64, w: 8.9, h: 0.3, fontSize: 11, color: TEAL, fontFace: FF, bold: true,
    });
    source(s, "Source: frames demo/out/citylife_redcar_trail/view/fpv/03600.jpg, citylife_redcar_id1/view/fpv/03000.jpg.");
    H.badge(s, page);
  }

  /* 08 · FOUR FLIGHTS WITH IDENTITY ---------------------------------------- */
  {
    const s = content("Flown 30 September", "Four Flights with Identity");
    // progress_0930_charts.py draws this chart at 8.9 x 2.72 in, the size it is
    // placed at, so its 11 pt labels stay 11 pt here.
    const chart = await img(IMG.estChart);
    H.figure(s, chart, 0.55, 1.58, 8.9, 2.72);
    const y = 4.36;
    const strip = [{ text: "Within 30 m:  ", options: { color: GREY, bold: true, fontSize: 11 } }];
    NEW.forEach((r, i) => {
      const bad = r.within_30m < 0.9;
      strip.push({ text: short(r.tag) + " ", options: { color: GREY, fontSize: 11 } });
      strip.push({ text: pct1(r.within_30m), options: { color: bad ? CHARCOAL : TEAL, bold: true, fontSize: 14 } });
      if (i < NEW.length - 1) strip.push({ text: "   ·   ", options: { color: GREY, fontSize: 11 } });
    });
    s.addText(strip, { x: 0.55, y, w: 8.9, h: 0.34, fontFace: FF, valign: "middle", align: "left" });
    const land = notLandable.length
      ? `all landed on pavement except ${notLandable.join(", ")} (${landShortCm} cm short of clearance)`
      : "all landed on pavement";
    s.addText(`P0 ${p0Max} · ${collisions === 0 ? "no mission collision" : collisions + " mission collisions"} · ${land}`, {
      x: 0.55, y: y + 0.34, w: 8.9, h: 0.28, fontSize: 10.5, color: INK, fontFace: FF, align: "left",
    });
    source(s, "Source: demo/out/citylife_redcar_id1..4/metrics.json; before: pipeline.json old_as_flown (replayed).");
    H.badge(s, page);
  }

  /* 09 · WHAT ID3 TAUGHT US ------------------------------------------------ */
  {
    const s = content("One flight lost the car", "What id3 Taught Us");
    const cw = (8.9 - 2 * 0.3) / 3, ch = 1.42, y0 = 1.62;
    const tl = [
      [`t = ${Math.round(id3Lapse.t)} s`, `Car nearly stopped; estimate lapsed. Re-acquired in ${id3Reacq.after_s} s at ${id3Reacq.rng_h} m.`],
      [`t = ${ID3_TURN} s`, "Sharp turn under the drone; car left the frame."],
      ["After", `Car stood at a light ${ID3_LIGHT_M} m away, in view. ${id3.refused_by["too far"]} boxes refused 'too far' (> ${REACQ_MAX_M} m).`],
    ];
    tl.forEach(([t, b], i) => {
      const x = 0.55 + i * (cw + 0.3);
      H.card(s, x, y0, cw, ch, null, t, b, 10);
      if (i < tl.length - 1) arrowR(s, x + cw + 0.04, y0 + ch / 2, x + cw + 0.26);
    });
    const fw = 4.35, fh = 1.3, fy = 3.24;
    H.card(s, 0.55, fy, fw, fh, ic.lead, "Far lead",
      `Replayed on id3: fires at t = ${FAR_LEAD_T_S} s, ${FAR_LEAD_OFF_M} m from the car. Not yet flown (${short(id4.tag)} never lost the car).`, 10.5);
    H.card(s, 5.1, fy, fw, fh, ic.land, "Landing margin",
      `id3 touched down ${landGot} m from an obstacle (${r1(landNeed)} needed). Sites now keep +${SITE_MARGIN_M} m; ${short(id4.tag)} ${id4.landing_final_landable ? "landable" : "not landable"}.`, 10.5);
    source(s, "Source: demo/out/citylife_redcar_id3/metrics.json; docs/WORKLOG.md 30 Sept, replay of id3 candidates.");
    H.badge(s, page);
  }

  /* 10 · A CITY THAT BEHAVES ----------------------------------------------- */
  {
    const s = content("CityLife level", "A City That Behaves");
    const jn = await img(IMG.junction);
    const f = H.figure(s, jn, 0.55, 1.66, 5.6, 5.6 / jn.aspect);
    // The lit lamp is ~5 px in a 1600 px render: circle it. Its centre is pixel
    // (907, 141) of the 1343 x 548 image. Junction (4100, 4100) has only B heads
    // (pedestrian, docs/data/citylife_signals.json), and no loop junction has a
    // vehicle head (FINDING-crowd "Still open"; WORKLOG 0 of 13).
    const LAMP = [907 / 1343, 141 / 548], LAMP_D = 0.3;
    s.addShape(pres.shapes.OVAL, {
      x: f.x + LAMP[0] * f.w - LAMP_D / 2, y: f.y + LAMP[1] * f.h - LAMP_D / 2, w: LAMP_D, h: LAMP_D,
      fill: { type: "none" }, line: { color: TEAL, width: 1.5 },
    });
    s.addText("Red pedestrian lamp (circled); cars stop behind the zebra by the plan (no vehicle heads at loop junctions); a figure waits.", {
      x: 0.55, y: f.y + f.h + 0.06, w: 5.6, h: 0.5, fontSize: 10.5, color: GREY, fontFace: FF, italic: true, valign: "top",
    });
    const sx = 6.4, sw = 3.05, dy = 0.8;
    const rows = [
      [`${SIM.lamps_driven} of ${P.signals.heads_surveyed}`, `signal heads switch\nfixed-time plan, ${SIGNAL_CYCLE_S} s cycle`],
      [`${SIM.cars_red_violations} · ${SIM.cars_zebra_wait}`, `red-light violations · stops on the zebra\nin ${simMin(simLongS)} min of Simulate`],
      [`${SIM.before_fix_figures} · ${Math.max(...SIM.after_fix_crossings)}`, `pedestrians on routes · crossings\nin ${simMin(simLongS)} min`],
      [String(CARS), `cars on ${LOOPS} keep-left loops\none red car`],
    ];
    rows.forEach(([v, l], i) => {
      const y = 1.62 + i * dy;
      s.addText(v, { x: sx, y, w: sw, h: 0.36, fontSize: 20, color: TEAL, fontFace: FF, bold: true, valign: "middle" });
      s.addText(l, { x: sx, y: y + 0.36, w: sw, h: 0.4, fontSize: 10, color: GREY, fontFace: FF, valign: "top", lineSpacingMultiple: 1.05 });
      if (i < rows.length - 1) s.addShape(pres.shapes.LINE, { x: sx, y: y + dy - 0.02, w: sw - 0.1, h: 0, line: { color: LINE, width: 1 } });
    });
    source(s, "Source: docs/data/citylife_signals.json; Simulate counters (verify_drive/signals/peds), docs/WORKLOG.md 30 Sept.");
    H.badge(s, page);
  }

  /* 11 · CHECKS THAT COULD NOT FAIL ---------------------------------------- */
  {
    const s = content("Measuring the city", "Checks That Could Not Fail");
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.22, y0 = 1.66;
    [
      "Zebra-wait counter sat in a branch it could never reach",
      "Lamp check read only the lit slot",
      "Zebra check compared positions, not state",
    ].forEach((b, i) => tagCard(s, 0.55 + i * (cw + 0.2), y0, cw, ch, ic.clip, "Loose check", b, 10.5));
    const bw = 4.35, bh = 1.45, by = 3.1;
    H.card(s, 0.55, by, bw, bh, ic.bug, `${SIM.zebra_figures_before_nkind} figures on zebras in WALK`,
      "A Blueprint pure node read after its index moved. Fixed (NKind latch).", 10.5);
    H.card(s, 5.1, by, bw, bh, ic.bug, `${SIM.before_fix_off_route} of ${SIM.before_fix_figures} figures lost after ${simMin(SIM.before_fix_t_s)} min`,
      `Stuck rule skipped nodes into buildings. Now: detour, never skip. ${Math.max(...SIM.after_fix_off_route)} of ${SIM.before_fix_figures} lost over ${simMin(simLongS)} min.`, 10.5);
    source(s, "Source: docs/FINDING-crowd-pedestrians-and-traffic.md, \"Re-measured, 2026-09-30\".");
    H.badge(s, page);
  }

  /* 12 · NUMBERS WE CORRECTED ---------------------------------------------- */
  {
    const s = content("Retractions since 16 September", "Numbers We Corrected");
    const R = RETRACTED;
    const rows = [
      ["Claim", "Correct"],
      [`Detector rate ${R.detOld} Hz (red car)`, `${R.detNew} Hz over the mission; start-gate inferences were counted`],
      [`Mid-eval report: detector ${R.midEvalDet}`, `${R.midEvalDetNew} Hz over the mission (correction note of 6 Oct, row A14)`],
      ["Identity gate passed", `Held out ${pctN(IG.held_out.on_notok_rate)} % vs ${pct0(IG.gate.on_notok_max)} gate: near miss`],
      // "to", not an arrow: Poppins has no U+2192 and a table cell is one run.
      [`Replay ${R.replayOldOn} to ${R.replayOldNew} %, ${R.replayOldSeeds} vs ${R.replayOldWrong}`,
        `Against the ${OLD_ARM_COMMIT} estimator: ${pctN(RP.as_flown_on_car_of_served)} to ${pctN(RP.new_on_car_of_served)} %, ${RP.new_false_seeds}/${RP.new_seeds} vs ${RP.as_flown_false_post_gap}/${RP.as_flown_post_gap}`],
      ["Tracking score (linear projection)", `Pinhole: ${R.pinholeBelowChance} more flights at or below chance`],
      ["\"Nobody standing on a zebra\"", `Check was loose; ${SIM.zebra_figures_before_nkind} figures found`],  // retracted claim, quoted (CHANGELOG 2026-09-29)
    ];
    H.table(s, rows, { y: 1.66, rh: 0.44, fontSize: 10, colFrac: [0.4, 0.6], align: ["left", "left"] });
    source(s, "Source: CHANGELOG.md [Unreleased], 2026-09-29 and 2026-09-30.");
    H.badge(s, page);
  }

  /* 13 · AGAINST THE CONTRACT ---------------------------------------------- */
  {
    const s = content("ITRI subcontract, WP1-WP4", "Against the Contract");
    // Status text: demo/out/ros2_*/kpi.json (P0 0.0 on every shielded dev-topology
    // run, stored under the old label canonical-hil; mean time to safe only on the unshielded ros2_shield_off, the shielded
    // runs have 0 unsafe episodes; no .replay bundle in any ros2_* folder),
    // docs/DESIGN-hil-perception-bridge.md ("scoping only"), git log since 09-16.
    // WP3: no FENCE_ENABLE / FENCE_ACTION anywhere in sitl/ or guardrail/.
    // WP4: no paraphraser in the repo (the only hit, demo/vla_bridge.py, says
    // strings are NOT paraphrased); per-paraphrase robustness is a grant KPI.
    // Gazebo: the grant's mid-term gate (2026-07-20) names the functional rail
    // (Gazebo Harmonic); the sources disagree on whether it ever ran, so the
    // slide states only what is checkable: the launch scripts exist
    // (sitl/setup_gazebo.sh, sitl/run_gazebo_demo.sh), no run output is kept.
    // Rows corrected 2026-10-06 (the delivered 30 Sept deck is not rebuilt):
    // WP1/WP2 "Built", WP3 "KPI-grade (canonical-hil)", "Measured", and Gazebo
    // "no artefacts" were withdrawn (CHANGELOG.md, Retracted).
    const kf = SWEEP._counts.known_failure;
    const rows = [
      ["Item", "Status", "Since 16 Sept"],
      ["WP1 Policy DSL + bundle", "Partial", "One new policy"],
      ["WP2 Prefix compiler", "Partial: CSP saved, no VLA reads it", "Token budget, coverage not measured"],
      ["WP3 Safety Shield", `Shield core; 5 dev-topology runs (${KPI_DATE})`, "Off-map fix; KPIs not re-flown; ArduPilot fence backstop not configured"],
      ["WP4 Stress testing", `${sweepTotal} ${SWEEP._backend}: ${SWEEP._counts.pass} pass, ${kf} known failure${kf === 1 ? "" : "s"}`,
        `No new scenario since ${SWEEP_DATE}; no paraphraser or per-paraphrase KPI`],
      ["Five acceptance KPIs", "Computed on dev runs, P0 0.0; time to safe on control only", `Evidence ${KPI_AGE_D} days old, no replay bundles`],
      ["Gazebo rail (mid-term gate)", "Open; scripts exist, no run output kept", `Not touched; asked ${GAZEBO_ASKED}`],
      ["Perception-rail integration", "Not started", "Final delivery item"],
    ];
    const tb = tableV(s, rows, {
      y: 1.58, fontSize: 10, colFrac: [0.28, 0.33, 0.39], rhs: [0.32, 0.3, 0.3, 0.42, 0.42, 0.42, 0.3, 0.3],
      emph: [[3, 1]], align: ["left", "center", "left"],
    });
    // docs research: verdict PARTIALLY DRIFTING. Of the post-09-16 work only the
    // scene items were PI requests (09-16 minutes); traffic signals, search,
    // landing and trail are outside the contract, identity was our own 09-29 ask.
    s.addText("Shield core unchanged since 16 Sept. The PI asked for the 16 Sept scene items; signals, identity, search and landing were our own additions. Partial drift: perception-rail integration has not started.", {
      x: 0.55, y: tb + 0.06, w: 8.9, h: 0.42, fontSize: 11, color: TEAL, fontFace: FF, bold: true, valign: "top", lineSpacingMultiple: 1.0,
    });
    source(s, "Source: reference/Grant overview (PDF); demo/out/ros2_*/kpi.json; docs/data/scenario_sweep.json; mid-eval report s.7.");
    H.badge(s, page);
  }

  /* 14 · DECISIONS NEEDED -------------------------------------------------- */
  {
    const s = content("From Prof. Lai", "Decisions Needed");
    steps(s, [
      ["HIL perception bridge", "Option A: AirSim + MAVROS, new label.\nOption B: HIL_GPS/HIL_SENSOR, EKF on AirSim sensors."],
      ["What counts as 'hil'", "Our desktop SITL + MAVROS 2 rail is the grant's dev topology; hil puts VLA + Shield on a Jetson Orin."],
      ["Final-demo priorities", "Which scenarios first: NFZ in the city, crowd, intersection, square route?"],
    ], { y0: 1.72, dy: 1.04, rowH: 0.86, tw: 3.2, bx: 4.45, bw: 4.95, bodySize: 11.5 });
    H.badge(s, page);
  }

  /* 15 · NEXT STEPS -------------------------------------------------------- */
  {
    const s = content("Plan", "Next Steps");
    steps(s, [
      ["Commit and tag phase 3", `${SIM.suite_tests} tests green; flights still marked '-dirty'.`],
      ["Re-fly the five KPI runs at HEAD", "dev topology now, with replay bundles; hil on the Jetson Orin."],
      ["Show the Shield in the city", "NFZ over one CityLife junction, red-car mission through it."],
      ["Start the HIL bridge", "After the topology decision."],
      ["Person range from the ground ray", "Then identity for people."],
      ["Correct the mid-eval numbers", "Detector rate; 'not yet flown' items."],
    ], { y0: 1.62, dy: 0.55, rowH: 0.5, tw: 3.4, bx: 4.7, bw: 4.7, bodySize: 10.5 });
    H.badge(s, page);
  }

  /* 16 · THANK YOU --------------------------------------------------------- */
  {
    const s = pres.addSlide(); page++;
    H.darkBase(s, null);
    s.addText("Thank You", { x: 0.55, y: 2.1, w: 9, h: 1.2, fontSize: 48, color: WHITE, fontFace: FF, bold: true });
    s.addShape(pres.shapes.LINE, { x: 0.6, y: 3.35, w: 1.2, h: 0, line: { color: TEAL, width: 3 } });
  }

  /* 17 · LIMITS (backup) --------------------------------------------------- */
  {
    const s = content("Backup", "Limits");
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.42;
    const items = [
      [ic.lead, "Far lead", "Only in replay; not yet flown."],
      // Once on the car; citylife_ped_id also logged two, on people (unreliable).
      [ic.undo, "Re-acquisition", `Flown once on the car (${id3Reacq.after_s} s, ${short(id3.tag)}).`],
      [ic.people, "People", "Depth hits the facade; identity off for people."],
      [ic.speed, "Detector rate", `${detRange} Hz in flight; gate is ${r1(DET_GATE_HZ)}.`],
      [ic.video, "Demo_day + identity", `${pct1(demoId.within_30m)} within 30 m (${r1(demoAcq.t)} s to acquire; no start gate).`],
      [ic.flask, "Scope of evidence", `${NEW.length} flights, one red car, Project AirSim (not KPI-grade).`],
    ];
    // card() geometry with a 12 pt title in a 2.10 in box (card() uses 13 pt):
    // "Demo_day + identity" is 1.96 in at 13 pt and would wrap; at 12 pt it
    // is 1.81 in, leaving 0.09 in of the 1.90 in text width.
    items.forEach(([icon, t, b], i) => {
      const x = 0.55 + (i % 3) * (cw + 0.2), y = 1.66 + Math.floor(i / 3) * (ch + 0.2);
      cardBox(s, x, y, cw, ch);
      s.addImage({ data: icon, x: x + 0.22, y: y + 0.22, w: 0.36, h: 0.36 });
      s.addText(t, { x: x + 0.68, y: y + 0.2, w: 2.1, h: 0.4, fontSize: 12, color: INK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(b, { x: x + 0.24, y: y + 0.72, w: cw - 0.48, h: ch - 0.92, fontSize: 10.5, color: GREY, fontFace: FF, lineSpacingMultiple: 1.25, valign: "top" });
    });
    H.badge(s, page);
  }

  /* 18 · PEDESTRIAN MISSION (backup) --------------------------------------- */
  {
    const s = content("Backup", "Pedestrian Mission");
    s.addText("Presence pipeline · citylife_ped_0930", { x: 0.55, y: 1.6, w: 8.9, h: 0.3, fontSize: 11, color: INK, fontFace: FF, bold: true });
    // For a class subject the separation is to the NEAREST of the level's
    // figures (demo/follow_vlm.py _sep), so "within 30 m" is not tracking one
    // person; the estimate was on a person on 8.4 % of its ticks.
    const y = 1.92, sz = 22;
    H.stat(s, 0.55, y, 1.85, pct1(ped.within_30m), `within 30 m of the nearest of ${PED_TRUTH_FIGURES} figures`, TEAL, sz);
    H.stat(s, 2.45, y, 1.75, pct1(ped.estimate_on_subject), "estimate within 6 m of a person", CHARCOAL, sz);
    H.stat(s, 4.25, y, 1.35, `${r1(ped.sep_min_m)} m`, "closest figure", TEAL, sz);
    H.stat(s, 5.65, y, 1.35, r1(ped.p0), "P0 escape rate", TEAL, sz);
    H.stat(s, 7.05, y, 2.4, `${nf(ped.presence_blocked_ticks)} / ${nf(PED_TICKS)}`, "ticks blocked by the presence gate", CHARCOAL, sz);
    // metrics.json standoff_score: the ring against ground truth. It binds to
    // the served estimate, so P0 0.0 does not mean no figure came within 10 m.
    const ring = PED_M.standoff_score["standoff-pedestrian"];
    s.addText(`The ${r1(ring.min_range_m).replace(/\.0$/, "")} m ring binds only the tracked estimate: a figure was inside it on ${nf(ring.fn + ring.tp)} ticks; the ring fired ${ring.fires} times.`, {
      x: 0.55, y: 3.08, w: 8.9, h: 0.32, fontSize: 11, color: TEAL, fontFace: FF, bold: true, valign: "middle",
    });
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.48, cy = 3.45;
    H.card(s, 0.55, cy, cw, ch, ic.people, "With identity",
      `ped_id: estimate within 6 m of anybody ${pctN(pedId.estimate_on_subject)} %; facades ranged ${FACADE_RANGE_M} m.`, 10);
    H.card(s, 0.55 + cw + 0.2, cy, cw, ch, ic.ok, "Decision", "Runner uses identity for the car mission only.", 10);
    H.card(s, 0.55 + 2 * (cw + 0.2), cy, cw, ch, ic.ask, "Open", "Range from the nearest surface or the ground ray.", 10);
    source(s, "Source: demo/out/citylife_ped_0930/metrics.json (standoff_score), citylife_ped_id/metrics.json; CHANGELOG.md 30 Sept.");
    H.badge(s, page);
  }

  /* 19 · WITHIN 30 M, PER FLIGHT (backup) ---------------------------------- */
  {
    const s = content("Backup", "Within 30 m, Per Flight");
    const chart = await img(IMG.w30Chart);
    H.figure(s, chart, 0.55, 1.58, 8.9, 3.3);   // drawn at 8.9 x 3.3 in
    source(s, "Source: demo/out/citylife_redcar_*/metrics.json frac_within_30m; tools/deck/progress_0930_charts.py.");
    H.badge(s, page);
  }

  /* 20 · CITY STILL OPEN (backup) ------------------------------------------ */
  {
    const s = content("Backup", "City Still Open");
    const cw = 4.35, ch = 1.45, x0 = 0.55, y0 = 1.66;
    H.card(s, x0, y0, cw, ch, ic.wait, "Kerb waits",
      `Up to ${Math.round(SIM.kerb_wait_max_s)} s at loop A's turning zebra; cars do not yield to waiting figures.`, 10.5);
    H.card(s, x0 + cw + 0.2, y0, cw, ch, ic.light, "Vehicle signal heads",
      `${LOOP_JUNCTIONS_WITH_VEH_HEADS} of ${LOOP_JUNCTIONS} loop junctions have them.`, 10.5);
    H.card(s, x0, y0 + ch + 0.2, cw, ch, ic.road, "Zebras", `Only ${USABLE_ZEBRAS} usable.`, 10.5);
    H.card(s, x0 + cw + 0.2, y0 + ch + 0.2, cw, ch, ic.route, "Detours",
      `${nf(Math.max(...SIM.after_fix_detours))} in ${simMin(simLongS)} min; ${Math.max(...SIM.after_fix_teleports)} set-downs.`, 10.5);
    source(s, "Source: docs/FINDING-crowd-pedestrians-and-traffic.md; CHANGELOG.md 30 Sept, Known limitations.");
    H.badge(s, page);
  }

  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  await embedFonts(OUT);
  const mb = fs.statSync(OUT).size / 1048576;
  console.log(`wrote ${OUT}`);
  console.log(`slides: ${page}`);
  console.log(`size  : ${mb.toFixed(2)} MB`);
  if (mb > MAX_MB) {
    console.error(`REFUSED: deck exceeds ${MAX_MB} MB. Something is embedding media.`);
    process.exit(1);
  }
}

/* Embed the fonts. Poppins is installed per user on this PC only, and
 * pptxgenjs cannot embed fonts, so a meeting-room PC without it would reflow
 * every slide in a fallback font. PowerPoint re-saves the audited deck with
 * EmbedTrueTypeFonts (SaveAs 24 = ppSaveAsOpenXMLPresentation, -1 = msoTrue),
 * which embeds the characters used; then the result is checked and re-audited.
 * PowerPoint is already required for the PDF (tools/office_to_pdf.ps1). */
async function embedFonts(pptx) {
  const { execFileSync } = require("child_process");
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
  // A COM save can report success and write nothing: check the file itself.
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
