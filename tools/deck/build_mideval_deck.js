/*
 * Guardrail mid-evaluation, June to September 2026 — nathan-deck design system.
 * For the 18 September 2026 sharing meeting (~15 minutes).
 *
 * Every number comes from docs/data/eval_sep2026.json (tools/build_eval_data.py).
 *
 * CORRECTED 2026-10-06, in the generator only (the 16 Sept deck was delivered;
 * its corrections are rows B1-B10 of
 * docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md). A rebuild can no
 * longer print: "Hard KPI" (P0 escape is one of five acceptance KPIs, B1);
 * WP1/WP2/WP4 "Built" or WP3 "canonical-HIL runs" (B2); "rules into prompt"
 * and the swappable-source box (B3); "Measured on the KPI rail" (B4); the
 * contractual-gate card (B5); the "Canonical HIL" header and the 1.0
 * fail-safe target, which is >= 0.99 (B6); the inflated detector range (the
 * numbers file now carries the mission rate, B7); the KPI-rail / Gazebo plan
 * items (B8, B9); "KPI gate cleared" / "All five KPIs" on the timeline and the
 * conclusion. The desktop SITL + MAVROS 2 rail is the grant's `dev` topology;
 * reported KPI figures come from Stress Testing runs in `hil` (Jetson Orin).
 * tools/check_claims.py scans this file for the retracted wordings.
 *
 *   node tools/deck/build_mideval_deck.js
 */
const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const sharp = require("sharp");
const {
  FaRobot, FaBan, FaShieldAlt, FaDrawPolygon, FaArrowsAltV, FaTachometerAlt, FaBuilding,
  FaWalking, FaRoute, FaCheckCircle, FaEyeSlash, FaFlask, FaVideo, FaExclamationTriangle,
  FaHourglassHalf, FaBalanceScale, FaFileSignature, FaHistory, FaClipboardCheck, FaCube,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "..", "..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const E = JSON.parse(fs.readFileSync(path.join(REPO, "docs/data/eval_sep2026.json"), "utf8"));
const OUT = path.join(REPO, "docs/Guardrail-MidEvaluation-Sep2026.pptx");
const FPV = path.join(REPO, "demo/out/retarget_smooth/view/fpv");

// Desktop SITL + MAVROS 2 runs under today's name ("dev"); a numbers file
// written before 2026-10-06 stored them as "canonical-hil".
const nDev = (E.rails.counts["dev"] || 0) + (E.rails.counts["canonical-hil"] || 0);
const cityFlown = Object.keys(((E.unflown || {}).citylife_level || {}).flights || {}).length;
// The 768x432 camera's flights, read off the same artefact list as the
// report's Section 7, so a "next step" row cannot ask to fly what has flown.
const camFlown = (((E.unflown || {}).camera_768x432 || {}).flight_artefacts || []).length;
// "0 of 34 (33 measurable)", as the correction note states it (row A15).
const camAll = E.rails.camera_flights_all ?? E.rails.camera_flights;
const f1 = (v) => Number(v).toFixed(1);
const f2 = (v) => Number(v).toFixed(2);

async function frame(t, crop) {
  const file = path.join(FPV, String(Math.round(t * 20)).padStart(5, "0") + ".jpg");
  let p = sharp(fs.readFileSync(file));
  if (crop) p = p.extract(crop);
  const buf = await p.jpeg({ quality: 92 }).toBuffer();
  const m = await sharp(buf).metadata();
  return { data: "image/jpeg;base64," + buf.toString("base64"), aspect: m.width / m.height };
}

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "Guardrail — Mid-Evaluation, September 2026";
  const H = helpers(pres);
  const { TEAL, TEAL_DK, TEAL_TINT, TEAL_TINT2, WHITE, BLACK, INK, GREY, LGREY, LINE, FF, CHARCOAL } = T;

  const ic = {
    robot: await iconPng(FaRobot), ban: await iconPng(FaBan), shield: await iconPng(FaShieldAlt),
    poly: await iconPng(FaDrawPolygon), alt: await iconPng(FaArrowsAltV), speed: await iconPng(FaTachometerAlt),
    bld: await iconPng(FaBuilding), walk: await iconPng(FaWalking), route: await iconPng(FaRoute),
    ok: await iconPng(FaCheckCircle), blind: await iconPng(FaEyeSlash), flask: await iconPng(FaFlask),
    video: await iconPng(FaVideo), warn: await iconPng(FaExclamationTriangle), wait: await iconPng(FaHourglassHalf),
    scale: await iconPng(FaBalanceScale), sign: await iconPng(FaFileSignature), hist: await iconPng(FaHistory),
    clip: await iconPng(FaClipboardCheck), cube: await iconPng(FaCube),
  };

  const K = E.kpi.runs, R = E.retarget, D = E.detector, L = E.lock, S = E.sweep, G = E.repo, U = E.unflown;
  let page = 1;
  const content = (eyebrow, title) => {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, eyebrow, title);
    return s;
  };

  /* Diagram primitives: a labelled box and a straight arrow. Arrows are only
   * drawn left-to-right or top-to-bottom, so no shape gets a negative extent
   * (which is one of the defects tools/audit_pptx.py rejects). */
  function box(s, x, y, w, h, title, sub, { fill = WHITE, border = TEAL, titleColor = INK, dashed = false } = {}) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x, y, w, h, fill: { color: fill }, rectRadius: 0.06,
      line: { color: border, width: 1.25, dashType: dashed ? "dash" : "solid" },
    });
    s.addText(title, { x: x + 0.08, y: y + 0.06, w: w - 0.16, h: 0.34, fontSize: 11, color: titleColor, fontFace: FF, bold: true, align: "center", valign: "middle" });
    if (sub) s.addText(sub, { x: x + 0.08, y: y + 0.38, w: w - 0.16, h: h - 0.44, fontSize: 9, color: GREY, fontFace: FF, align: "center", valign: "top", lineSpacingMultiple: 1.1 });
  }
  function arrowR(s, x1, y, x2) {
    s.addShape(pres.shapes.LINE, { x: x1, y, w: x2 - x1, h: 0, line: { color: CHARCOAL, width: 1.25, endArrowType: "triangle" } });
  }
  function arrowD(s, x, y1, y2) {
    s.addShape(pres.shapes.LINE, { x, y: y1, w: 0, h: y2 - y1, line: { color: CHARCOAL, width: 1.25, endArrowType: "triangle" } });
  }

  /* 01 · COVER ------------------------------------------------------------- */
  {
    const s = pres.addSlide();
    H.darkBase(s, "ITRI VLA-UAV · Mid-Evaluation · September 2026");
    s.addText("Guardrail\nMid-Evaluation", {
      x: 0.55, y: 1.15, w: 8.4, h: 1.9, fontSize: 40, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1,
    });
    s.addText("A safety layer for language-commanded ArduPilot UAVs", {
      x: 0.55, y: 3.15, w: 8.4, h: 0.6, fontSize: 15, color: TEAL_TINT, fontFace: FF,
    });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.15, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    s.addText([
      { text: "Department of International Graduate Program, EECS\n" },
      { text: "Advisor: \n" },
      { text: "Presenter: " },
    ], { x: 0.55, y: 4.3, w: 8, h: 1.0, fontSize: 11, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3 });
  }

  /* 02 · THE PROBLEM ------------------------------------------------------- */
  {
    const s = content("Motivation", "The Problem");
    const cw = (8.9 - 2 * 0.25) / 3, cy = 1.85, ch = 1.75;
    H.card(s, 0.55, cy, cw, ch, ic.robot, "Model drives", "A VLA turns a phrase into velocity.");
    H.card(s, 0.55 + cw + 0.25, cy, cw, ch, ic.ban, "No rules inside", "Nothing in the model knows a no-fly zone.");
    H.card(s, 0.55 + 2 * (cw + 0.25), cy, cw, ch, ic.shield, "Guardrail", "Declared policy, enforced on every command.");
    s.addText("P0 violation escape rate, target 0, is one of five acceptance KPIs.", { x: 0.55, y: 3.9, w: 8.6, h: 0.35, fontSize: 12, color: TEAL, fontFace: FF, bold: true });
    H.badge(s, page);
  }

  /* 03 · GRANT SCOPE ------------------------------------------------------- */
  {
    const s = content("Grant scope", "Work Packages");
    const rows = [
      ["WP", "Deliverable", "Status", "Evidence"],
      ["WP1", "Policy DSL + IR", "Partial", "6 constraint types, bundle signed with a lab dev key"],
      ["WP2", "Prefix constraint compiler", "Partial", "CSP generated and saved; no flown VLA reads it yet"],
      ["WP3", "Suffix Safety Shield", "Partial", `Shield core; ${nDev} SITL + MAVROS 2 runs (dev topology)`],
      ["WP4", "Stress testing", "Partial", `${S.counts.pass + S.counts.fail + S.counts.known_failure}-scenario headless regression sweep`],
    ];
    H.table(s, rows, { y: 1.8, rh: 0.5, colFrac: [0.12, 0.3, 0.16, 0.42], emph: [[3, 2]], align: ["center", "left", "center", "left"] });
    s.addText("Five acceptance KPIs, all named in the grant: mission success · P0 escape rate · fail-safe correctness · mean repair magnitude · mean time to safe.", {
      x: 0.55, y: 4.45, w: 8.9, h: 0.5, fontSize: 10.5, color: GREY, fontFace: FF, lineSpacingMultiple: 1.25,
    });
    H.badge(s, page);
  }

  /* 04 · ARCHITECTURE ------------------------------------------------------ */
  {
    const s = content("System", "Architecture");
    const bw = 1.62, gap = 0.2, y = 1.75, bh = 1.0;
    const xs = [0, 1, 2, 3, 4].map((i) => 0.55 + i * (bw + gap));
    box(s, xs[0], y, bw, bh, "Camera", "frame + phrase");
    box(s, xs[1], y, bw, bh, "Detector + lock", "OWL-ViT, one instance");
    box(s, xs[2], y, bw, bh, "Estimator", "range + bearing → action");
    box(s, xs[3], y, bw, bh, "Safety Shield", "WP3 · check, repair, audit", { fill: TEAL_TINT, border: TEAL_DK, titleColor: TEAL_DK });
    box(s, xs[4], y, bw, bh, "MAVROS 2", "→ ArduPilot");
    for (let i = 0; i < 4; i++) arrowR(s, xs[i] + bw, y + bh / 2, xs[i + 1]);
    s.addText("Action4D · 10 Hz", { x: xs[2] + bw - 0.2, y: y - 0.3, w: 2.0, h: 0.26, fontSize: 8.5, color: TEAL, fontFace: FF, bold: true, align: "center" });

    const y2 = 3.35;
    box(s, xs[3], y2, bw, 0.95, "Policy DSL", "WP1 · hashed YAML");
    box(s, xs[1], y2, bw * 2 + gap, 0.95, "Action source", "test pilot in the city · OpenVLA-7B, AerialVLA in separate runs", { dashed: true, border: LGREY });
    box(s, xs[4], y2, bw, 0.95, "WP4 evidence", "sweep · replay · manifest");
    box(s, xs[0], y2, bw, 0.95, "Prefix compiler", "WP2 · rules summarised into a CSP");
    arrowR(s, xs[0] + bw, y2 + 0.475, xs[1]);
    H.source(s, "The Shield sees only the Action4D. Which model produced it does not change the safety argument.", 4.55);
    H.badge(s, page);
    // Up-arrows (policy → shield, source → estimator row) drawn as lines from the lower row's top edge.
    s.addShape(pres.shapes.LINE, { x: xs[3] + bw / 2, y: y + bh, w: 0, h: y2 - (y + bh), line: { color: CHARCOAL, width: 1.25, beginArrowType: "triangle" } });
    s.addShape(pres.shapes.LINE, { x: xs[2] + bw / 2 - 0.4, y: y + bh, w: 0, h: y2 - (y + bh), line: { color: LGREY, width: 1.25, beginArrowType: "triangle", dashType: "dash" } });
  }

  /* 05 · POLICY DSL -------------------------------------------------------- */
  {
    const s = content("WP1", "Six Constraint Types");
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.32;
    const items = [
      [ic.poly, "Polygon fence", "Keep-out zone, optional altitude band."],
      [ic.alt, "Altitude envelope", "Floor and ceiling above ground."],
      [ic.speed, "Kinematic caps", "Speed, climb and yaw-rate limits."],
      [ic.bld, "Clearance", "Distance from every mapped building."],
      [ic.walk, "Subject stand-off", "Distance from the followed subject, per class."],
      [ic.route, "Corridor", "Keep-in route with width and altitude."],
    ];
    items.forEach(([icon, t, b], i) => {
      H.card(s, 0.55 + (i % 3) * (cw + 0.2), 1.72 + Math.floor(i / 3) * (ch + 0.2), cw, ch, icon, t, b, 10.5);
    });
    s.addText("valid_time on every rule · policy hash in every audit record · signed bundle", {
      x: 0.55, y: 4.72, w: 8.6, h: 0.3, fontSize: 10.5, color: TEAL, fontFace: FF, bold: true,
    });
    H.badge(s, page);
  }

  /* 06 · SAFETY SHIELD ----------------------------------------------------- */
  {
    const s = content("WP3", "How the Shield Decides");
    const steps = [
      ["Predict", "3 s lookahead"],
      ["Check", "every rule"],
      ["Repair", "smallest legal change"],
      ["Re-check", "the repaired action"],
      ["Fall back", "recover or brake"],
    ];
    const bw = 1.62, gap = 0.2, y = 1.85, bh = 0.95;
    steps.forEach(([t, b], i) => {
      const x = 0.55 + i * (bw + gap);
      box(s, x, y, bw, bh, t, b, i === 2 ? { fill: TEAL_TINT, border: TEAL_DK, titleColor: TEAL_DK } : {});
      if (i < steps.length - 1) arrowR(s, x + bw, y + bh / 2, x + bw + gap);
    });
    const on = K.ros2_shield_on;
    H.card(s, 0.55, 3.15, 4.35, 1.45, ic.clip, "Audit record per tick", "Rule, repair, magnitude, policy hash.");
    H.card(s, 5.1, 3.15, 4.35, 1.45, ic.shield, "Measured on the dev-topology runs",
      `Mean repair ${f2(on.mean_repair_magnitude_mps)} m/s (max ${f2(on.max_repair_magnitude_mps)}).`);
    H.badge(s, page);
  }

  /* 07 · WHERE IT RUNS ----------------------------------------------------- */
  {
    const s = content("Simulation rails", "Where It Runs");
    // Counted from each flight's manifest by tools/build_eval_data.py.
    const rows = [
      ["Rail", "Flights", "Camera", "Grant topology"],
      ["Project AirSim (Unreal)", String(E.rails.counts["projectairsim-single-host"] || 0), "Yes", "perception rail"],
      ["ArduPilot SITL · pymavlink", String(E.rails.counts["ardupilot-sitl-pymavlink"] || 0), "No", "desktop, no MAVROS"],
      ["ArduPilot SITL · MAVROS 2 · ROS 2 Jazzy", String(nDev), "No", "dev"],
    ];
    H.table(s, rows, { y: 1.8, rh: 0.52, colFrac: [0.46, 0.16, 0.18, 0.2], emph: [[3, 3]] });
    H.card(s, 0.55, 4.02, 8.9, 1.02, null, "Next",
      "Tracking runs on Project AirSim; the ArduPilot runs are on one desktop (dev). Reported KPIs come from Stress Testing runs in hil (Jetson Orin).");
    H.badge(s, page);
  }

  /* 08 · KPI RESULTS ------------------------------------------------------- */
  {
    const s = content("Dev topology · ArduPilot SITL + MAVROS 2, one desktop", "KPI Results");
    const a = K.ros2_shield_on, b = K.ros2_shield_on_dynamic, c = K.ros2_ped_on, off = K.ros2_shield_off;
    const yes = (v) => (v ? "Yes" : "No");
    const rows = [
      ["KPI", "Target", "No-fly zone", "Dynamic NFZ", "Pedestrian"],
      ["P0 escape rate", "0", f1(a.p0_violation_escape_rate), f1(b.p0_violation_escape_rate), f1(c.p0_violation_escape_rate)],
      ["Fail-safe correctness*", "≥ 0.99", f1(a.failsafe_trigger_correctness), f1(b.failsafe_trigger_correctness), f1(c.failsafe_trigger_correctness)],
      ["Mission success", "—", yes(a.mission_success), yes(b.mission_success), yes(c.mission_success)],
      ["Mean repair (m/s)", "—", f2(a.mean_repair_magnitude_mps), f2(b.mean_repair_magnitude_mps), f2(c.mean_repair_magnitude_mps)],
      ["Unsafe episodes", "—", String(a.time_to_safe_episodes), String(b.time_to_safe_episodes), String(c.time_to_safe_episodes)],
    ];
    H.table(s, rows, { y: 1.72, rh: 0.44, colFrac: [0.3, 0.13, 0.19, 0.19, 0.19], emph: [[1, 2], [1, 3], [1, 4]] });
    s.addText(`Unshielded control: escape rate ${f2(off.p0_violation_escape_rate)}, mean time to safe ${f1(off.mean_time_to_safe_s)} s. Pedestrian position is declared, not detected. *Computed as the share of P0 ticks that did not escape.`, {
      x: 0.55, y: 4.5, w: 8.9, h: 0.45, fontSize: 10.5, color: GREY, fontFace: FF, lineSpacingMultiple: 1.2,
    });
    H.badge(s, page);
  }

  /* 09 · ON VS OFF ---------------------------------------------------------- */
  {
    const s = content("Same mission, shield off and on", "Guardrail On vs Off");
    const offImg = await img(path.join(REPO, "demo/out/ros2_shield_off/trajectory.png"));
    const onImg = await img(path.join(REPO, "demo/out/ros2_shield_on/trajectory.png"));
    const bw = 2.55, bh = 2.55, by = 1.7;
    H.figure(s, offImg, 0.55, by, bw, bh);
    H.figure(s, onImg, 3.3, by, bw, bh);
    s.addText(`Off: ${f1(K.ros2_shield_off.nfz_s)} s inside NFZ`, { x: 0.55, y: by + bh + 0.05, w: bw, h: 0.3, fontSize: 10.5, color: INK, fontFace: FF, bold: true, align: "center" });
    s.addText(`On: ${f1(K.ros2_shield_on.nfz_s)} s inside NFZ`, { x: 3.3, y: by + bh + 0.05, w: bw, h: 0.3, fontSize: 10.5, color: TEAL, fontFace: FF, bold: true, align: "center" });
    const po = K.ros2_ped_off, pn = K.ros2_ped_on;
    s.addText("Pedestrian run, 10 m rule", { x: 6.15, y: 1.7, w: 3.3, h: 0.3, fontSize: 11, color: INK, fontFace: FF, bold: true });
    H.stat(s, 6.15, 2.0, 3.3, `${f2(po.standoff_min_range_m)} → ${f2(pn.standoff_min_range_m)} m`, "closest approach", TEAL, 22);
    H.stat(s, 6.15, 2.95, 3.3, `${f2(po.p0_violation_escape_rate)} → ${f1(pn.p0_violation_escape_rate)}`, "P0 escape rate", TEAL, 22);
    H.stat(s, 6.15, 3.9, 3.3, `${f1(po.standoff_s)} → ${f1(pn.standoff_s)} s`, "time inside the ring", TEAL, 22);
    H.badge(s, page);
  }

  /* 10 · PERCEPTION --------------------------------------------------------- */
  {
    const s = content("Open-vocabulary perception", "What the Camera Rail Shows");
    const ow = D.bench.owlvit, gd = D.bench.gdino;
    const fl = Object.values(D.flights);
    const minmax = (k) => [Math.min(...fl.map((x) => x[k])), Math.max(...fl.map((x) => x[k]))];
    const [dLo, dHi] = minmax("det_hz"), [lLo, lHi] = minmax("loop_hz");
    const rows = [
      ["Measure", "Result", "Against"],
      ["Instance lock, box error", `${f1(L.city_full.box_err_px_median_in_shot)} → ${f1(L.lock_on.box_err_px_median_in_shot)} px`, `null ${f1(L.city_full.null_centre)} → ${f1(L.lock_on.null_centre)} px`],
      ["Detector latency, idle GPU", `${f1(ow.total_ms_median)} ms`, `G-DINO ${f1(gd.total_ms_median)} ms`],
      ["Person score, G-DINO / OWL-ViT", `${f2(D.gdino_over_owlvit_person)}×`, "taxi " + f2(D.gdino_over_owlvit_taxi) + "×"],
      ["Detector rate in flight", `${f2(dLo)}–${f2(dHi)} Hz`, `gate ${f1(D.gate.det_hz_min)} Hz`],
      ["Control loop in flight", `${f2(lLo)}–${f2(lHi)} Hz`, `gate ${f1(D.gate.loop_hz_min)} Hz`],
    ];
    H.table(s, rows, { y: 1.72, rh: 0.46, colFrac: [0.4, 0.3, 0.3], emph: [[1, 1]] });
    H.source(s, "Every tracking score is shown beside a null: a \"detector\" that emits the frame centre.", 4.62);
    H.badge(s, page);
  }

  /* 11 · CLASS-CONDITIONAL SAFETY ------------------------------------------- */
  {
    const s = content("What a tracker cannot do", "Class-Conditional Safety");
    const FF_ = E.retarget.figure_frames;
    const car = await frame(FF_.car.t);
    const ped = await frame(FF_.person.t, { left: 510, top: 77, width: 480, height: 270 });
    const bw = 3.0, bh = 1.69, by = 1.75;
    H.figure(s, car, 0.55, by, bw, bh);
    H.figure(s, ped, 3.75, by, bw, bh);
    s.addText("\"a yellow car\" → 5 m ring", { x: 0.55, y: by + bh + 0.05, w: bw, h: 0.3, fontSize: 10.5, color: INK, fontFace: FF, bold: true, align: "center" });
    s.addText("\"a person\" → 10 m ring", { x: 3.75, y: by + bh + 0.05, w: bw, h: 0.3, fontSize: 10.5, color: INK, fontFace: FF, bold: true, align: "center" });
    const recl = S.results.find((r) => r.id === "standoff-reclassified");
    H.card(s, 7.0, 1.75, 2.45, 1.69, ic.ok, "Sweep", `standoff-reclassified: ${recl.status}.`);
    s.addText("A tracker returns an ID. The policy needs a class to pick the rule.", {
      x: 0.55, y: 4.2, w: 8.9, h: 0.4, fontSize: 12, color: TEAL, fontFace: FF, bold: true,
    });
    H.source(s, `Phrase changed at t+${f1(R.retarget_event.t)} s on retarget_smooth. Same aircraft, policy and hash.`, 4.7);
    H.badge(s, page);
  }

  /* 12 · EVIDENCE DISCIPLINE ------------------------------------------------ */
  {
    const s = content("WP4", "Evidence Discipline");
    const sw = 2.2, tf = G.tests_fast;
    H.stat(s, 0.55, 1.7, sw, `${tf.passed}/${tf.total}`, "fast tests");
    H.stat(s, 0.55 + sw, 1.7, sw, `${G.tests_coverage.passed}/${G.tests_coverage.total}`, "coverage tests");
    H.stat(s, 0.55 + 2 * sw, 1.7, sw, String(G.finding_docs), "finding documents");
    H.stat(s, 0.55 + 3 * sw, 1.7, sw, `${S.counts.pass}/${S.counts.pass + S.counts.known_failure}`, "sweep scenarios pass");
    const cw = 4.35, ch = 0.82;
    H.card(s, 0.55, 3.0, cw, ch, ic.scale, "Every score has a null", null);
    H.card(s, 5.1, 3.0, cw, ch, ic.sign, "Manifest per flight", null);
    H.card(s, 0.55, 3.97, cw, ch, ic.hist, "Replay re-derives KPIs", null);
    H.card(s, 5.1, 3.97, cw, ch, ic.flask, "Retractions recorded", null);
    H.badge(s, page);
  }

  /* 13 · HONEST LIMITS ------------------------------------------------------ */
  {
    const s = content("Open at mid-term", "Honest Limits");
    const cov = R.coverage_after_full, pa = R.after_ped_full;
    const fl = Object.values(D.flights);
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.35;
    const items = [
      [ic.blind, "Bystanders", `${cov.outside_hfov} of ${cov.ticks_person_inside_10m} close ticks outside camera view.`],
      [ic.walk, "Pedestrian detection", `${f1(pa.box_err_px_median)} px vs ${f1(pa.box_err_px_null_centre)} px null.`],
      [ic.video, "Camera rail rates", `${E.rails.camera_flights_meeting_both_gates} of ${camAll} flights (${E.rails.camera_flights} measurable) meet both gates.`],
      [ic.cube, "Tracking not KPI-grade", "SITL has no camera."],
      (cityFlown && camFlown) ? [ic.wait, "CityLife flown", `${cityFlown} flights at 768×432.`]
        : [ic.wait, "Not yet flown", [camFlown ? "" : "768×432 camera", cityFlown ? "" : "CityLife scene"].filter(Boolean).join(", ") + "."],
      [ic.warn, "Known failure", "Subject on the route wedges the mission."],
    ];
    items.forEach(([icon, t, b], i) => {
      H.card(s, 0.55 + (i % 3) * (cw + 0.2), 1.72 + Math.floor(i / 3) * (ch + 0.2), cw, ch, icon, t, b, 10.5);
    });
    H.source(s, "P0 escape rate stays 0.0 in every shielded case; these limit what it can be claimed to cover.", 4.72);
    H.badge(s, page);
  }

  /* 14 · TIMELINE ----------------------------------------------------------- */
  {
    const s = content("June to September 2026", "Timeline");
    const ms = [
      ["23 Jun", "Kickoff"],
      ["3 Jul", "Shield prototype"],
      ["16 Jul", "OpenVLA-7B flown"],
      ["3 Aug", "Clearance rule"],
      ["19 Aug", "Midterm review"],
      ["25 Aug", "SITL + MAVROS 2"],
      ["1 Sep", "Five KPIs computed"],
      ["10 Sep", `v0.5.0 public`],
    ];
    const x0 = 1.15, x1 = 8.85, y = 2.9;
    s.addShape(pres.shapes.LINE, { x: x0, y, w: x1 - x0, h: 0, line: { color: TEAL_TINT2, width: 4 } });
    ms.forEach(([d, t], i) => {
      const x = x0 + (i * (x1 - x0)) / (ms.length - 1);
      s.addShape(pres.shapes.OVAL, { x: x - 0.11, y: y - 0.11, w: 0.22, h: 0.22, fill: { color: TEAL }, line: { color: WHITE, width: 1.5 } });
      const up = i % 2 === 0;
      s.addText(d, { x: x - 0.6, y: up ? y - 0.95 : y + 0.22, w: 1.2, h: 0.3, fontSize: 10, color: TEAL, fontFace: FF, bold: true, align: "center" });
      s.addText(t, { x: x - 0.7, y: up ? y - 0.65 : y + 0.5, w: 1.4, h: 0.45, fontSize: 10, color: INK, fontFace: FF, align: "center", valign: "top" });
    });
    s.addText(`${G.commits_total} commits in this repository since 24 August; earlier work lives in the upstream fork.`, {
      x: 0.55, y: 4.35, w: 8.9, h: 0.4, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    H.badge(s, page);
  }

  /* 15 · PLAN TO COMPLETION ------------------------------------------------- */
  {
    const s = content("Next milestones", "Plan to Completion");
    const steps = [
      ["AirSim imagery into the ArduPilot rail", "Perception-rail integration (final delivery)."],
      ["Cover bystanders", "A rule for any pedestrian, and a wider sensor."],
      [camFlown ? "Evaluate a narrower FOV" : "Fly 768×432, then a narrower FOV", `45° doubles subject size to ${f2(D.pedestrian_patches_by_hfov_16m["45deg"])} patches.`],
      ["Fix the wedge", "Mission must pass a subject on its route."],
      ["Gazebo Harmonic rail", "Mid-term item: scripts exist; keep a run."],
    ];
    steps.forEach(([t, b], i) => {
      const y = 1.72 + i * 0.64;
      s.addShape(pres.shapes.OVAL, { x: 0.6, y: y + 0.07, w: 0.42, h: 0.42, fill: { color: TEAL }, line: { color: TEAL } });
      s.addText(String(i + 1), { x: 0.6, y: y + 0.07, w: 0.42, h: 0.42, fontSize: 12, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(t, { x: 1.2, y, w: 3.9, h: 0.56, fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(b, { x: 5.1, y, w: 4.3, h: 0.56, fontSize: 11, color: GREY, fontFace: FF, valign: "middle" });
      if (i < steps.length - 1) s.addShape(pres.shapes.LINE, { x: 1.2, y: y + 0.6, w: 8.2, h: 0, line: { color: LINE, width: 1 } });
    });
    H.badge(s, page);
  }

  /* 16 · CONCLUSION --------------------------------------------------------- */
  {
    const s = pres.addSlide();
    H.darkBase(s, "Conclusion");
    s.addText("Five acceptance KPIs computed on the dev-topology runs, with P0 escape rate 0.0.", {
      x: 0.55, y: 1.5, w: 8.6, h: 1.7, fontSize: 26, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.15,
    });
    s.addText("Next: the hil topology on a Jetson Orin for the reported KPIs, and Project AirSim imagery into the ArduPilot rail.", {
      x: 0.55, y: 3.4, w: 8.2, h: 0.8, fontSize: 13, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3,
    });
  }

  /* 17 · THANK YOU ---------------------------------------------------------- */
  {
    const s = pres.addSlide();
    H.darkBase(s, null);
    s.addText("Thank You", { x: 0.55, y: 2.1, w: 9, h: 1.2, fontSize: 48, color: WHITE, fontFace: FF, bold: true });
    s.addShape(pres.shapes.LINE, { x: 0.6, y: 3.35, w: 1.2, h: 0, line: { color: TEAL, width: 3 } });
  }

  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  console.log("wrote " + OUT);
}

main().catch((e) => { console.error(e); process.exit(1); });
