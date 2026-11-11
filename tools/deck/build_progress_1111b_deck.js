/*
 * Progress update, 11 Nov 2026, combined deck: the city follow in Project
 * AirSim (28 Oct results, PC and Jetson, with the guardrail and its videos)
 * and the guardrail's KPI rig (AGX Orin + ArduPilot SITL over MAVROS 2).
 * nathan-deck design system (tools/deck/nathan_theme.js).
 *
 * The user's rules for this deck (11 Nov): show the topology as a picture,
 * keep the Project AirSim videos, show what is finished; the KPI scoreboard
 * and the contract status stay in the cheat sheet, not on slides.
 *
 * Numbers are read at build time from docs/data/progress_1028.json
 * (build_progress_1028_data.py) and _build/progress_1111.json
 * (build_progress_1111_data.py). The video clips are cut from the 28 Oct
 * share videos (45 s each) and the 11 Nov rig replay video.
 *
 *   node tools/deck/build_progress_1111b_deck.js
 */
const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const {
  FaCar, FaShieldAlt, FaMicrochip, FaDesktop, FaCheckCircle, FaWind, FaTools, FaFlagCheckered,
  FaVideo, FaBan, FaStopwatch, FaRulerHorizontal, FaRobot,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "../..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const SHARE = path.join(REPO, "docs/share/2026-11-11-Progress");
const BUILD = path.join(SHARE, "_build");
const D28 = JSON.parse(fs.readFileSync(path.join(REPO, "docs/data/progress_1028.json"), "utf8"));
const D11 = JSON.parse(fs.readFileSync(path.join(BUILD, "progress_1111.json"), "utf8")).campaigns;
const OUT = path.join(SHARE, process.env.DECK_NAME || "VLA-Drone-Progress-11Nov2026.pptx");
const { TEAL, TEAL_DK, TEAL_TINT, TEAL_TINT2, INK, GREY, WHITE, LINE, FF } = T;
const ORANGE = "E08A1E";

const f2 = (v) => (v == null ? "-" : Number(v).toFixed(2));
const f3 = (v) => (v == null ? "-" : Number(v).toFixed(3));
const f1 = (v) => Number(v).toFixed(1);
const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
const frac = (r) => `${r.correct}/${r.scored}`;
const TEXT = [];
function say(s, t, o) { TEXT.push(typeof t === "string" ? t : JSON.stringify(t)); s.addText(t, o); }
const jpg = (f) => "image/jpeg;base64," + fs.readFileSync(path.join(BUILD, f)).toString("base64");

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "VLA Drone with Guardrail - progress, 11 Nov 2026";
  const H = helpers(pres);
  const ic = {
    car: await iconPng(FaCar), shield: await iconPng(FaShieldAlt), chip: await iconPng(FaMicrochip),
    desk: await iconPng(FaDesktop), ok: await iconPng(FaCheckCircle), wind: await iconPng(FaWind),
    tools: await iconPng(FaTools), flag: await iconPng(FaFlagCheckered), video: await iconPng(FaVideo),
    ban: await iconPng(FaBan), clock: await iconPng(FaStopwatch), ruler: await iconPng(FaRulerHorizontal),
    robot: await iconPng(FaRobot),
    deskW: await iconPng(FaDesktop, "#FFFFFF"), chipW: await iconPng(FaMicrochip, "#FFFFFF"),
  };
  let page = 1;
  const content = (eyebrow, title) => {
    const s = pres.addSlide();
    s.background = { color: WHITE };
    H.heading(s, eyebrow, title);
    TEXT.push(eyebrow, title);
    H.badge(s, ++page);
    return s;
  };
  const note = (s, t) => s.addNotes(t);
  // a rounded "pill" box with centred text
  const pill = (s, x, y, w, h, t, { fill = WHITE, color = INK, bold = false, size = 10.5, line = LINE } = {}) => {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.06, fill: { color: fill }, line: { color: line, width: 1 } });
    say(s, t, { x: x + 0.08, y, w: w - 0.16, h, fontSize: size, color, bold, fontFace: FF, align: "center", valign: "middle" });
  };
  const down = (s, x, y, h = 0.14, color = TEAL) =>
    s.addShape(pres.shapes.LINE, { x, y, w: 0, h, line: { color, width: 1.5, endArrowType: "triangle" } });

  const saf = D28.safety, base = D28.baseline, N = D28.nfz;
  const S = D11.sm1111, S0 = D11.sm1110, R3 = D11.rh03, G = D11.ge1110;
  const sw = S.sweep, tk = S.ticks, rev = S.manifests.code_revisions.join(", ");

  /* 1 · COVER */
  {
    const s = pres.addSlide();
    H.darkBase(s, "Progress update · 11 Nov 2026");
    say(s, "VLA Drone with Guardrail:\nCity Follow and the KPI Rig", {
      x: 0.55, y: 1.1, w: 8.8, h: 1.9, fontSize: 34, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1 });
    say(s, "Project AirSim city · ArduPilot SITL on the AGX Orin · one safety layer", {
      x: 0.55, y: 3.1, w: 8.8, h: 0.5, fontSize: 15, color: TEAL_TINT, fontFace: FF });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.1, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    say(s, [
      { text: "Department of Electronic Engineering, National Taipei University of Technology\n" },
      { text: "Advisor: Lai Kuan-Ting\n" }, { text: "Presenter: Nathanael Tjahyadi" },
    ], { x: 0.55, y: 4.22, w: 8.6, h: 1.0, fontSize: 11, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3 });
  }

  /* 2 · TWO SETUPS, ONE GUARDRAIL */
  {
    const s = content("Overview", "Two Test Setups, One Guardrail");
    const col = (x, title, purpose, boxes, hw) => {
      const w = 4.2;
      say(s, title, { x, y: 1.52, w, h: 0.32, fontSize: 13.5, color: TEAL, bold: true, fontFace: FF });
      say(s, purpose, { x, y: 1.82, w, h: 0.28, fontSize: 10.5, color: GREY, italic: true, fontFace: FF });
      boxes.forEach((b, i) => {
        const y = 2.2 + i * 0.52;
        pill(s, x, y, w, 0.4, b.t, b.hi ? { fill: TEAL, color: WHITE, bold: true, line: TEAL }
          : b.tint ? { fill: TEAL_TINT2, bold: true, line: TEAL } : { fill: TEAL_TINT });
        if (i < boxes.length - 1) down(s, x + w / 2, y + 0.4, 0.12);
      });
      say(s, hw, { x, y: 4.3, w, h: 0.3, fontSize: 9.5, color: GREY, fontFace: FF, align: "center" });
    };
    col(0.55, "City follow · Project AirSim", "Can a VLA fly the mission, safely?", [
      { t: "Camera image + \"follow the red car\"" }, { t: "VLA (or our controller) picks the move", hi: true },
      { t: "Guardrail checks every command", tint: true }, { t: "Drone in the 3D city" }],
      "PC (RTX 4090) + Jetson · 3D view, camera");
    col(5.25, "KPI rig · ArduPilot SITL", "Does the guardrail meet the grant's KPIs?", [
      { t: "Scripted pilot: same moves every run" }, { t: "Guardrail on the AGX Orin", tint: true },
      { t: "MAVROS 2 → ArduPilot autopilot", hi: true }, { t: "Simulated drone, real autopilot code" }],
      "Desktop PC + AGX Orin · no camera, map replay");
    s.addShape(pres.shapes.LINE, { x: 4.98, y: 1.6, w: 0, h: 3.0, line: { color: LINE, width: 1, dashType: "dash" } });
    pill(s, 0.55, 4.7, 8.6, 0.36, "Same Shield + escalation FSM in both: check, fix, brake, hold, return home",
      { fill: TEAL_DK, color: WHITE, bold: true, line: TEAL_DK, size: 10.5 });
    note(s, "Two setups answer two questions. The city tests whether a VLA can do the job with the guardrail under it. " +
      "The rig tests the guardrail itself against the grant's KPIs, with a scripted pilot so every scenario repeats exactly. " +
      "SITL has no camera, so the rig shows a map replay instead of a 3D view.");
  }

  /* 3 · THE TASK (city) */
  {
    const s = content("City follow · Project AirSim", "Follow the Red Car");
    const cy = 1.75, ch = 1.55, gap = 0.25, cw = (9.0 - 2 * gap) / 3;
    H.card(s, 0.55, cy, cw, ch, ic.car, "Mission", "Follow Car_10 for 7 min in the CityLife city.");
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.ruler, "Score", "Share of time the drone is within 30 m.");
    H.card(s, 0.55 + 2 * (cw + gap), cy, cw, ch, ic.shield, "Guardrail", "Checks every command, 10 times per second.");
    TEXT.push("Mission", "Score", "Guardrail");
    const sy = 3.55;
    H.stat(s, 0.7, sy, 2.8, String(saf.flights), "flights (27-28 Oct)");
    H.stat(s, 3.6, sy, 2.8, String(saf.mission_collisions), "collisions during a mission");
    H.stat(s, 6.5, sy, 2.8, saf.p0_escape_rates.join(" / "), "P0 escape rate (target 0)");
    note(s, "Results from the 28 Oct update. Same task, same city, same car route for every flight. Score counts from the first lock on the car.");
  }

  /* 4 · TWO WAYS TO USE A VLA */
  {
    const s = content("City follow · Project AirSim", "Two Ways to Use a VLA");
    const lane = (x, y, label, boxes) => {
      say(s, label, { x, y, w: 4.3, h: 0.35, fontSize: 13, color: TEAL, fontFace: FF, bold: true });
      boxes.forEach((b, i) => {
        const by = y + 0.45 + i * 0.68;
        s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: by, w: 4.2, h: 0.5, rectRadius: 0.06,
          fill: { color: b.hi ? TEAL : b.tint ? TEAL_TINT2 : TEAL_TINT }, line: { color: b.hi ? TEAL : LINE, width: 1 } });
        say(s, b.t, { x: x + 0.15, y: by, w: 3.9, h: 0.5, fontSize: 11, color: b.hi ? WHITE : INK, fontFace: FF, bold: !!b.hi, valign: "middle" });
        if (i < boxes.length - 1) down(s, x + 2.1, by + 0.5, 0.18);
      });
    };
    lane(0.55, 1.6, "Pure VLA (end to end)", [
      { t: "Camera + \"follow the red car\"" }, { t: "VLA: every move (forward, turn, climb)", hi: true },
      { t: "Guardrail: check and fix", tint: true }, { t: "Drone" }]);
    lane(5.2, 1.6, "Hierarchical (VLA + our controller)", [
      { t: "VLA: what to do + how far behind", hi: true }, { t: "Our controller: chase at 10 Hz (OWL-ViT + map)" },
      { t: "Guardrail: check and fix", tint: true }, { t: "Drone" }]);
    note(s, "Pure VLA: the model flies every step. Hierarchical: the model decides the intent and the following distance; our controller flies. The guardrail sits under both.");
  }

  /* 5 · HIERARCHICAL vs CONTROLLER (bars) */
  {
    const s = content("City follow · Project AirSim", "Hierarchical VLA vs Our Controller");
    const items = [["Our controller", base.PC, base.Jetson, true]];
    const byModel = {};
    D28.matrix_rows.filter((r) => r.kind === "hier")
      .forEach((r) => { (byModel[r.model] = byModel[r.model] || { PC: [], Jetson: [] })[r.platform].push(r.score); });
    const names = { aerovla: "AerialVLA (original)", r1: "FT r1", r2: "FT r2", r3: "FT r3", r4: "FT r4", dronecats: "DroneCATS-2B", racevla: "RaceVLA-7B" };
    Object.entries(byModel).sort((a, b) => mean([...b[1].PC, ...b[1].Jetson]) - mean([...a[1].PC, ...a[1].Jetson]))
      .forEach(([m, v]) => items.push([names[m], v.PC, v.Jetson, false]));
    const x0 = 2.55, wMax = 5.6, y0 = 1.74, rowH = 0.4;
    items.forEach(([n, pc, jt, hi], i) => {
      const y = y0 + i * rowH;
      say(s, n, { x: 0.55, y, w: 1.95, h: rowH, fontSize: 10.5, color: hi ? TEAL : INK, bold: hi, fontFace: FF, valign: "middle" });
      [[pc, TEAL, 0.05], [jt, ORANGE, 0.21]].forEach(([arr, c, dy]) => {
        if (!arr.length) return;
        const v = mean(arr);
        s.addShape(pres.shapes.RECTANGLE, { x: x0, y: y + dy, w: Math.max(0.02, wMax * v), h: 0.15, fill: { color: c }, line: { color: c } });
        say(s, f3(v), { x: x0 + wMax * v + 0.05, y: y + dy - 0.04, w: 0.6, h: 0.22, fontSize: 8.5, color: GREY, fontFace: FF });
      });
    });
    s.addShape(pres.shapes.RECTANGLE, { x: 6.0, y: 1.52, w: 0.16, h: 0.12, fill: { color: TEAL }, line: { color: TEAL } });
    say(s, "PC", { x: 6.2, y: 1.45, w: 0.4, h: 0.25, fontSize: 9, color: GREY, fontFace: FF });
    s.addShape(pres.shapes.RECTANGLE, { x: 6.65, y: 1.52, w: 0.16, h: 0.12, fill: { color: ORANGE }, line: { color: ORANGE } });
    say(s, "Jetson (VLA model on PC GPU)", { x: 6.85, y: 1.45, w: 2.35, h: 0.25, fontSize: 9, color: GREY, fontFace: FF });
    H.source(s, "Mean score of 1-3 flights each; bar length = share of time within 30 m. Our controller has no VLA: it ran fully on each computer.", 4.95);
    note(s, "Best VLA setup (DroneCATS-2B, hierarchical) comes close to our own controller. Pure VLA (not shown) loses the car at turns: scores below 0.55.");
  }

  /* 6 · THREE ZONES, NEVER ENTERED */
  {
    const s = content("City follow · Project AirSim", "Three No-Fly Zones: Never Entered");
    const map = await img(path.join(REPO, D28.figures.nfz3_map));
    H.figure(s, map, 0.55, 1.55, 4.7, 3.35);
    const P = N.PC, Jt = N.Jetson, zx = 5.55;
    say(s, [{ text: "Closest to each zone's edge", options: { bold: true, color: INK } }],
      { x: zx, y: 1.5, w: 3.9, h: 0.3, fontSize: 11, fontFace: FF });
    ["nfz-frontage", "nfz-north-street", "nfz-south-block"].forEach((z, i) => {
      const y = 1.85 + i * 0.5;
      s.addImage({ data: ic.ban, x: zx, y: y + 0.07, w: 0.24, h: 0.24 });
      say(s, z, { x: zx + 0.33, y, w: 1.9, h: 0.38, fontSize: 10.5, color: INK, fontFace: FF, valign: "middle" });
      say(s, `PC ${f2(P.zones[z].drone_min_dist_m)} m · Jetson ${f2(Jt.zones[z].drone_min_dist_m)} m`,
        { x: zx + 2.0, y, w: 1.95, h: 0.38, fontSize: 9.5, color: GREY, fontFace: FF, valign: "middle" });
    });
    const inside = ["nfz-frontage", "nfz-north-street", "nfz-south-block"]
      .reduce((a, z) => a + P.zones[z].drone_ticks_inside + Jt.zones[z].drone_ticks_inside, 0);
    H.stat(s, zx, 3.45, 1.9, String(inside), "ticks inside a zone");
    H.stat(s, zx + 2.0, 3.45, 1.9, `${P.p0_escape_rate}`, "P0 escape rate");
    H.source(s, `Drone keeps 4 m from every zone. PC: ${P.name}; Jetson: ${Jt.name}.`, 4.98);
    note(s, "Three zones placed on the car's route; the car is not limited, the drone is. Both test flights went round every zone. " +
      `Jetson score was lower (${f3(Jt.within30)}) because the car was lost at the north-east corner, the same corner where that model lost it without the zones.`);
  }

  /* 7 · CITY VIDEO */
  {
    const s = content("City follow · Project AirSim", "Flight Video: Guardrail in the City");
    const vw = 4.35, vh = vw * 9 / 16, vy = 1.6;
    [["clip_PC-DroneCATS.mp4", "poster_PC-DroneCATS.jpg", 0.55, "PC · DroneCATS-2B (hierarchical)"],
      ["clip_Jetson-AeroVLA.mp4", "poster_Jetson-AeroVLA.jpg", 5.1, "Jetson · AerialVLA (hierarchical)"]].forEach(([v, p, x, cap]) => {
      s.addMedia({ type: "video", path: path.join(BUILD, v), cover: jpg(p), x, y: vy, w: vw, h: vh });
      say(s, cap, { x, y: vy + vh + 0.08, w: vw, h: 0.3, fontSize: 11, color: INK, bold: true, fontFace: FF, align: "center" });
    });
    say(s, "45 s from each 7-min flight, near nfz-north-street. Rule panel top right; map bottom right.",
      { x: 0.55, y: 4.55, w: 8.9, h: 0.3, fontSize: 10, color: GREY, fontFace: FF, align: "center" });
    note(s, "Click a video to play. Panel: each rule with its state (OK / NEAR / AVOID). Banner: what the guardrail does now. " +
      "Full 7-minute videos are in the 28 Oct share folder.");
  }

  /* 8 · THE RIG (topology) */
  {
    const s = content("KPI rig · ArduPilot SITL", "The KPI Rig (hil Topology)");
    const by = 1.55, bh = 2.75, bw = 3.55;
    // Desktop PC (simulated drone)
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.55, y: by, w: bw, h: bh, rectRadius: 0.1, fill: { color: TEAL_TINT }, line: { color: TEAL, width: 1.25 } });
    s.addImage({ data: ic.desk, x: 0.75, y: by + 0.17, w: 0.36, h: 0.36 });
    say(s, "Desktop PC", { x: 1.2, y: by + 0.12, w: 2.7, h: 0.3, fontSize: 14, bold: true, color: INK, fontFace: FF });
    say(s, "the simulated drone", { x: 1.2, y: by + 0.4, w: 2.7, h: 0.25, fontSize: 9.5, color: GREY, italic: true, fontFace: FF });
    pill(s, 0.75, by + 0.8, bw - 0.4, 0.5, "ArduPilot SITL\nsame code as drone firmware", { bold: false, size: 10 });
    pill(s, 0.75, by + 1.42, bw - 0.4, 0.5, "Scenario: zones, wind, GPS loss", { size: 10 });
    pill(s, 0.75, by + 2.04, bw - 0.4, 0.5, "Scorer: checks every run", { size: 10 });
    // AGX Orin (real hardware)
    const ox = 5.9;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: ox, y: by, w: bw, h: bh, rectRadius: 0.1, fill: { color: TEAL_DK }, line: { color: TEAL_DK, width: 1.25 } });
    s.addImage({ data: ic.chipW, x: ox + 0.2, y: by + 0.17, w: 0.36, h: 0.36 });
    say(s, "AGX Orin", { x: ox + 0.65, y: by + 0.12, w: 2.7, h: 0.3, fontSize: 14, bold: true, color: WHITE, fontFace: FF });
    say(s, "real companion computer", { x: ox + 0.65, y: by + 0.4, w: 2.7, h: 0.25, fontSize: 9.5, color: TEAL_TINT, italic: true, fontFace: FF });
    const op = { fill: WHITE, line: WHITE, size: 10 };
    pill(s, ox + 0.2, by + 0.8, bw - 0.4, 0.42, "① Pilot asks for a move", op);
    down(s, ox + bw / 2, by + 1.22, 0.12, WHITE);
    pill(s, ox + 0.2, by + 1.34, bw - 0.4, 0.5, "② Shield + FSM check it, 10×/s\nfix · brake · hold · return home", { ...op, bold: true });
    down(s, ox + bw / 2, by + 1.84, 0.12, WHITE);
    pill(s, ox + 0.2, by + 1.96, bw - 0.4, 0.42, "③ MAVROS 2 sends the safe command", op);
    // the cable: command out, state back
    const cx = 0.55 + bw, cw2 = ox - cx;
    s.addShape(pres.shapes.LINE, { x: cx + 0.08, y: by + 2.17, w: cw2 - 0.16, h: 0, line: { color: TEAL, width: 2.25, beginArrowType: "triangle" } });
    say(s, "safe command", { x: cx, y: by + 1.88, w: cw2, h: 0.26, fontSize: 9, color: TEAL, bold: true, fontFace: FF, align: "center" });
    s.addShape(pres.shapes.LINE, { x: cx + 0.08, y: by + 1.1, w: cw2 - 0.16, h: 0, line: { color: ORANGE, width: 2.25, endArrowType: "triangle" } });
    say(s, "④ position, mode", { x: cx, y: by + 0.81, w: cw2, h: 0.26, fontSize: 9, color: ORANGE, bold: true, fontFace: FF, align: "center" });
    say(s, "Ethernet\ncable", { x: cx, y: by + 1.36, w: cw2, h: 0.45, fontSize: 9, color: GREY, fontFace: FF, align: "center" });
    // bottom: what is real, what counts
    pill(s, 0.55, 4.45, 8.9, 0.42,
      `KPI-grade (grant): this topology · real time · clean code · signed policy. All ${sw.missions_flown} missions of the 11 Nov campaign met it.`,
      { fill: TEAL_TINT2, line: TEAL_TINT2, size: 10 });
    note(s, "Only the AGX Orin is real hardware; the drone and its autopilot are simulated, but the autopilot runs the same ArduPilot code as a real drone. " +
      `The grant counts KPI numbers only from this topology. Commit ${rev}. Video slides from the city are 3D; this rig has no camera, so its video is a map replay.`);
  }

  /* 8b · WHAT RUNS WHERE (hardware + software) */
  {
    const s = content("KPI rig · ArduPilot SITL", "What Runs Where: Hardware + Software");
    const by = 1.55, bh = 3.2, bw = 3.8, px = 0.55, ax = 5.65;
    const bY = [2.2, 3.04, 3.88], bH = 0.72;
    // a software block: category eyebrow, name, one line; dashed = not yet flown on the rig
    const block = (x, y, cat, name, line, { dashed = false, dark = false } = {}) => {
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w: bw - 0.3, h: bH, rectRadius: 0.06,
        fill: { color: WHITE }, line: { color: dashed ? GREY : dark ? TEAL_DK : LINE, width: dashed ? 1.25 : 1, dashType: dashed ? "dash" : "solid" } });
      say(s, cat, { x: x + 0.12, y: y + 0.04, w: bw - 0.55, h: 0.2, fontSize: 7.5, color: TEAL, bold: true, charSpacing: 2, fontFace: FF });
      say(s, name, { x: x + 0.12, y: y + 0.22, w: bw - 0.55, h: 0.25, fontSize: 10.5, color: INK, bold: true, fontFace: FF });
      say(s, line, { x: x + 0.12, y: y + 0.45, w: bw - 0.55, h: 0.24, fontSize: 8.5, color: GREY, fontFace: FF });
    };
    // Desktop PC
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: px, y: by, w: bw, h: bh, rectRadius: 0.1, fill: { color: TEAL_TINT }, line: { color: TEAL, width: 1.25 } });
    s.addImage({ data: ic.desk, x: px + 0.15, y: by + 0.12, w: 0.32, h: 0.32 });
    say(s, "Desktop PC (RTX 4090)", { x: px + 0.55, y: by + 0.08, w: bw - 0.6, h: 0.28, fontSize: 12.5, bold: true, color: INK, fontFace: FF });
    say(s, "Windows + WSL2", { x: px + 0.55, y: by + 0.33, w: bw - 0.6, h: 0.22, fontSize: 9, italic: true, color: GREY, fontFace: FF });
    block(px + 0.15, bY[0], "ENVIRONMENT", "Project AirSim (Unreal)", "3D city, red car, camera", { dashed: true });
    block(px + 0.15, bY[1], "AUTOPILOT", "ArduPilot SITL (in WSL)", "same code as drone firmware");
    block(px + 0.15, bY[2], "LINK + TEST TOOLS", "mavlink-router · scenarios · scorer", "wind, GPS loss, zones; scores each run");
    // AGX Orin
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: ax, y: by, w: bw, h: bh, rectRadius: 0.1, fill: { color: TEAL_DK }, line: { color: TEAL_DK, width: 1.25 } });
    s.addImage({ data: ic.chipW, x: ax + 0.15, y: by + 0.12, w: 0.32, h: 0.32 });
    say(s, "AGX Orin 64 GB", { x: ax + 0.55, y: by + 0.08, w: bw - 0.6, h: 0.28, fontSize: 12.5, bold: true, color: WHITE, fontFace: FF });
    say(s, "Ubuntu 24.04 · ROS 2 Jazzy", { x: ax + 0.55, y: by + 0.33, w: bw - 0.6, h: 0.22, fontSize: 9, italic: true, color: TEAL_TINT, fontFace: FF });
    block(ax + 0.15, bY[0], "VLA (PILOT)", "AIoT-VLA + OWL-ViT detector", "33 ms per decision on the Orin", { dashed: true });
    block(ax + 0.15, bY[1], "GUARDRAIL", "Shield + escalation FSM", "checks every command, 10×/s", { dark: true });
    block(ax + 0.15, bY[2], "BRIDGE", "MAVROS 2 (container)", "ROS 2 ↔ MAVLink");
    [bY[0] + bH, bY[1] + bH].forEach((y) => down(s, ax + bw / 2, y, 0.12, WHITE));
    // inside the PC: AirSim <-> SITL <-> router
    [bY[0] + bH, bY[1] + bH].forEach((y) => s.addShape(pres.shapes.LINE, { x: px + bw / 2, y, w: 0, h: 0.12,
      line: { color: TEAL, width: 1.5, beginArrowType: "triangle", endArrowType: "triangle" } }));
    // across the cable
    const cx = px + bw + 0.05, cw2 = ax - cx - 0.05, mid = bY[0] + bH / 2;
    s.addShape(pres.shapes.LINE, { x: cx, y: mid, w: cw2, h: 0, line: { color: GREY, width: 1.75, dashType: "dash", endArrowType: "triangle" } });
    say(s, "camera image", { x: cx - 0.05, y: mid - 0.3, w: cw2 + 0.1, h: 0.26, fontSize: 8.5, color: GREY, bold: true, fontFace: FF, align: "center" });
    const m3 = bY[2] + bH / 2;
    s.addShape(pres.shapes.LINE, { x: cx, y: m3 - 0.13, w: cw2, h: 0, line: { color: ORANGE, width: 2, endArrowType: "triangle" } });
    say(s, "position, mode", { x: cx - 0.05, y: m3 - 0.42, w: cw2 + 0.1, h: 0.26, fontSize: 8.5, color: ORANGE, bold: true, fontFace: FF, align: "center" });
    s.addShape(pres.shapes.LINE, { x: cx, y: m3 + 0.13, w: cw2, h: 0, line: { color: TEAL, width: 2, beginArrowType: "triangle" } });
    say(s, "safe command", { x: cx - 0.05, y: m3 + 0.17, w: cw2 + 0.1, h: 0.26, fontSize: 8.5, color: TEAL, bold: true, fontFace: FF, align: "center" });
    say(s, "Ethernet\ncable", { x: cx, y: bY[1] + 0.1, w: cw2, h: 0.5, fontSize: 9, color: GREY, fontFace: FF, align: "center" });
    // legend
    const ly = 4.88;
    s.addShape(pres.shapes.RECTANGLE, { x: 0.6, y: ly + 0.07, w: 0.32, h: 0.17, fill: { color: WHITE }, line: { color: LINE, width: 1 } });
    say(s, "runs in the KPI campaign today", { x: 1.0, y: ly, w: 3.0, h: 0.3, fontSize: 9.5, color: INK, fontFace: FF });
    s.addShape(pres.shapes.RECTANGLE, { x: 3.95, y: ly + 0.07, w: 0.32, h: 0.17, fill: { color: WHITE }, line: { color: GREY, width: 1.25, dashType: "dash" } });
    say(s, "next step: built, not yet flown on the rig",
      { x: 4.35, y: ly, w: 4.85, h: 0.3, fontSize: 9.5, color: INK, fontFace: FF });
    note(s, "Where each piece runs. PC: the environment (Project AirSim) and the autopilot (ArduPilot SITL), plus the scenario and scoring tools. " +
      "AGX Orin: the pilot (AIoT-VLA with the OWL-ViT detector), the guardrail (Shield + FSM) and MAVROS 2. " +
      "In today's KPI campaign the pilot is scripted (also on the Orin) and SITL uses its own physics, so the dashed parts are not used. " +
      "The dashed setup is built (scripts ready, AIoT-VLA measured at 33 ms on the Orin) but not yet flown; AerialVLA did fly ArduPilot inside Project AirSim on the PC on 29 Oct.");
  }

  /* 9 · BEFORE / AFTER */
  {
    const s = content("KPI rig · ArduPilot SITL", "Rig Campaigns, Before and After");
    const row = (name, c) => [name, String(c.sweep.missions_flown), f3(c.sweep.p0_shield_on_max),
      String(c.sweep.failsafe_ideal.censored_in_loiter), `${c.sweep.tts_mean_s} s`, `${f1(c.ticks.tick_max)} ms`];
    H.table(s, [
      ["Campaign", "Missions", "P0 escape\n(Shield on)", "Stuck in\nhold", "Time to\nsafe", "Slowest\ntick"],
      row("7 Nov · rh03", R3), row("10 Nov · sm1110", S0), row("11 Nov · sm1111", S),
    ], { y: 1.6, colFrac: [0.24, 0.13, 0.17, 0.14, 0.15, 0.17], rh: 0.5, fontSize: 11, emph: [[3, 2], [3, 3], [3, 4]] });
    const cy = 3.75, cw = 2.85, gap = 0.175;
    H.stat(s, 0.7, cy, cw, `${sw.p0_control.join(", ")} → ${f3(sw.p0_shield_on_max)}`, "P0 escape, Shield off → on", TEAL, 26);
    H.stat(s, 0.7 + cw + gap, cy, cw, `${R3.sweep.tts_mean_s} → ${sw.tts_mean_s} s`, "mean time to safe", TEAL, 26);
    H.stat(s, 0.7 + 2 * (cw + gap), cy, cw, `${f1(tk.tick_max)} ms`, `slowest of ${tk.tick_n.toLocaleString("en-US")} Orin ticks (budget 100)`, TEAL, 26);
    note(s, `Same smoke scenario library each time, flown in real time on the rig. sm1111 at ${rev}. ` +
      "Stuck in hold = the run ended while the aircraft was still held, so the fail-safe outcome is unknown. Shield off = the control arm with the same pilot.");
  }

  /* 10 · WHAT THE RIG TAUGHT US */
  {
    const s = content("KPI rig · ArduPilot SITL", "Found Only on the Real Rig");
    H.table(s, [
      ["Problem on the rig", "Fix", "Proof"],
      ["Hold point swept by a rotating zone", "Hold must stay legal while held", "Rotate-spin 8/8 correct"],
      ["Autopilot BRAKE drifts ~2.5 m", "Shield holds in GUIDED", "Hold stays on its point"],
      ["Hold planning took 147 ms on Orin", "Spread over ticks", "Max tick 96 ms"],
      ["Hold / release loop, 14 times", "Release only if safe at speed", "Fail-safe fires as labelled"],
      ["“Landed” never reported", "Ask ArduPilot by message ID", "G10, X5 fire on the rig"],
      ["Take-off into the GeoFence", "Wait for SITL params; refuse", "Unit-tested; to re-fly"],
    ], { y: 1.55, colFrac: [0.38, 0.33, 0.29], rh: 0.46, fontSize: 10.5 });
    note(s, "None of these appeared in simulation. Each was found by a short shakedown on the rig, fixed, tested in CI, and re-flown (the GeoFence one is not re-flown yet). " +
      "The BRAKE finding: ArduCopter's BRAKE keeps the last GUIDED acceleration, so it carries the aircraft along the last command.");
  }

  /* 11 · STRESS + GRANT EXAMPLE */
  {
    const s = content("KPI rig · ArduPilot SITL", "Stress on the Rig, and the Grant's Own Example");
    const cy = 1.65, cw = 4.35, ch = 1.55;
    H.card(s, 0.55, cy, cw, ch, ic.wind, "Wind and GPS loss", "Wind 5 / 8 / 12 m/s, GPS off 4-11 s, set in SITL.", 10.5);
    H.card(s, 0.55 + cw + 0.2, cy, cw, ch, ic.shield, "FSM edges first flown", "G10 landed · X5 landed in RTL · X8 GPS lost.", 10.5);
    H.card(s, 0.55, cy + ch + 0.25, 8.9, 1.45, ic.flag, "Grant's example sweep, flown on the rig",
      `${G.sweep.missions_flown - 4} of ${G.sweep.missions_flown} flew (4 hit the GeoFence bug): ${frac(G.readings.r2)} correct, P0 ${f3(G.sweep.p0_shield_on_max)}, success ${f3(G.sweep.mission_success_rate)}.`, 10.5);
    note(s, "The grant's ScenarioSpec example (zone spawned 60 m ahead, sliding 5 m/s sideways, speeds 6-12 m/s, wind 0-5) flown as written on the hil rig. " +
      "Its zone never blocks the path, so its 2.0 s time-to-safe target is never exercised.");
  }

  /* 12 · RIG VIDEO */
  {
    const s = content("KPI rig · ArduPilot SITL", "Flight Video: the Rig, Replayed");
    const vw = 5.4, vh = vw * 9 / 16;
    s.addMedia({ type: "video", path: path.join(SHARE, "VLA-Guardrail-Progress-11Nov2026.mp4"), cover: jpg("thumb_headon.jpg"),
      x: 0.55, y: 1.6, w: vw, h: vh });
    H.card(s, 6.15, 1.6, 3.3, vh, ic.video, "2:43, four clips",
      "1. Shield OFF vs ON, head-on\n2. Rotating bar: old vs new\n3. Corridor drift: 14 restarts vs RTL\n4. 5 m/s wind: goal reached", 10.5);
    say(s, "Top-down replay of the flight logs: SITL has no camera.", { x: 0.55, y: 1.6 + vh + 0.1, w: 8.9, h: 0.3, fontSize: 10, color: GREY, fontFace: FF });
    note(s, "Clip 1 (Shield on): the RTL at the end is not the point; the point is no zone entry. Clip 2: old build flew the bar at 3 deg/s, new at 6 deg/s.");
  }

  /* 13 · NEXT */
  {
    const s = content("Next", "Next Steps");
    const cy = 1.6, gap = 0.2, cw = (8.9 - gap) / 2, ch = 1.5;
    H.card(s, 0.55, cy, cw, ch, ic.clock, "Nightly campaign", "591 missions (~20 h) for the final KPI numbers.", 10.5);
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.robot, "Our VLA on the rig", "AIoT-VLA on the Orin, in the 3D city, with ArduPilot.", 10.5);
    H.card(s, 0.55, cy + ch + 0.2, cw, ch, ic.wind, "Strong wind", "Return home when wind beats the drone.", 10.5);
    H.card(s, 0.55 + cw + gap, cy + ch + 0.2, cw, ch, ic.flag, "Final report", "Due 30 Nov, with every KPI and its evidence.", 10.5);
    note(s, "Contract (say, do not show): most grant obligations are done or in progress; the nightly campaign closes the KPI items. " +
      "Open but small: a SITL parameter read that starts too early on 2 stress missions; one ~110 ms tick per long mission in the grant example sweep (older commit), under investigation.");
  }

  /* 14 · THANK YOU */
  {
    const s = pres.addSlide();
    H.darkBase(s, null);
    say(s, "Thank You", { x: 0.55, y: 2.1, w: 9, h: 1.2, fontSize: 48, color: WHITE, fontFace: FF, bold: true });
    s.addShape(pres.shapes.LINE, { x: 0.6, y: 3.35, w: 1.2, h: 0, line: { color: TEAL, width: 3 } });
  }

  const bad = TEXT.filter((t) => /undefined|NaN|null/.test(t));
  if (bad.length) throw new Error("a slide would print undefined/NaN/null: " + bad.join(" | "));
  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  const mb = fs.statSync(OUT).size / 1e6;
  if (mb > 60) throw new Error(`deck is ${mb.toFixed(1)} MB (limit 60)`);
  console.log(`written: ${OUT} (${mb.toFixed(2)} MB)`);
}

main().catch((e) => { console.error(e); process.exit(1); });
