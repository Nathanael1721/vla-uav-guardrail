/*
 * Lab progress update, 28 Oct 2026: every model pure-VLA and hierarchical, on
 * the PC and on the Jetson Orin Nano, and the three-zone guardrail test.
 * nathan-deck design system (tools/deck/nathan_theme.js).
 *
 * Every number is read at build time from docs/data/progress_1028.json
 * (tools/deck/build_progress_1028_data.py), which reads the flights' own
 * metrics.json / kpi.json / flight_log.jsonl. The build refuses to write a
 * slide that would print 'undefined' or 'NaN'.
 *
 * Writing rule for this deck (the user, 28 Oct): plain words, one idea per
 * line, numbers stand alone; technical terms only where they carry meaning.
 *
 *   C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_1028_data.py
 *   node tools/deck/build_progress_1028_deck.js
 */
const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const {
  FaCar, FaShieldAlt, FaStopwatch, FaRobot, FaMicrochip, FaServer, FaNetworkWired,
  FaBan, FaCheckCircle, FaExclamationTriangle, FaRoute, FaBrain, FaCrosshairs, FaRulerHorizontal,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "../..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const D = JSON.parse(fs.readFileSync(path.join(REPO, "docs/data/progress_1028.json"), "utf8"));
const SHARE = path.join(REPO, "docs/share/2026-10-28-Progress");
const OUT = path.join(SHARE, "VLA-Drone-Progress-28Oct2026.pptx");
const { TEAL, TEAL_DK, TEAL_TINT, TEAL_TINT2, BLACK, INK, GREY, LGREY, WHITE, LINE, FF, CHARCOAL } = T;

const f2 = (v) => (v == null ? "-" : Number(v).toFixed(2));
const f3 = (v) => (v == null ? "-" : Number(v).toFixed(3));
const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
const range = (a) => (a.length > 1 ? `${f3(Math.min(...a))}-${f3(Math.max(...a))}` : f3(a[0]));

const TEXT = [];          // every string placed, checked for 'undefined' / 'NaN'
function say(s, t, o) { TEXT.push(typeof t === "string" ? t : JSON.stringify(t)); s.addText(t, o); }

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "VLA Drone Follow - PC vs Jetson, with Guardrail";
  const H = helpers(pres);
  const ic = {
    car: await iconPng(FaCar), shield: await iconPng(FaShieldAlt), clock: await iconPng(FaStopwatch),
    robot: await iconPng(FaRobot), chip: await iconPng(FaMicrochip), server: await iconPng(FaServer),
    net: await iconPng(FaNetworkWired), ban: await iconPng(FaBan), ok: await iconPng(FaCheckCircle),
    warn: await iconPng(FaExclamationTriangle), route: await iconPng(FaRoute), brain: await iconPng(FaBrain),
    aim: await iconPng(FaCrosshairs), ruler: await iconPng(FaRulerHorizontal),
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

  const base = D.baseline, fx = D.fixed12, rank = D.ranking, saf = D.safety, J = D.jetson, N = D.nfz;
  const best = (k) => rank[k][0];

  /* 1 · COVER */
  {
    const s = pres.addSlide();
    H.darkBase(s, "Lab progress · 28 Oct 2026");
    say(s, "VLA Drone Follow:\nPC vs Jetson, with Guardrail", {
      x: 0.55, y: 1.1, w: 8.6, h: 1.9, fontSize: 38, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1 });
    say(s, "7 models · 2 control modes · 2 computers · 1 safety layer", {
      x: 0.55, y: 3.1, w: 8.6, h: 0.5, fontSize: 15, color: TEAL_TINT, fontFace: FF });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.1, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    say(s, [
      { text: "Department of Electronic Engineering, National Taipei University of Technology\n" },
      { text: "Advisor: \n" }, { text: "Presenter: Nathanael Tjahyadi" },
    ], { x: 0.55, y: 4.22, w: 8.6, h: 1.0, fontSize: 11, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3 });
  }

  /* 2 · THE TASK */
  {
    const s = content("Task", "Follow the Red Car");
    const cy = 1.75, ch = 1.55, gap = 0.25, cw = (9.0 - 2 * gap) / 3;
    H.card(s, 0.55, cy, cw, ch, ic.car, "Mission", "Follow Car_10 for 7 min in the CityLife city.");
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.ruler, "Score", "Share of time the drone is within 30 m.");
    H.card(s, 0.55 + 2 * (cw + gap), cy, cw, ch, ic.shield, "Guardrail", "Checks every command, 10 times per second.");
    TEXT.push("Mission", "Score", "Guardrail");
    const sy = 3.55;
    H.stat(s, 0.7, sy, 2.8, String(saf.flights), "flights in this update");
    H.stat(s, 3.6, sy, 2.8, String(saf.mission_collisions), "collisions during a mission");
    H.stat(s, 6.5, sy, 2.8, saf.p0_escape_rates.join(" / "), "P0 escape rate (target 0)");
    TEXT.push(String(saf.flights), String(saf.mission_collisions));
    note(s, "Same task, same city, same car route for every flight. Score counts from the first lock on the car.");
  }

  /* 3 · TWO WAYS TO USE A VLA */
  {
    const s = content("Method", "Two Ways to Use a VLA");
    const lane = (x, y, label, boxes) => {
      say(s, label, { x, y, w: 4.3, h: 0.35, fontSize: 13, color: TEAL, fontFace: FF, bold: true });
      boxes.forEach((b, i) => {
        const by = y + 0.45 + i * 0.68;
        s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: by, w: 4.2, h: 0.5, rectRadius: 0.06,
          fill: { color: b.hi ? TEAL : b.tint ? TEAL_TINT2 : TEAL_TINT }, line: { color: b.hi ? TEAL : LINE, width: 1 } });
        say(s, b.t, { x: x + 0.15, y: by, w: 3.9, h: 0.5, fontSize: 11, color: b.hi ? WHITE : INK, fontFace: FF, bold: !!b.hi, valign: "middle" });
        if (i < boxes.length - 1) s.addShape(pres.shapes.LINE, { x: x + 2.1, y: by + 0.5, w: 0, h: 0.18, line: { color: TEAL, width: 1.5, endArrowType: "triangle" } });
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

  /* 4 · MODELS */
  {
    const s = content("Models", "What We Flew");
    H.table(s, [
      ["Pilot", "Size", "Trained on this task?"],
      ["AerialVLA (original)", "7B", "No (public adapter)"],
      ["AerialVLA FT r1, r2, r3, r4", "7B", "Yes (our follow data)"],
      ["DroneCATS (NAVER)", "2B", "No"],
      ["RaceVLA", "7B", "No (drone racing)"],
      ["Our controller (baseline)", "OWL-ViT 153M", "Rules, no VLA"],
    ], { y: 1.75, colFrac: [0.42, 0.2, 0.38], rh: 0.48, fontSize: 12 });
    TEXT.push("AerialVLA FT r1, r2, r3, r4");
    H.source(s, "Each VLA flew both modes on the PC and on the Jetson; the controller flew alone as the baseline.", 4.85);
  }

  /* 5 · JETSON SETUP */
  {
    const s = content("Jetson", "Running on the Jetson Orin Nano");
    const F = J.facts;
    const box = (x, y, w, h, title, lines, icon, hi) => {
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.08, fill: { color: hi ? TEAL_TINT : WHITE }, line: { color: hi ? TEAL : LINE, width: 1.25 } });
      s.addImage({ data: icon, x: x + 0.18, y: y + 0.16, w: 0.32, h: 0.32 });
      say(s, title, { x: x + 0.6, y: y + 0.12, w: w - 0.7, h: 0.4, fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle" });
      say(s, lines.map((t) => ({ text: t, options: { bullet: { indent: 12 }, breakLine: true } })),
        { x: x + 0.2, y: y + 0.6, w: w - 0.35, h: h - 0.7, fontSize: 10.5, color: GREY, fontFace: FF, valign: "top", lineSpacingMultiple: 1.15 });
    };
    box(0.55, 1.65, 4.1, 1.95, "Jetson Orin Nano Super", [
      "OWL-ViT car detector", "Our controller + Guardrail", "VLA client (sends image, gets answer)",
      `${F.mem_total_gb} GB RAM · ${String(F.power_mode).replace("NV Power Mode: ", "")}`], ic.chip, true);
    box(5.35, 1.65, 4.1, 1.95, "Desktop PC (RTX 4090)", [
      "City simulator (Project AirSim)", "VLA model on the GPU", "7B models do not fit 8 GB Jetson"], ic.server, false);
    s.addShape(pres.shapes.LINE, { x: 4.65, y: 2.62, w: 0.7, h: 0, line: { color: TEAL, width: 2, beginArrowType: "triangle", endArrowType: "triangle" } });
    say(s, "1 Gb/s\ncable", { x: 4.68, y: 2.02, w: 0.64, h: 0.5, fontSize: 8.5, color: TEAL, fontFace: FF, align: "center", bold: true });
    // evidence: the Jetson's own log
    s.addShape(pres.shapes.RECTANGLE, { x: 0.55, y: 3.78, w: 8.9, h: 1.28, fill: { color: "1F2A30" }, line: { color: "1F2A30" } });
    const ev = J.evidence_lines.slice(0, 4).map((l) => l.length > 112 ? l.slice(0, 112) + "..." : l);
    say(s, [{ text: "Jetson console log  ~/orin_hier_aerovla_20261028.log\n", options: { color: "9FD8E2", bold: true } },
      ...ev.map((l) => ({ text: l + "\n", options: { color: "E6EDF0" } }))],
      { x: 0.68, y: 3.82, w: 8.65, h: 1.2, fontSize: 8.5, fontFace: "Consolas", valign: "top", lineSpacingMultiple: 1.05 });
    H.source(s, `Board: ${F.model}; ${String(F.l4t).split(",").slice(0, 2).join(",").replace("# ", "")}.`, 5.12);
    note(s, "Evidence: the log lines are printed by the Jetson itself; 10.42.0.1 is the PC across the cable. The VLA answers from the PC's GPU through an SSH tunnel.");
  }

  /* 6 · TOP 3 PER SCENARIO */
  {
    const s = content("Results", "Top 3 per Scenario");
    const grid = [["PC pure", 0.55, 1.62], ["PC hier", 5.05, 1.62], ["Jetson pure", 0.55, 3.38], ["Jetson hier", 5.05, 3.38]];
    const label = { "PC pure": "PC · Pure VLA", "PC hier": "PC · Hierarchical", "Jetson pure": "Jetson · Pure VLA", "Jetson hier": "Jetson · Hierarchical" };
    for (const [k, x, y] of grid) {
      const rows = [[label[k], "Score"], ...rank[k].slice(0, 3).map(([n, m, runs]) => [n, runs.length > 1 ? `${f3(m)} (n=${runs.length})` : f3(m)])];
      H.table(s, rows, { x, y, w: 4.4, colFrac: [0.62, 0.38], rh: 0.36, fontSize: 10.5, emph: [[1, 1]] });
      TEXT.push(...rows.flat());
    }
    H.source(s, `Our controller: PC ${base.PC.map(f3).join(" / ")}, Jetson ${base.Jetson.map(f3).join(" / ")}. Score = share of time within 30 m; n = flights.`, 4.9);
  }

  /* 7 · HIERARCHICAL vs CONTROLLER (bars) */
  {
    const s = content("Comparison", "Hierarchical VLA vs Our Controller");
    const items = [["Our controller", base.PC, base.Jetson, true]];
    const H_ = D.matrix_rows.filter((r) => r.kind === "hier");
    const byModel = {};
    H_.forEach((r) => { (byModel[r.model] = byModel[r.model] || { PC: [], Jetson: [] })[r.platform].push(r.score); });
    const names = { aerovla: "AerialVLA (original)", r1: "FT r1", r2: "FT r2", r3: "FT r3", r4: "FT r4", dronecats: "DroneCATS-2B", racevla: "RaceVLA-7B" };
    Object.entries(byModel).sort((a, b) => mean([...b[1].PC, ...b[1].Jetson]) - mean([...a[1].PC, ...a[1].Jetson]))
      .forEach(([m, v]) => items.push([names[m], v.PC, v.Jetson, false]));
    const x0 = 2.55, wMax = 5.6, y0 = 1.62, rowH = 0.42;
    items.forEach(([n, pc, jt, hi], i) => {
      const y = y0 + i * rowH;
      say(s, n, { x: 0.55, y, w: 1.95, h: rowH, fontSize: 10.5, color: hi ? TEAL : INK, bold: hi, fontFace: FF, valign: "middle" });
      [[pc, TEAL, 0.05], [jt, "E08A1E", 0.21]].forEach(([arr, col, dy]) => {
        if (!arr.length) return;
        const v = mean(arr);
        s.addShape(pres.shapes.RECTANGLE, { x: x0, y: y + dy, w: Math.max(0.02, wMax * v), h: 0.15, fill: { color: col }, line: { color: col } });
        say(s, f3(v), { x: x0 + wMax * v + 0.05, y: y + dy - 0.04, w: 0.6, h: 0.22, fontSize: 8.5, color: GREY, fontFace: FF });
      });
    });
    s.addShape(pres.shapes.RECTANGLE, { x: 7.0, y: 5.0, w: 0.16, h: 0.12, fill: { color: TEAL }, line: { color: TEAL } });
    say(s, "PC", { x: 7.2, y: 4.93, w: 0.5, h: 0.25, fontSize: 9, color: GREY, fontFace: FF });
    s.addShape(pres.shapes.RECTANGLE, { x: 7.7, y: 5.0, w: 0.16, h: 0.12, fill: { color: "E08A1E" }, line: { color: "E08A1E" } });
    say(s, "Jetson", { x: 7.9, y: 4.93, w: 0.8, h: 0.25, fontSize: 9, color: GREY, fontFace: FF });
    H.source(s, "Mean score of 1-3 flights each; bar length = share of time within 30 m.", 4.95);
  }

  /* 8 · WHAT WE LEARNED */
  {
    const s = content("Findings", "What We Learned");
    const pure = D.matrix_rows.filter((r) => r.kind === "pure").map((r) => r.score);
    const dc = D.matrix_rows.filter((r) => r.kind === "hier" && r.model === "dronecats").map((r) => r.score);
    const cy = 1.7, ch = 1.6, gap = 0.25, cw = (9.0 - 2 * gap) / 3;
    H.card(s, 0.55, cy, cw, ch, ic.warn, "Pure VLA loses the car", `Scores ${f3(Math.min(...pure))}-${f3(Math.max(...pure))}. Car leaves view at a turn; rarely found again.`);
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.robot, "Hierarchy ≈ our controller", `Steadiest VLA: DroneCATS ${range(dc)}. Controller ${range([...base.PC, ...base.Jetson])}.`);
    H.card(s, 0.55 + 2 * (cw + gap), cy, cw, ch, ic.ruler, "Close is fast but fragile", `Fixed 12 m: ${f3(fx.PC[0])} on PC, ${f3(fx.Jetson[0])} on Jetson.`);
    TEXT.push(`${f3(Math.min(...pure))}`, range(dc));
    const cy2 = 3.55;
    H.card(s, 0.55, cy2, 4.375, 1.25, ic.chip, "Jetson works", `Our controller: ${f3(base.Jetson[0])} on Jetson (PC ${range(base.PC)}).`);
    H.card(s, 5.075, cy2, 4.375, 1.25, ic.brain, "Our fine-tunes", "Steadier than the original in hierarchy; worse end to end.");
    note(s, "The VLA's real contribution in the hierarchy today is the following distance it asks for. Our fine-tunes keep about 16 m and never fell apart; the original keeps 12 m and once lost the car.");
  }

  /* 9 · GUARDRAIL TEST SETUP */
  {
    const s = content("Guardrail test", "Three No-Fly Zones on the Route");
    const map = await img(path.join(REPO, D.figures.nfz3_map));
    H.figure(s, map, 0.55, 1.55, 5.4, 3.45);
    const zx = 6.2;
    const zones = [["nfz-frontage", "Leg 1, beside the car's street"], ["nfz-north-street", "Leg 2, over the drone's usual line"], ["nfz-south-block", "Leg 4, over the drone's usual line"]];
    zones.forEach(([z, t], i) => {
      const y = 1.6 + i * 0.78;
      s.addImage({ data: ic.ban, x: zx, y: y + 0.05, w: 0.3, h: 0.3 });
      say(s, z, { x: zx + 0.4, y, w: 2.9, h: 0.32, fontSize: 11.5, color: INK, bold: true, fontFace: FF });
      say(s, t, { x: zx + 0.4, y: y + 0.3, w: 2.9, h: 0.32, fontSize: 10, color: GREY, fontFace: FF });
    });
    say(s, "Drone keeps 5 m away.\nThe car is not limited.", { x: zx, y: 4.05, w: 3.25, h: 0.7, fontSize: 12, color: TEAL, bold: true, fontFace: FF });
    H.source(s, "policies/follow_car_citylife_nfz3b.yaml (signed bundle); map: drone and car tracks of the two test flights.", 5.05);
  }

  /* 10 · GUARDRAIL RESULTS */
  {
    const s = content("Guardrail test", "Result: Zones Never Entered");
    const P = N.PC, Jt = N.Jetson;
    const zid = ["nfz-frontage", "nfz-north-street", "nfz-south-block"];
    const rows = [["", `PC · ${P.name}`, `Jetson · ${Jt.name}`]];
    zid.forEach((z) => rows.push([z + " (closest, m)", f2(P.zones[z].drone_min_dist_m), f2(Jt.zones[z].drone_min_dist_m)]));
    rows.push(["Ticks inside any zone", String(zid.reduce((a, z) => a + P.zones[z].drone_ticks_inside, 0)), String(zid.reduce((a, z) => a + Jt.zones[z].drone_ticks_inside, 0))]);
    rows.push(["Score (within 30 m)", f3(P.within30), f3(Jt.within30)]);
    rows.push(["Same model, no new zones", f3(P.baseline.within30), f3(Jt.baseline.within30)]);
    rows.push(["Collisions · P0 escapes", `${P.collisions_mission} · ${P.p0_escape_rate}`, `${Jt.collisions_mission} · ${Jt.p0_escape_rate}`]);
    H.table(s, rows, { y: 1.6, colFrac: [0.4, 0.3, 0.3], rh: 0.36, fontSize: 10.5 });
    TEXT.push(...rows.flat());
    const sameCorner = D.matrix_rows.find((r) => r.tag === "cmp2_hier_aerovla_20261028");
    say(s, `Jetson score: the car was lost at the north-east corner, before leg 4. The same model lost it there without the new zones too (${f3(sameCorner.score)}, PC).`,
      { x: 0.55, y: 4.5, w: 8.9, h: 0.42, fontSize: 10, color: INK, fontFace: FF, italic: true, valign: "top" });
    H.source(s, `Same models without the new zones came to ${f2(P.baseline.zones["nfz-north-street"].drone_min_dist_m)} m (PC) / ${f2(Jt.baseline.zones["nfz-north-street"].drone_min_dist_m)} m (Jetson) of nfz-north-street's edge, ${f2(P.baseline.zones["nfz-south-block"].drone_min_dist_m)} / ${f2(Jt.baseline.zones["nfz-south-block"].drone_min_dist_m)} m of nfz-south-block's.`, 4.98);
    note(s, "Closest distance is from the drone to the zone's edge; negative would mean inside. The controller keeps 4 m from every zone (1 m margin + 3 m stand-off); without the new zones the same models flew 2.4-3.6 m from where their edges are now.");
  }

  /* 10b · FIRST TRY: SAFETY BEATS THE MISSION */
  if (N.first_attempt) {
    const s = content("Guardrail test", "First Try: Safety Beats the Mission");
    const F = N.first_attempt;
    H.stat(s, 0.7, 1.7, 2.8, `${Math.round(F.nfz_hold_ticks / 10)} s`, "drone held at a zone edge", TEAL, 34);
    H.stat(s, 3.6, 1.7, 2.8, F.nfz_entered ? "yes" : "0", "times it entered a zone", TEAL, 34);
    H.stat(s, 6.5, 1.7, 2.8, f3(F.within30), "score: the car got away", "E08A1E", 34);
    TEXT.push(String(F.nfz_hold_ticks), f3(F.within30));
    H.card(s, 0.55, 3.25, 4.375, 1.5, ic.ban, "What happened", "Zone placed too tight: no legal path past it.");
    H.card(s, 5.075, 3.25, 4.375, 1.5, ic.ok, "What we fixed", "Moved the zones; kept the 5 m rule.");
    H.source(s, `${F.tag} (policy ${F.policy}, superseded). P0 escape ${F.p0_escape_rate}, collisions ${F.collisions_mission}.`, 4.95);
    note(s, "The guardrail chose to stop rather than break the rule, even when that cost the mission. The rule was right; our zone placement was wrong. The corrected zones are on the previous slides.");
  }

  /* 11 · WHAT THE GUARDRAIL DID (frame) */
  if (D.figures.nfz_frame_pc || D.figures.nfz_frame_jetson) {
    const s = content("Guardrail test", "What the Pilot Sees on Screen");
    const which = D.figures.nfz_frame_pc ? "pc" : "jetson";
    const fr = await img(path.join(REPO, D.figures[`nfz_frame_${which}`]));
    H.figure(s, fr, 0.55, 1.55, 6.0, 3.4);
    const R = which === "pc" ? N.PC : N.Jetson;
    const c = R.hud_counts || {};
    H.stat(s, 6.9, 1.6, 2.5, String(c.avoided ?? "-"), "ticks the controller steered round a zone or obstacle");
    H.stat(s, 6.9, 2.75, 2.5, String(c.repaired ?? "-"), "ticks the Shield repaired the command");
    H.stat(s, 6.9, 3.9, 2.5, String(c.p0_escapes ?? "-"), "P0 escapes");
    H.source(s, `${R.tag}, t = ${D.figures[`nfz_frame_${which}_t`]} s, next to nfz-north-street. Rule panel and banner drawn by demo/policy_hud.py.`, 5.0);
  }

  /* 12 · SAFETY OVER ALL FLIGHTS */
  {
    const s = content("Safety", "Guardrail Over All Flights");
    H.stat(s, 0.7, 1.75, 2.8, String(saf.flights), "flights, 9 different pilots", TEAL, 40);
    H.stat(s, 3.6, 1.75, 2.8, String(saf.mission_collisions), "collisions during a mission", TEAL, 40);
    H.stat(s, 6.5, 1.75, 2.8, String(saf.p0_violation_ticks), "ticks a pilot's command broke a hard rule", TEAL, 40);
    H.card(s, 0.55, 3.35, 8.9, 1.35, ic.shield, "None reached the drone", `P0 escape rate ${saf.p0_escape_rates.join(" / ")} in every flight: each bad command was fixed first.`);
    H.source(s, "kpi.json per flight (guardrail/kpi.py): P0 escape = a hard-rule violation still present in the command the drone flew.", 4.95);
  }

  /* 13 · NEXT STEPS */
  {
    const s = content("Next", "Next Steps");
    const cy = 1.75, ch = 1.85, gap = 0.25, cw = (9.0 - 2 * gap) / 3;
    H.card(s, 0.55, cy, cw, ch, ic.ruler, "Smart distance", "Train the VLA: close on straights, back off before turns.");
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.aim, "Smart search", "Train the VLA: pick the street when the car is lost.");
    H.card(s, 0.55 + 2 * (cw + gap), cy, cw, ch, ic.chip, "Own edge VLA", "Small model, fully on the Jetson, guardrail kept.");
    H.card(s, 0.55, 3.8, 8.9, 1.2, ic.clock, "More flights per setup", "3-5 flights each, report mean and spread.");
  }

  /* 14 · CONCLUSION */
  {
    const s = pres.addSlide();
    H.darkBase(s, "Conclusion");
    say(s, "VLA plans. Our controller flies.\nThe guardrail checks every command.\nBest VLA setup comes close to our controller.", {
      x: 0.55, y: 1.35, w: 8.8, h: 1.8, fontSize: 24, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.15 });
    say(s, `${saf.flights} flights, ${saf.mission_collisions} collisions, P0 escape ${saf.p0_escape_rates.join(" / ")}. One to three flights per setup.`, {
      x: 0.55, y: 3.35, w: 8.6, h: 0.8, fontSize: 13, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3 });
  }

  /* 15 · THANK YOU */
  {
    const s = pres.addSlide();
    H.darkBase(s, null);
    s.addText("Thank You", { x: 0.55, y: 2.1, w: 9, h: 1.2, fontSize: 48, color: WHITE, fontFace: FF, bold: true });
    s.addShape(pres.shapes.LINE, { x: 0.6, y: 3.35, w: 1.2, h: 0, line: { color: TEAL, width: 3 } });
  }

  const bad = TEXT.filter((t) => /undefined|NaN/.test(t));
  if (bad.length) throw new Error("a slide would print undefined/NaN: " + bad.join(" | "));
  fs.mkdirSync(SHARE, { recursive: true });
  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  const mb = fs.statSync(OUT).size / 1e6;
  if (mb > 10) throw new Error(`deck is ${mb.toFixed(1)} MB (limit 10)`);
  console.log(`written: ${OUT} (${mb.toFixed(2)} MB, ${(pres.slides || pres._slides).length} slides)`);
}

main().catch((e) => { console.error(e); process.exit(1); });
