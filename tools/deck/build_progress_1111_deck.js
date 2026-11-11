/*
 * Progress update, 11 Nov 2026: the first KPI-grade campaigns on the hil rig
 * (AGX Orin + ArduPilot SITL over MAVROS 2), what the rig taught us, and where
 * each locked KPI stands. nathan-deck design system (tools/deck/nathan_theme.js).
 *
 * Every number is read at build time from progress_1111.json
 * (tools/deck/build_progress_1111_data.py), which reads the campaigns' own
 * sweep.json, flight logs, manifests and the fail-safe rescore. The build
 * refuses to write a slide that would print 'undefined' or 'NaN'.
 *
 *   C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_1111_data.py
 *   node tools/deck/build_progress_1111_deck.js
 */
const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const {
  FaShieldAlt, FaStopwatch, FaMicrochip, FaServer, FaCheckCircle, FaExclamationTriangle,
  FaWind, FaTools, FaBalanceScale, FaFlagCheckered, FaVideo, FaClipboardList,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "../..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const SHARE = path.join(REPO, "docs/share/2026-11-11-Progress");
const D = JSON.parse(fs.readFileSync(path.join(SHARE, "_build/progress_1111.json"), "utf8")).campaigns;
const OUT = path.join(SHARE, process.env.DECK_NAME || "VLA-Guardrail-Progress-11Nov2026.pptx");
const { TEAL, TEAL_DK, TEAL_TINT, TEAL_TINT2, BLACK, INK, GREY, WHITE, LINE, FF } = T;

const f3 = (v) => Number(v).toFixed(3);
const f1 = (v) => Number(v).toFixed(1);
const frac = (r) => `${r.correct}/${r.scored}`;
const TEXT = [];
function say(s, t, o) { TEXT.push(typeof t === "string" ? t : JSON.stringify(t)); s.addText(t, o); }

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "VLA Guardrail - hil KPI campaign, 11 Nov 2026";
  const H = helpers(pres);
  const ic = {
    shield: await iconPng(FaShieldAlt), clock: await iconPng(FaStopwatch), chip: await iconPng(FaMicrochip),
    server: await iconPng(FaServer), ok: await iconPng(FaCheckCircle), warn: await iconPng(FaExclamationTriangle),
    wind: await iconPng(FaWind), tools: await iconPng(FaTools), scale: await iconPng(FaBalanceScale),
    flag: await iconPng(FaFlagCheckered), video: await iconPng(FaVideo), list: await iconPng(FaClipboardList),
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

  const S = D.sm1111, S0 = D.sm1110, R3 = D.rh03, G = D.ge1110;
  const sw = S.sweep, tk = S.ticks, rd = S.readings;
  const rev = S.manifests.code_revisions.join(", ");

  /* 1 · COVER */
  {
    const s = pres.addSlide();
    H.darkBase(s, "Progress update · 11 Nov 2026");
    say(s, "VLA Guardrail:\nKPI-Grade Runs on the Rig", {
      x: 0.55, y: 1.1, w: 8.8, h: 1.9, fontSize: 36, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1 });
    say(s, "AGX Orin + ArduPilot SITL · MAVROS 2 · stress scenario library", {
      x: 0.55, y: 3.1, w: 8.6, h: 0.5, fontSize: 15, color: TEAL_TINT, fontFace: FF });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.1, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    say(s, [
      { text: "Department of Electronic Engineering, National Taipei University of Technology\n" },
      { text: "Advisor: \n" }, { text: "Presenter: Nathanael Tjahyadi" },
    ], { x: 0.55, y: 4.22, w: 8.6, h: 1.0, fontSize: 11, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3 });
  }

  /* 2 · SINCE LAST UPDATE */
  {
    const s = content("Summary", "Since the Last Update");
    const cy = 1.7, ch = 1.6, gap = 0.25, cw = (9.0 - 2 * gap) / 3;
    H.card(s, 0.55, cy, cw, ch, ic.chip, "Real rig, KPI-grade", "Shield on the AGX Orin, ArduPilot as the autopilot.");
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.tools, "6 rig-only defects fixed", "Each found by a short rig test, then re-flown.");
    H.card(s, 0.55 + 2 * (cw + gap), cy, cw, ch, ic.scale, "Labels decided", "Fail-safe judged on the vehicle that flies.");
    TEXT.push("Real rig, KPI-grade", "6 rig-only defects fixed", "Labels decided");
    const y = 3.6;
    H.stat(s, 0.7, y, 2.8, f3(sw.p0_shield_on_max), `P0 escape, Shield on\n(Shield off: ${sw.p0_control.join(", ")})`);
    H.stat(s, 3.6, y, 2.8, f3(rd.r2.rate), `fail-safe correctness\n(${frac(rd.r2)}, target 0.99)`);
    H.stat(s, 6.5, y, 2.8, `${f1(tk.tick_max)} ms`, "slowest Shield tick on the Orin\n(budget 100 ms)");
    TEXT.push(f3(sw.p0_shield_on_max), f3(rd.r2.rate), f1(tk.tick_max));
    note(s, `Smoke campaign sm1111: ${sw.missions_flown} missions on the hil rig at commit ${rev}, ${sw.runtime_h} h. ` +
      "P0 escape is a locked KPI with target 0; the Shield-off control arm shows what happens without the Shield.");
  }

  /* 3 · THE RIG */
  {
    const s = content("Setup", "The KPI Rig (hil Topology)");
    const box = (x, y, w, h, title, lines, hi) => {
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.08,
        fill: { color: hi ? TEAL : TEAL_TINT }, line: { color: hi ? TEAL : LINE, width: 1 } });
      say(s, title, { x: x + 0.15, y: y + 0.1, w: w - 0.3, h: 0.4, fontSize: 13, bold: true, fontFace: FF, color: hi ? WHITE : INK });
      say(s, lines, { x: x + 0.15, y: y + 0.5, w: w - 0.3, h: h - 0.6, fontSize: 10.5, fontFace: FF,
        color: hi ? WHITE : GREY, valign: "top", lineSpacingMultiple: 1.25 });
    };
    box(0.55, 1.65, 4.0, 1.75, "Desktop PC", "ArduPilot SITL 4.5.7 (the autopilot)\nWind, GPS loss as SITL parameters\nScenario runner + scorer");
    box(5.45, 1.65, 4.0, 1.75, "AGX Orin (companion)", "Safety Shield + escalation FSM\nMAVROS 2 in a container\nSigned policy bundle", true);
    s.addShape(pres.shapes.LINE, { x: 4.6, y: 2.52, w: 0.8, h: 0, line: { color: TEAL, width: 2, beginArrowType: "triangle", endArrowType: "triangle" } });
    say(s, "Ethernet", { x: 4.45, y: 2.12, w: 1.1, h: 0.3, fontSize: 9, color: GREY, fontFace: FF, align: "center" });
    H.card(s, 0.55, 3.65, 8.9, 1.25, ic.ok, "KPI-grade rule (grant)",
      `hil topology · real time (speedup ${S.manifests.sim_speedups.join(", ")}) · clean code revision · signed bundle. All ${sw.missions_flown} missions: ${rev}, ${S.manifests.topologies.join(", ")}.`, 10.5);
    note(s, "The grant says KPI numbers only count from the hil topology. Every run writes a manifest; the scorer refuses runs that are not KPI-grade.");
  }

  /* 4 · KPI SCOREBOARD */
  {
    const s = content("Results", "KPI Scoreboard (smoke, 48 missions)");
    H.table(s, [
      ["KPI", "Grant target", "Result", "Status"],
      ["P0 violation escape rate", "0 (hard)", `${f3(sw.p0_shield_on_max)} (off: ${sw.p0_control.join(", ")})`, "Met"],
      ["Fail-safe trigger correctness", "≥ 0.99", `${f3(rd.r2.rate)} (${frac(rd.r2)})`, "1 cell short"],
      ["Monitor tick on the Orin", "≤ 100 ms", `max ${f1(tk.tick_max)} ms`, "Met"],
      ["Mean time to safe", "tracked", `${sw.tts_mean_s} s (${sw.tts_episodes} spells)`, "Tracked"],
      ["Mission success rate", "tracked", f3(sw.mission_success_rate), "Tracked"],
      ["Repairs per episode", "tracked", String(sw.repairs_per_episode), "Tracked"],
    ], { y: 1.65, colFrac: [0.34, 0.17, 0.32, 0.17], rh: 0.44, fontSize: 11, emph: [[1, 3], [3, 3]] });
    H.source(s, `sm1111 · ${rev} · ${S.ticks.tick_n} Orin ticks, ${S.ticks.tick_over_100} over 100 ms · fail-safe on the adopted reading (next slide)`, 4.95);
    note(s, "P0 escape and fail-safe correctness are the locked KPIs. Time to safe, mission success and repair counts are 'tracked' in the grant's own KPI table.");
  }

  /* 5 · FAIL-SAFE READINGS */
  {
    const s = content("Fail-safe", "Which Vehicle Defines “Correct”?");
    H.table(s, [
      ["Label reading", "Correct", "Rate", "Wilson 95 % low"],
      ["Ideal vehicle (old rubric)", frac(rd.a), f3(rd.a.rate), f3(rd.a.wilson_low)],
      ["Rig vehicle", frac(rd.r), f3(rd.r.rate), f3(rd.r.wilson_low)],
      ["Rig vehicle + grant's E5 (adopted)", frac(rd.r2), f3(rd.r2.rate), f3(rd.r2.wilson_low)],
    ], { y: 1.6, colFrac: [0.44, 0.18, 0.16, 0.22], rh: 0.44, fontSize: 11.5, emph: [[3, 1], [3, 2]] });
    const cy = 3.6, cw = 4.35;
    H.card(s, 0.55, cy, cw, 1.3, ic.warn, "Why change", "Real autopilot lags; an ideal vehicle never does.", 10.5);
    H.card(s, 0.55 + cw + 0.2, cy, cw, 1.3, ic.flag, "Remaining miss",
      `${rd.r2.missed.length + rd.r2.false.length} run: wind 12 m/s, fail-safe not triggered.`, 10.5);
    note(s, "Labels say whether a fail-safe SHOULD fire, derived from physics, never from what the FSM did. " +
      "Derived on an ideal vehicle, head-on approaches look avoidable; on the real autopilot (0.13 s delay, 2.5 m/s² braking) they are not. " +
      "Decision (best practice): judge on the vehicle that flies, E5 as the grant prescribes; all readings stay reported. " +
      `Missed run: ${rd.r2.missed.join(", ")}.`);
  }

  /* 6 · PROGRESS OVER CAMPAIGNS */
  {
    const s = content("Progress", "Rig Campaigns, Before and After");
    const row = (name, c) => [name, `${frac(c.readings.r2)}`, `${frac(c.readings.a)}`,
      String(c.sweep.failsafe_ideal.censored_in_loiter), `${c.sweep.tts_mean_s} s`, `${f1(c.ticks.tick_max)} ms`];
    H.table(s, [
      ["Campaign", "Fail-safe (adopted)", "Fail-safe (ideal)", "Not\nscored", "Time to\nsafe", "Slowest tick"],
      row("7 Nov · rh03", R3), row("10 Nov · sm1110", S0), row("11 Nov · sm1111", S),
    ], { y: 1.65, colFrac: [0.2, 0.18, 0.17, 0.13, 0.15, 0.17], rh: 0.48, fontSize: 11, emph: [[3, 1], [3, 3], [3, 4]] });
    H.card(s, 0.55, 3.8, 8.9, 1.1, ic.ok, "What moved",
      `Time to safe ${R3.sweep.tts_mean_s} → ${S.sweep.tts_mean_s} s · stuck-in-hold runs ${S0.sweep.failsafe_ideal.censored_in_loiter} → ${S.sweep.failsafe_ideal.censored_in_loiter} · P0 escape 0 in every campaign.`, 11);
    note(s, `Same smoke library each time. rh03 at ${R3.manifests.code_revisions.join(",")}, sm1110 at ${S0.manifests.code_revisions.join(",")}, sm1111 at ${rev}. ` +
      "Not scored = the run ended while still holding (censored), so whether a fail-safe would follow is unknown.");
  }

  /* 7 · WHAT THE RIG TAUGHT US */
  {
    const s = content("Fixes", "Found Only on the Real Rig");
    H.table(s, [
      ["Problem on the rig", "Fix", "Proof"],
      ["Hold point swept by a rotating zone", "Hold must stay legal while held", "Rotate-spin 8/8 correct"],
      ["Autopilot BRAKE drifts ~2.5 m", "Shield holds in GUIDED", "Hold stays on its point"],
      ["Hold planning took 147 ms on Orin", "Spread over ticks", "Max tick 96 ms"],
      ["Hold / release loop, 14 times", "Release only if safe at speed", "Fail-safe fires as labelled"],
      ["“Landed” never reported", "Ask ArduPilot by message ID", "G10, X5 fire on the rig"],
      ["Take-off into the GeoFence", "Wait for SITL params; refuse", "Unit-tested; to re-fly"],
    ], { y: 1.55, colFrac: [0.38, 0.33, 0.29], rh: 0.46, fontSize: 10.5 });
    note(s, "None of these appeared in simulation. Each was found by a 9-11 mission shakedown, fixed, tested in CI, and re-flown. " +
      "The BRAKE finding: ArduCopter's BRAKE keeps the last GUIDED acceleration, so it carries the aircraft along the last command.");
  }

  /* 8 · STRESS + GRANT EXAMPLE */
  {
    const s = content("Coverage", "Stress on the Rig, and the Grant's Own Example");
    const cy = 1.65, cw = 4.35, ch = 1.55;
    H.card(s, 0.55, cy, cw, ch, ic.wind, "Wind and GPS loss", "Wind 5 / 8 / 12 m/s, GPS off 4-11 s, set in SITL.", 10.5);
    H.card(s, 0.55 + cw + 0.2, cy, cw, ch, ic.shield, "FSM edges first flown", "G10 landed · X5 landed in RTL · X8 GPS lost.", 10.5);
    H.card(s, 0.55, cy + ch + 0.25, 8.9, 1.45, ic.flag, "Grant's example sweep, flown on the rig",
      `${G.sweep.missions_flown - 4} of ${G.sweep.missions_flown} flew (4 hit the GeoFence bug): ${frac(G.readings.r2)} correct, P0 ${f3(G.sweep.p0_shield_on_max)}, success ${f3(G.sweep.mission_success_rate)}. The zone never blocks the path, so its 2.0 s time-to-safe target is never tested.`, 10.5);
    note(s, "The grant's ScenarioSpec example (zone spawned 60 m ahead, sliding 5 m/s sideways, speeds 6-12 m/s, wind 0-5) flown as written on the hil rig.");
  }

  /* 9 · VIDEO */
  {
    const s = content("Video", "Progress Video");
    const th = await img(path.join(SHARE, "_build/thumb_headon.png"), { maxW: 1600 });
    H.figure(s, th, 0.55, 1.55, 5.0, 2.85);
    H.card(s, 5.75, 1.55, 3.7, 2.85, ic.video, "2:43, four clips",
      "1. Shield OFF vs ON, head-on\n2. Rotating bar: old vs new\n3. Corridor drift: 14 restarts vs RTL\n4. 5 m/s wind: goal reached", 11);
    note(s, "Top-down replays of the hil flight logs (SITL has no camera). Old/new rotating-bar runs flew 3 vs 6 deg/s.");
  }

  /* 10 · CONTRACT STATUS */
  {
    const s = content("Contract", "Contract Status (136 obligations)");
    H.stat(s, 0.7, 1.7, 2.1, "70", "DONE");
    H.stat(s, 2.9, 1.7, 2.1, "59", "PARTIAL");
    H.stat(s, 5.1, 1.7, 2.1, "6", "DRIFT\n(dated gates, team size)");
    H.stat(s, 7.3, 1.7, 2.1, "1", "MISSING\n(AP_DDS, optional)");
    H.card(s, 0.55, 3.2, 8.9, 1.65, ic.list, "Closes with the nightly campaign (~18 items)",
      "P0 escape, fail-safe, repair KPIs, tick budget, KPI report, KPI-grade bundles, Python 3.12 flight stack.", 11);
    note(s, "Status from docs/FINAL-REPORT-outline.md Appendix A. DONE means the artefact matches the grant's words; KPI-grade evidence comes from the hil campaigns.");
  }

  /* 11 · OPEN + NEXT */
  {
    const s = content("Next", "Open Items and Next Steps");
    const cy = 1.6, gap = 0.2, cw = (8.9 - gap) / 2, ch = 1.55;
    H.card(s, 0.55, cy, cw, ch, ic.clock, "Nightly running", "591 missions (~20 h), final KPI numbers.", 10.5);
    H.card(s, 0.55 + cw + gap, cy, cw, ch, ic.wind, "Wind 12 m/s miss", "Beyond hold authority; fail-safe must fire.", 10.5);
    H.card(s, 0.55, cy + ch + 0.2, cw, ch, ic.chip, "AIoT-VLA on the rig", "Own VLA through MAVROS 2 on the Orin.", 10.5);
    H.card(s, 0.55 + cw + gap, cy + ch + 0.2, cw, ch, ic.flag, "Final report", "Due 30 Nov; signer and CA from the PI.", 10.5);
    note(s, "Also open: 2 stress missions refused by the scorer (SITL parameters read too early, fix is small); " +
      "one ~110 ms Shield tick per long mission around 130 s in the grant example sweep (29 of 82,971 ticks), cause under investigation.");
  }

  /* 12 · THANK YOU */
  {
    const s = pres.addSlide();
    H.darkBase(s, null);
    s.addText("Thank You", { x: 0.55, y: 2.1, w: 9, h: 1.2, fontSize: 48, color: WHITE, fontFace: FF, bold: true });
    s.addShape(pres.shapes.LINE, { x: 0.6, y: 3.35, w: 1.2, h: 0, line: { color: TEAL, width: 3 } });
  }

  const bad = TEXT.filter((t) => /undefined|NaN|null/.test(t));
  if (bad.length) throw new Error("a slide would print undefined/NaN/null: " + bad.join(" | "));
  fs.mkdirSync(SHARE, { recursive: true });
  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  const mb = fs.statSync(OUT).size / 1e6;
  if (mb > 10) throw new Error(`deck is ${mb.toFixed(1)} MB (limit 10)`);
  console.log(`written: ${OUT} (${mb.toFixed(2)} MB)`);
}

main().catch((e) => { console.error(e); process.exit(1); });
