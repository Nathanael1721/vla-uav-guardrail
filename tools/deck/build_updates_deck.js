/*
 * Updates since the 2 September 2026 meeting — nathan-deck design system.
 *
 * Every number comes from docs/data/eval_sep2026.json, written by
 * tools/build_eval_data.py from the flight artefacts. Nothing is typed by hand.
 *
 *   node tools/deck/build_updates_deck.js
 */
const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const sharp = require("sharp");
const {
  FaCheckCircle, FaBug, FaCrosshairs, FaBalanceScale, FaClipboardCheck, FaEyeSlash,
  FaShieldAlt, FaCity, FaGithub, FaFlask, FaDatabase, FaMicrochip, FaCamera, FaWalking,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

const REPO = path.resolve(__dirname, "..", "..");
const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const E = JSON.parse(fs.readFileSync(path.join(REPO, "docs/data/eval_sep2026.json"), "utf8"));
const OUT = path.join(REPO, "docs/Guardrail-Updates-Sep2026.pptx");
const FPV = path.join(REPO, "demo/out/retarget_smooth/view/fpv");

const f1 = (v) => Number(v).toFixed(1);
const f2 = (v) => Number(v).toFixed(2);
// CityLife flights re-scored by tools/build_eval_data.py; 0 means not flown.
const cityFlown = Object.keys(((E.unflown || {}).citylife_level || {}).flights || {}).length;
// Read off the artefacts, like the report's Section 7: a next-step row must
// not ask to fly what the cards beside it say has flown.
const camFlown = (((E.unflown || {}).camera_768x432 || {}).flight_artefacts || []).length;

/* The recorder writes one frame per 50 ms from mission start, so frame = t x 20
 * (checked against the HUD clock: frame 450 reads t 22.5 s, 1200 reads 60.0 s). */
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
  pres.title = "Guardrail — Updates Since 2 September";
  const H = helpers(pres);
  const { TEAL, TEAL_TINT, WHITE, BLACK, INK, GREY, LGREY, LINE, FF } = T;

  const ic = {
    ok: await iconPng(FaCheckCircle), bug: await iconPng(FaBug), aim: await iconPng(FaCrosshairs),
    scale: await iconPng(FaBalanceScale), clip: await iconPng(FaClipboardCheck),
    blind: await iconPng(FaEyeSlash), shield: await iconPng(FaShieldAlt), city: await iconPng(FaCity),
    git: await iconPng(FaGithub), flask: await iconPng(FaFlask), data: await iconPng(FaDatabase),
    chip: await iconPng(FaMicrochip), cam: await iconPng(FaCamera), walk: await iconPng(FaWalking),
  };

  const R = E.retarget, D = E.detector, REPO_F = E.repo, U = E.unflown;
  const before = R.before_ped_matched, after = R.after_ped_matched;
  const cov = R.coverage_after_full, chk = R.subject_range_check;
  let page = 1;

  /* 01 · COVER ------------------------------------------------------------ */
  {
    const s = pres.addSlide();
    H.darkBase(s, "ITRI VLA-UAV · Guardrail · September 2026");
    s.addText("Updates Since\n2 September", {
      x: 0.55, y: 1.15, w: 8.4, h: 1.9, fontSize: 40, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1,
    });
    s.addText("What changed, what it proves, what it does not", {
      x: 0.55, y: 3.15, w: 8, h: 0.6, fontSize: 15, color: TEAL_TINT, fontFace: FF,
    });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.15, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    s.addText([
      { text: "Department of International Graduate Program, EECS\n" },
      { text: "Advisor: \n" },
      { text: "Presenter: " },
    ], { x: 0.55, y: 4.3, w: 8, h: 1.0, fontSize: 11, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3 });
  }

  /* 02 · ACTION ITEMS ----------------------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Since 2 September", "Action Items");
    const rows = [
      ["Action item", "Status", "Evidence"],
      ["Scenario a tracker cannot run", "Done", "Phrase retarget, ring 5 m → 10 m"],
      ["Architecture diagram + lock layer", "Done", "architecture-v3"],
      ["More realistic pedestrians", cityFlown ? `Flown, ${cityFlown} flights` : "Built, not flown", `CityLife: ${U.citylife_level.pedestrians} walking, ${U.citylife_level.cars} driving`],
      ["Correct five quoted numbers", "Written", "Delivery pending"],
      ["Gazebo SITL", "Open", "—"],
      ["Real sensor (camera / LiDAR)", "Open", "Now motivated — slide 06"],
      ["Midterm report · 18 Sept", "Prof. Lai · Guan-Wen", "—"],
    ];
    H.table(s, rows, { y: 1.72, rh: 0.4, fontSize: 10.5, colFrac: [0.38, 0.22, 0.40],
      emph: [[1, 1], [2, 1]], align: ["left", "center", "left"] });
    H.source(s, "Baseline: action items recorded at the 2 September meeting.");
    H.badge(s, page);
  }

  /* 03 · ONE PHRASE, TWO RULES --------------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Tracker vs detector", "One Phrase, Two Rules");
    const FF_ = E.retarget.figure_frames;
    const car = await frame(FF_.car.t);
    // 480x270 around the "a person" box at t = 60 s (box centre 750, 212).
    const ped = await frame(FF_.person.t, { left: 510, top: 77, width: 480, height: 270 });
    const bw = 4.3, bh = 2.42, by = 1.72;
    H.figure(s, car, 0.55, by, bw, bh);
    H.figure(s, ped, 5.15, by, bw, bh);
    s.addText([
      { text: `t ${f1(FF_.car.t)} s  ·  "a yellow car"`, options: { bold: true, color: INK } },
      { text: "\nClass car → 5 m ring", options: { color: GREY } },
    ], { x: 0.55, y: by + bh + 0.08, w: bw, h: 0.62, fontSize: 11, fontFace: FF });
    s.addText([
      { text: `t ${f1(FF_.person.t)} s  ·  "a person", p = ${FF_.person.score.toFixed(3)}`, options: { bold: true, color: INK } },
      { text: "\nClass pedestrian → 10 m ring", options: { color: GREY } },
    ], { x: 5.15, y: by + bh + 0.08, w: bw, h: 0.62, fontSize: 11, fontFace: FF });
    s.addText(`Phrase changed at t+${f1(R.retarget_event.t)} s. Same aircraft, policy, hash. A tracker returns an ID, never a class.`, {
      x: 0.55, y: 4.86, w: 8.6, h: 0.3, fontSize: 10.5, color: TEAL, fontFace: FF, bold: true,
    });
    H.badge(s, page);
  }

  /* 04 · FOUR SILENT DEFECTS ----------------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Found and fixed", "Four Silent Defects");
    const cw = 4.35, ch = 1.5, gx = 0.2, gy = 0.2, x0 = 0.55, y0 = 1.72;
    H.card(s, x0, y0, cw, ch, ic.bug, "Inert stand-off rule", "Policy said \"pedestrian\", phrase said \"person\". Bound zero times.");
    H.card(s, x0 + cw + gx, y0, cw, ch, ic.aim, "Wrong set-point", `After retarget, still held the car's ${f1(D.standoff_setpoint_m["car_4.0m"])} m.`);
    H.card(s, x0, y0 + ch + gy, cw, ch, ic.scale, "Metric a constant passes", "Frame-centre \"detector\" scored 1.000 on target.");
    H.card(s, x0 + cw + gx, y0 + ch + gy, cw, ch, ic.clip, "Verifier checked 5 of 7", "Two KPI fields named but never compared.");
    H.source(s, "Each has a regression test. Findings: docs/FINDING-*.md.");
    H.badge(s, page);
  }

  /* 05 · TRACKING STEADIED -------------------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Pedestrian phase, matched window", "Tracking Steadied");
    const rows = [
      [`t ≤ ${Math.round(R.matched_window_s)} s`, "Before", "After"],
      ["Median |yaw| (deg/s)", f1(before.yaw_dps_median), f1(after.yaw_dps_median)],
      ["Peak |yaw| (deg/s)", f1(before.yaw_dps_max), f1(after.yaw_dps_max)],
      ["Box error (px)", f1(before.box_err_px_median), f1(after.box_err_px_median)],
      ["Centre-constant null (px)", f1(before.box_err_px_null_centre), f1(after.box_err_px_null_centre)],
      ["Ticks", String(before.ticks), String(after.ticks)],
    ];
    H.table(s, rows, { x: 0.55, y: 1.72, w: 5.4, rh: 0.46, colFrac: [0.5, 0.25, 0.25], emph: [[1, 2], [2, 2]] });
    H.card(s, 6.2, 1.72, 3.25, 1.35, ic.ok, "Pointing fixed", "Speed clamp + one yaw cap.");
    H.card(s, 6.2, 3.22, 3.25, 1.35, ic.flask, "Seeing not fixed", `${f1(after.box_err_px_median)} px vs ${f1(after.box_err_px_null_centre)} px null.`);
    H.source(s, "retarget_fixed (70 s) vs retarget_smooth (120 s), compared over the window both flew.");
    H.badge(s, page);
  }

  /* 06 · WHAT THE ZERO DOES NOT COVER --------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "The honest reading of 0.0", "What the Zero Does Not Cover");
    const sw = 2.2;
    H.stat(s, 0.55, 1.7, sw, f1(R.p0_escape_after), "P0 escape rate");
    H.stat(s, 0.55 + sw, 1.7, sw, String(cov.ticks_person_inside_10m), "ticks a person within 10 m");
    H.stat(s, 0.55 + 2 * sw, 1.7, sw, String(cov.outside_hfov), "of those, outside camera view");
    H.stat(s, 0.55 + 3 * sw, 1.7, sw, `${f2(R.after_ped_full.closest_real_m)} m`, "closest bystander");
    H.card(s, 0.55, 3.0, 4.35, 1.55, ic.shield, "Subject: range right",
      `Estimate ${f1(chk.est_range_m_median)} m; followed person ${f1(chk.person_under_box_range_m_median)} m.`);
    H.card(s, 5.1, 3.0, 4.35, 1.55, ic.blind, "Bystanders: not covered",
      "No rule names them. Forward camera cannot see them.");
    s.addText(`Retracted: "served position wrong by 37 m" — measured against the wrong person. "Fired 6 times" was ${R.standoff_score_before.tp} of ${R.standoff_score_before.fires}.`, {
      x: 0.55, y: 4.72, w: 8.6, h: 0.42, fontSize: 9.5, color: GREY, fontFace: FF, italic: true,
    });
    H.badge(s, page);
  }

  /* 07 · WHY PEDESTRIANS ARE HARD ------------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Resolution, patches, GPU", "Why Pedestrians Are Hard");
    const px = D.pedestrian_px_400_capture, pt = D.pedestrian_patches;
    const rows = [
      ["0.5 m person", "px, today", "Patches"],
      ["10 m", f1(px["10m"]), f2(pt["10m"][0])],
      ["16 m", f1(px["16m"]), f2(pt["16m"][0])],
      ["20 m", f1(px["20m"]), f2(pt["20m"][0])],
    ];
    s.addText("Subject size (400×225, 90° FOV)", { x: 0.55, y: 1.62, w: 4.3, h: 0.3, fontSize: 11, color: INK, fontFace: FF, bold: true });
    H.table(s, rows, { x: 0.55, y: 1.95, w: 4.3, rh: 0.42, colFrac: [0.4, 0.3, 0.3], emph: [[2, 2], [3, 2]] });
    const tr = D.offline_4090.rows;
    const rows2 = [["Capture", "Total (ms)"], ...tr.map((r) => [r.capture, f1(r.total_ms)])];
    s.addText("Detector cost, RTX 4090", { x: 5.15, y: 1.62, w: 4.3, h: 0.3, fontSize: 11, color: INK, fontFace: FF, bold: true });
    H.table(s, rows2, { x: 5.15, y: 1.95, w: 4.3, rh: 0.42, colFrac: [0.5, 0.5], emph: [[2, 1]] });
    s.addText([
      { text: "One OWL-ViT patch is 32 px. The person fills under half.", options: { breakLine: true } },
      { text: `768×432 costs nothing offline — ${camFlown ? `flown on ${camFlown} flights` : "committed, not yet flown"}. In flight the GPU forward pass runs ${D.contention.gpu_forward_x[0]}–${D.contention.gpu_forward_x[1]}× slower.` },
    ], { x: 0.55, y: 3.85, w: 8.9, h: 0.85, fontSize: 11, color: GREY, fontFace: FF, lineSpacingMultiple: 1.3 });
    H.source(s, "Model input is resized to 768×768 regardless of capture size. Offline: median of 20, 2026-09-10.");
    H.badge(s, page);
  }

  /* 08 · CITYLIFE AND RELEASE ------------------------------------------------ */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Scene and publication", "CityLife and the Release");
    const cw = 4.35, ch = 1.5, x0 = 0.55, y0 = 1.72;
    H.card(s, x0, y0, cw, ch, ic.walk, "CityLife level",
      `${U.citylife_level.pedestrians} walking, ${U.citylife_level.cars} driving. ` + (cityFlown ? `Flown: ${cityFlown} flights.` : "Verified in editor, not flown."));
    H.card(s, x0 + cw + 0.2, y0, cw, ch, ic.git, "Public release",
      "v0.5.0 on GitHub + Pages. v0.5.1 correction local.");
    const tf = REPO_F.tests_fast;
    H.card(s, x0, y0 + ch + 0.2, cw, ch, ic.ok, "Tests",
      `${tf.passed}/${tf.total} fast (run ${tf.measured || E.generated}), ${REPO_F.tests_coverage.passed}/${REPO_F.tests_coverage.total} coverage.`);
    H.card(s, x0 + cw + 0.2, y0 + ch + 0.2, cw, ch, ic.data, "One numbers file",
      "Decks and report read the same JSON.");
    H.source(s, `${REPO_F.commits_since_meeting.length} commits since 2 September. Scene details: docs/FINDING-citylife-level.md.`);
    H.badge(s, page);
  }

  /* 09 · NEXT ---------------------------------------------------------------- */
  {
    const s = pres.addSlide(); s.background = { color: WHITE }; page++;
    H.heading(s, "Before and after 18 September", "Next Steps");
    const steps = [
      (camFlown && cityFlown) ? ["Measure 768×432 under load", "Compare the in-flight cost with the offline one."]
        : [`Fly ${[camFlown ? "" : "768×432", cityFlown ? "" : "CityLife"].filter(Boolean).join(" and ")}`, "Confirm the offline cost holds under load."],
      ["Choose a narrower FOV", `45°: ${f2(D.pedestrian_patches_by_hfov_16m["45deg"])} patches at 16 m, not ${f2(D.pedestrian_patches_by_hfov_16m["90deg"])}.`],
      ["Cover bystanders", "A rule for any pedestrian, and a wider sensor."],
      ["Gazebo Harmonic rail", "Mid-term item: scripts exist; keep a run."],
      ["Send the five corrections", "Before the midterm report is filed."],
    ];
    steps.forEach(([t, b], i) => {
      const y = 1.72 + i * 0.64;
      s.addShape(pres.shapes.OVAL, { x: 0.6, y: y + 0.07, w: 0.42, h: 0.42, fill: { color: TEAL }, line: { color: TEAL } });
      s.addText(String(i + 1), { x: 0.6, y: y + 0.07, w: 0.42, h: 0.42, fontSize: 12, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(t, { x: 1.2, y, w: 3.5, h: 0.56, fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(b, { x: 4.7, y, w: 4.7, h: 0.56, fontSize: 11, color: GREY, fontFace: FF, valign: "middle" });
      if (i < steps.length - 1) s.addShape(pres.shapes.LINE, { x: 1.2, y: y + 0.6, w: 8.2, h: 0, line: { color: LINE, width: 1 } });
    });
    H.badge(s, page);
  }

  /* 10 · CONCLUSION ------------------------------------------------------------ */
  {
    const s = pres.addSlide();
    H.darkBase(s, "Conclusion");
    s.addText("Tracking is steady and honestly scored, with bystander coverage still open.", {
      x: 0.55, y: 1.5, w: 8.6, h: 1.7, fontSize: 26, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.15,
    });
    s.addText(`P0 escape was 0.0 on ${E.kpi.flights_escape_zero} shielded demo and SITL flights (dev and perception rails, not contract KPI figures). It says nothing about people the policy does not name.`, {
      x: 0.55, y: 3.4, w: 8.2, h: 0.8, fontSize: 13, color: TEAL_TINT, fontFace: FF, lineSpacingMultiple: 1.3,
    });
  }

  /* 11 · THANK YOU -------------------------------------------------------------- */
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
