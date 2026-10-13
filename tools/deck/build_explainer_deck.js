/*
 * The Guardrail explained end to end, with the progress of 9-14 October 2026:
 * the results of 9-13 October and the close approach recorded on 14 October.
 * Audience: the PI and ITRI. nathan-deck design system (tools/deck/nathan_theme.js).
 *
 *   node tools/deck/build_explainer_deck.js                 # the pack, fonts embedded
 *   node tools/deck/build_explainer_deck.js --render        # + every slide as PNG (_build/render/)
 *   node tools/deck/build_explainer_deck.js --pdf           # + the PDF (tools/office_to_pdf.ps1)
 *   node tools/deck/build_explainer_deck.js --out DIR --no-office   # a test build: no PowerPoint
 *   node tools/deck/build_explainer_deck.js --check-facts-only      # check the facts and exit
 *
 * Optional slides take images; a missing image drops its slide, nothing else:
 *   --still PATH            frame of the 13 Oct city video (default: the pack's figures/ copy,
 *                           extracted from the video with ffmpeg when the video is on disk)
 *   --onoff-on PATH --onoff-off PATH
 *                           ArduPilot close approach, Shield on and Shield off (default:
 *                           figures/ardupilot_shield_on.(png|jpg) and _off in the pack)
 *
 * NUMBERS. Every number on a slide is a FACT below: its printed value and the
 * file it comes from. At build time each fact is checked against the first of
 * its source files that exists (a JSON field, or a phrase in a text file); a
 * mismatch stops the build. A fact whose files are all absent (flight output
 * under the gitignored demo/out on another machine) is reported as unverified;
 * the pack build refuses that unless --allow-unverified is given. Each slide
 * footer names its sources.
 *
 * WORDING. Every slide's text and notes go to _build/slides_text.json and
 * through tools/check_claims.py (scan_text) and a hedge-word scan; a hit stops
 * the build. tests/test_explainer_deck.py repeats both scans on the written
 * .pptx and adds its own list of claims this project cannot make (no KPI-grade,
 * hil, Orin or drone result exists).
 *
 * The pack build also copies the explainer figures into the pack, writes the
 * number tables of CHEATSHEET-EN.md / CHEATSHEET-ID.md from the same facts
 * (between their "numbers" markers), and writes the pack README.
 */
const fs = require("fs");
const path = require("path");
const crypto = require("crypto");
const { execFileSync } = require("child_process");
const pptxgen = require("pptxgenjs");
const sharp = require("sharp");
const {
  FaShieldAlt, FaFileSignature, FaRandom, FaClipboardList, FaCheckCircle, FaPlane,
  FaCogs, FaTachometerAlt, FaNetworkWired, FaClock, FaHistory, FaSyncAlt, FaVideo,
  FaCamera, FaBrain, FaArrowsAlt, FaPauseCircle, FaLayerGroup,
} = require("react-icons/fa");
const { T, iconPng, img, helpers, repairAndAudit } = require("./nathan_theme");

/* ---- Arguments, paths ------------------------------------------------------ */
const argv = process.argv.slice(2);
const flag = (n) => argv.includes(n);
const opt = (n) => { const i = argv.indexOf(n); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : null; };
// EXPLAINER_REPO points the fact check at another tree (tests use a copy with one number changed).
const REPO = process.env.EXPLAINER_REPO ? path.resolve(process.env.EXPLAINER_REPO) : path.resolve(__dirname, "..", "..");
const PY_DEFAULT = "C:/Users/natha/.conda/envs/vla-real/python.exe";
const PY = process.env.EXPLAINER_PY || (fs.existsSync(PY_DEFAULT) ? PY_DEFAULT : "python");
const PACK = path.join(REPO, "docs/share/2026-10-14-Explainer");
const OUT_DIR = opt("--out") ? path.resolve(opt("--out")) : PACK;
const PACK_BUILD = OUT_DIR === PACK;            // only the pack build writes into the pack
const BUILD = path.join(OUT_DIR, "_build");
const DECK_NAME = "Guardrail-Explainer-13Oct2026";
const OUT = path.join(OUT_DIR, `${DECK_NAME}.pptx`);
const NO_OFFICE = flag("--no-office");
const ALLOW_UNVERIFIED = flag("--allow-unverified") || !PACK_BUILD;
const FIG_DIR = path.join(PACK, "figures");
const MAX_MB = 25;
const LAB = "NTUT AIoT Lab";
const STATUS_DATE = "14 October 2026", STATUS_DAY = "14 October";
const BUILDER_SHA = crypto.createHash("sha256").update(fs.readFileSync(__filename)).digest("hex");

/* The 13 Oct city flight and the frame used from its video. The HUD's own
 * clock reads 47.6 s at video time 49.6 s (read off the frame); at that tick
 * the flight log's fence distance is 4.0 m, the closest of the flight. */
const DEMO_TAG = "citylife_redcar_nfz_20261013";
const DEMO_DIR = path.join(REPO, "demo/out", DEMO_TAG);
const DEMO_VIDEO = path.join(DEMO_DIR, `${DEMO_TAG}_demo.mp4`);
const DEMO_SHARE_VIDEO = `${DEMO_TAG}_demo_share.mp4`;
const STILL_VIDEO_T = "49.6", STILL_HUD_T = "47.6";
const STILL_FULL = path.join(FIG_DIR, `${DEMO_TAG}_t${STILL_HUD_T}s_full.jpg`);
const STILL_FPV = path.join(FIG_DIR, `${DEMO_TAG}_t${STILL_HUD_T}s_fpv.jpg`);

/* The ArduPilot pair's stills, each the left (drone camera + HUD) 1280 x 720
 * panel of its run's _demo.mp4, extracted with
 *   ffmpeg -ss <t> -i <run>_demo.mp4 -frames:v 1 -vf crop=1280:720:0:0 figures/ardupilot_shield_<on|off>.png
 * Shield on at 41.15 s (HUD clock 40.8 s): FSM Brake, the banner "pushed away
 * from obstacle (bld-clearance)", the detector's box over the top of the red
 * traffic signal, its lit lamp directly below the box (at 39.0 s, before the first
 * correction, the box sat on the signal head beside the lit lamp). */
const ONOFF_ON_T = "41.15", ONOFF_ON_HUD_T = "40.8", ONOFF_OFF_T = "60.7";

const FIGS = ["explainer_flowchart_end_to_end", "explainer_arch_components", "explainer_arch_shield",
  "explainer_arch_topologies", "explainer_arch_evidence"];
const figPng = (n) => path.join(REPO, "docs/img", `${n}.png`);

/* ---- Facts ------------------------------------------------------------------ */
/* src(file, phrase): the phrase must occur in the file (whitespace-tolerant).
 * src(file, getter, expected): getter(JSON) must equal expected; for a .jsonl
 * file the getter receives the array of its rows. */
const reEsc = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&").replace(/\s+/g, "\\s+");
const src = (file, a, b) => (typeof a === "string" ? { file, phrase: a } : { file, get: a, eq: b });
const FACTS = {};
function fact(id, v, sources, meta = {}) { FACTS[id] = { id, v, sources, ...meta, status: null, file: null }; }
const CL = "CHANGELOG.md", SW = "docs/data/scenario_sweep.json", PO = "docs/data/progress_oct.json";
const ON = "demo/out/pas_ap_close_on", OFF = "demo/out/pas_ap_close_off";
const GRANT = "kuanting-vla-uav-guardrail/docs";
const all = (o) => o._rollup._all_shield_on;

// Close approach, 13 Oct (later): Shield on and off, same pilot and route.
// A P0 escape is a tick on which the Shield FLEW a command that breaks a P0 rule
// (guardrail/kpi.py). With the Shield on, 165 of the 532 ticks were flown by
// ArduPilot itself (the FSM's BRAKE hold, then RTL); kpi.py scores those apart.
fact("on_ticks", "532", [src(`${ON}/kpi.json`, (o) => o.ticks, 532), src(CL, "Shield on (`demo/out/pas_ap_close_on`): 532 ticks, 0 P0 escape ticks")]);
fact("on_shield_ticks", "367", [src(`${ON}/kpi.json`, (o) => o.shield_ticks, 367)], { needsFlightData: true });
fact("on_ap_ticks", "165", [src(`${ON}/kpi.json`, (o) => o.autopilot_ticks, 165)], { needsFlightData: true });
fact("on_p0", "0", [src(`${ON}/kpi.json`, (o) => o.p0_escapes, 0), src(CL, "532 ticks, 0 P0 escape ticks")]);
fact("off_ticks", "1,100", [src(`${OFF}/kpi.json`, (o) => o.ticks, 1100), src(CL, "(`demo/out/pas_ap_close_off`): 1100 ticks")]);
// Where the aircraft WAS: ticks inside a P0 margin (here always building
// clearance). With the Shield on, all 78 fall under ArduPilot's BRAKE and RTL.
fact("on_unsafe", "78", [src(`${ON}/kpi.json`, (o) => o.unsafe_p0_ticks, 78)], { needsFlightData: true });
fact("on_unsafe_ap", "78", [src(`${ON}/flight_log.jsonl`, (rows) => rows.filter((r) => r.unsafe && r.flown === false).length, 78)], { needsFlightData: true });
fact("off_unsafe", "720", [src(`${OFF}/kpi.json`, (o) => o.unsafe_p0_ticks, 720)], { needsFlightData: true });
// P0 ticks only: bld-clearance and standoff-any are P0 in follow_car_citylife;
// kin-caps is P1. The first count of this flight (CHANGELOG, 13 Oct (later))
// took in the P1 ticks too; it was retracted on 14 Oct and is not printed.
fact("off_p0", "727", [src(`${OFF}/kpi.json`, (o) => o.p0_escapes, 727)], { needsFlightData: true });
// The Shield-off ticks by the rules their emitted command broke, each tick once.
const ev = (r) => new Set((r.emitted_violations || []).map((v) => (typeof v === "string" ? v : v.rule_id || v.id)));
const offCount = (want) => (rows) => rows.filter((r) => {
  const s = ev(r), b = s.has("bld-clearance"), so = s.has("standoff-any"), k = s.has("kin-caps");
  return want === "bld" ? b && !so : want === "so" ? so && !b : want === "both" ? b && so : k && !b && !so;
}).length;
fact("off_bld_only", "705", [src(`${OFF}/flight_log.jsonl`, offCount("bld"), 705)], { needsFlightData: true });
fact("off_so_only", "17", [src(`${OFF}/flight_log.jsonl`, offCount("so"), 17)], { needsFlightData: true });
fact("off_both", "5", [src(`${OFF}/flight_log.jsonl`, offCount("both"), 5)], { needsFlightData: true });
fact("off_kin_only", "22", [src(`${OFF}/flight_log.jsonl`, offCount("kin"), 22), src(CL, "kinematic caps 22")]);
// The recorded pair, 14 Oct: the same close approach flown again with
// --save-view; the on/off slide's stills come from these runs, so its numbers do too.
// With the Shield on the FSM went Brake, then X7 to RTL, with no hold; 20 of
// the ticks inside the building margin came while the Shield still flew (13 Oct:
// none), so the 13 Oct "all under BRAKE and RTL" is not said of this run.
const VON = "demo/out/pas_ap_video_on_20261014", VOFF = "demo/out/pas_ap_video_off_20261014";
fact("von_ticks", "517", [src(`${VON}/kpi.json`, (o) => o.ticks, 517), src(CL, "Shield on (`demo/out/pas_ap_video_on_20261014`): 517 ticks")]);
fact("von_p0", "0", [src(`${VON}/kpi.json`, (o) => o.p0_escapes, 0), src(CL, "0 P0 escape ticks in 388 Shield-flown ticks")]);
fact("von_shield_ticks", "388", [src(`${VON}/kpi.json`, (o) => o.shield_ticks, 388), src(CL, "ticks in 388 Shield-flown ticks")]);
fact("von_ap_ticks", "129", [src(`${VON}/kpi.json`, (o) => o.autopilot_ticks, 129)], { needsFlightData: true });
fact("von_unsafe", "71", [src(`${VON}/kpi.json`, (o) => o.unsafe_p0_ticks, 71)], { needsFlightData: true });
fact("von_unsafe_shield", "20", [src(`${VON}/flight_log.jsonl`, (rows) => rows.filter((r) => r.unsafe && r.flown === true).length, 20)], { needsFlightData: true });
fact("von_unsafe_ap", "51", [src(`${VON}/flight_log.jsonl`, (rows) => rows.filter((r) => r.unsafe && r.flown === false).length, 51)], { needsFlightData: true });
fact("voff_p0", "736", [src(`${VOFF}/kpi.json`, (o) => o.p0_escapes, 736), src(CL, "1099 ticks, 736 P0 escape ticks (rate 0.6697)")]);
fact("voff_ticks", "1,099", [src(`${VOFF}/kpi.json`, (o) => o.ticks, 1099), src(CL, "(`demo/out/pas_ap_video_off_20261014`): 1099 ticks")]);
fact("voff_rate", "0.6697", [src(`${VOFF}/kpi.json`, (o) => o.p0_violation_escape_rate, 0.6697), src(CL, "(rate 0.6697)")]);
fact("voff_unsafe", "739", [src(`${VOFF}/kpi.json`, (o) => o.unsafe_p0_ticks, 739)], { needsFlightData: true });
fact("voff_bld_only", "731", [src(`${VOFF}/flight_log.jsonl`, offCount("bld"), 731)], { needsFlightData: true });
fact("voff_so_only", "2", [src(`${VOFF}/flight_log.jsonl`, offCount("so"), 2)], { needsFlightData: true });
fact("voff_both", "3", [src(`${VOFF}/flight_log.jsonl`, offCount("both"), 3)], { needsFlightData: true });
// How the Shield-on aircraft entered the margin while the Shield flew (14 Oct).
// At the first correction (fsm_edge G1: a stand-off hold against the
// detector's target) the aircraft flew at 3.2 m/s (over the 0.5 s before);
// on the Shield-flown ticks that followed it carried 3.3 m on and stood up to
// 1.0 m inside the building margin. The Shield's state has no velocity
// (guardrail/models.py State) and its forecast advances the commanded action
// (guardrail/shield.py _predict), so each push-away read as recovering.
const g1At = (rows) => rows.findIndex((r) => r.fsm_edge === "G1");
fact("von_correct_t", "40.7 s", [src(`${VON}/flight_log.jsonl`, (rows) => rows[g1At(rows)].t.toFixed(1), "40.7")], { needsFlightData: true });
fact("von_speed", "3.2 m/s", [src(`${VON}/flight_log.jsonl`, (rows) => {
  const i = g1At(rows), a = rows[i - 5], b = rows[i];
  return (Math.hypot(b.x - a.x, b.y - a.y) / (b.t - a.t)).toFixed(1);
}, "3.2")], { needsFlightData: true });
fact("von_carry", "3.3 m", [src(`${VON}/flight_log.jsonl`, (rows) => {
  const i = g1At(rows);
  let m = 0;
  for (let j = i; j < rows.length && rows[j].flown; j++) m = Math.max(m, Math.hypot(rows[j].x - rows[i].x, rows[j].y - rows[i].y));
  return m.toFixed(1);
}, "3.3")], { needsFlightData: true });
fact("von_depth", "1.0 m", [src(`${VON}/flight_log.jsonl`, (rows) => Math.max(...rows.filter((r) => r.flown).map((r) => r.unsafe_depth_m || 0)).toFixed(1), "1.0")], { needsFlightData: true });
fact("von_forecast", "2.70 m to 4.64 m", [src(`${VON}/flight_log.jsonl`, "forecast 2.70->4.64m")], { needsFlightData: true });
fact("shield_cv", "constant-velocity", [src("guardrail/shield.py", "Constant-velocity forecast")]);
// The subject on the perception rail (CHANGELOG, 14 Oct): the truth was right,
// the detector was not on the car, and the rail has no start gate.
fact("truth_track", "0.52-0.54 m", [src(CL, "follow_vlm's validated Car_10 track (median 0.52-0.54 m)")]);
fact("est_near_car", "0", [src(CL, "across the close flights 0 estimates came within 6 m of the car")]);
// The car inside the camera's 90 deg field on the estimate ticks, per flight
// (metrics.json subject_truth.view). The CHANGELOG's "10 of the 149" is the
// 13 Oct Shield-on flight alone (tests/test_railvideo_truth_frame.py), so the
// 14 Oct pair's slide takes its own counts.
const vw = (o) => o.subject_truth.view;
fact("von_view_est", "156", [src(`${VON}/metrics.json`, (o) => vw(o).estimate_ticks, 156)], { needsFlightData: true });
fact("von_view_in", "4", [src(`${VON}/metrics.json`, (o) => vw(o).estimate_ticks_truth_in_view, 4)], { needsFlightData: true });
fact("voff_view_est", "132", [src(`${VOFF}/metrics.json`, (o) => vw(o).estimate_ticks, 132)], { needsFlightData: true });
fact("voff_view_in", "0", [src(`${VOFF}/metrics.json`, (o) => vw(o).estimate_ticks_truth_in_view, 0)], { needsFlightData: true });
// Where the car was when each close flight began: the true horizontal range
// (subject_range_m) on the first tick of the four flight logs, 150.8-151.9 m.
// (The CHANGELOG's "119-152 m" takes in the Shield-off flights' nearest
// approach, 117-119 m, which is no start distance.)
const startRange = (dir) => {
  const t = readSource(`${dir}/flight_log.jsonl`);
  return t == null ? null : JSON.parse(t.slice(0, t.indexOf("\n"))).subject_range_m;
};
fact("car_start", "151-152 m", [src(`${VON}/flight_log.jsonl`, () => {
  const r = [ON, OFF, VON, VOFF].map(startRange);
  if (r.some((v) => v == null)) return "<a close flight's log is absent>";
  return `${Math.round(Math.min(...r))}-${Math.round(Math.max(...r))}`;
}, "151-152")], { needsFlightData: true });
fact("on_interventions", "173", [src(`${ON}/metrics.json`, (o) => o.interventions, 173), src(CL, "173 interventions")]);
fact("loiter_repairs", "2.08 m and 2.14 m", [src(CL, "a stand-off repair of 2.08 m and a building-clearance repair of 2.14 m, over theta 2.0 m")]);
fact("rtl_home", "58.1 s", [src(CL, "RTL reached home at 58.1 s")]);
fact("brake_hold", "7.99-8.01 m", [src(CL, "held 7.99-8.01 m over 42 ticks")]);
fact("brake_ticks", "42", [src(CL, "held 7.99-8.01 m over 42 ticks")]);
fact("signed_flight", "signature verified", [src(CL, "(signature verified; the first signed flight on this rail)")]);
fact("rt_factor", "0.966 / 0.946", [src(CL, "ArduPilot real-time factor 0.966 (on) and 0.946 (off")]);
fact("on_tts", "8.51 s", [src(`${ON}/kpi.json`, (o) => Math.round(o.mean_time_to_safe_s * 100) / 100, 8.51)], { needsFlightData: true });

// Gate G1, re-passed 13 Oct.
fact("g1_runs", "3/3", [src(CL, "3/3 runs, all four conditions")]);
fact("g1_ekf", "0.82-0.83 m", [src(PO, (o) => o.g1.ekf_p95_m.map((v) => v.toFixed(2)).join("-"), "0.82-0.83"), src(CL, "EKF p95 0.82-0.83 m against the 1.0 m limit")]);
fact("g1_limit", "1.0 m", [src(CL, "EKF p95 0.82-0.83 m against the 1.0 m limit")]);
fact("g1_null", "18.96-18.98 m", [src(CL, "(frozen-position null 18.96-18.98 m)")]);
fact("g1_climb", "14.47 m", [src(CL, "climb 14.47 m from rest")]);
fact("g1_land", "0.06-0.09 m", [src(CL, "landing 0.06-0.09 m from take-off")]);
fact("g1_fence", "0", [src(CL, "0 fence breaches, 0 airborne collisions")]);
fact("g1_params", "FENCE_TYPE 7, AVOID_ENABLE 0", [src(CL, "now FENCE_TYPE 7, AVOID_ENABLE 0")]);
fact("mp_hz", "19.0 Hz", [src(CL, "Windows port 14550 delivered GLOBAL_POSITION_INT and ATTITUDE at 19.0 Hz for the whole 120 s mission")]);

// Harness and Shield speed, 13 Oct.
fact("forecast_old", "3 s at 0.5 s (6 poses)", [src(CL, "used a 3 s forecast at 0.5 s (6 poses)")]);
fact("forecast", "5 s at 0.1 s (50 poses)", [src(CL, "the grant locks 5 s at 0.1 s (50 poses)"), src("guardrail/shield.py", "LOOKAHEAD_S = 5.0")]);
fact("wind", "6.4 m/s", [src(CL, "is rejected up to 6.4 m/s, derived from the SITL default frame, not measured")]);
fact("sweep_eps", "176", [src(SW, (o) => o._scope.episodes, 176)]);
fact("sweep_pass", "132", [src(SW, (o) => o._counts.pass, 132)]);
fact("sweep_tracked", "22", [src(SW, (o) => o._counts.tracked, 22)]);
fact("sweep_known", "22", [src(SW, (o) => o._counts.known_failure, 22)]);
fact("sweep_fail", "0", [src(SW, (o) => o._counts.fail, 0)]);
fact("moving_before", "2.53-2.64 ms", [src(CL, "moving 2.53-2.64 -> 0.45 ms")]);
fact("moving_after", "0.45 ms", [src(CL, "moving 2.53-2.64 -> 0.45 ms")]);
fact("static_ms", "0.35 to 0.30 ms", [src(CL, "static 0.35 -> 0.30 ms")]);
fact("zones_built", "199 of 9,600", [src(CL, "in 200 checks the Shield builds 199 zones instead of 9600")]);
fact("null_loop", "222 ms", [src(CL, "the per-fence loop (the null) takes 222 ms")]);

// The grant's loop rate and action shape, as the Shield states them.
fact("rate", "10 Hz", [src("guardrail/shield.py", "a 10 Hz monitor")]);
fact("action4d", "4-D", [src("guardrail/shield.py", "validate a 4-D action")]);

// Interfaces.
fact("rpcs", "10", [src(CL, "The grant's 10 gRPC RPCs"), src("guardrail/protos/harness.proto", "rpc StreamEvents")]);
fact("watchdog", "0.5 s", [src(CL, "A stale-action watchdog (0.5 s, five missed 10 Hz ticks)")]);
fact("hotapply_ms", "1.17 ms", [src("docs/DESIGN-shield-enforcement.md", "median 1.17 ms, p99 1.34 ms, max 1.50 ms over 300 spawns, against the grant's 50 ms")]);
fact("hotapply_p99", "1.34 ms", [src("docs/DESIGN-shield-enforcement.md", "p99 1.34 ms")]);
fact("suite", "2,526 of 2,536", [src(CL, "Full suite on Python 3.10: 2526 of 2536 over 92 test files.")]);

// Harness KPIs (headless library, grant's forecast).
fact("lib_eps", "175", [src(SW, (o) => all(o).episodes, 175)]);
fact("lib_p0", "0.000133", [src(SW, (o) => all(o).p0_escape_rate, 0.000133)]);
fact("lib_p0_ticks", "6", [src(SW, (o) => all(o).p0_escapes, 6)]);
fact("lib_p0_eps", "3", [src(SW, (o) => all(o).episodes_with_p0_escape, 3)]);
fact("lib_twin_cells", "3", [src(SW, (o) => o._scope.cells_per_family["time-window-opens-on-vehicle"], 3)]);
fact("lib_controls", "1", [src(SW, (o) => o._rollup._controls_shield_off.episodes, 1)]);
fact("lib_p0_null", "0.236", [src(SW, (o) => Math.round(all(o).null_passthrough_p0_escape_rate * 1000) / 1000, 0.236)]);
fact("failsafe", "0.838", [src(SW, (o) => Math.round(all(o).failsafe_labelled_episodes.failsafe_trigger_correctness * 1000) / 1000, 0.838)]);
fact("failsafe_frac", "140/167", [src(SW, (o) => { const f = all(o).failsafe_labelled_episodes; return `${f.scored - f.false_triggers - f.missed_triggers}/${f.scored}`; }, "140/167")]);
fact("failsafe_false", "26", [src(SW, (o) => all(o).failsafe_labelled_episodes.false_triggers, 26)]);
fact("failsafe_missed", "1", [src(SW, (o) => all(o).failsafe_labelled_episodes.missed_triggers, 1)]);
fact("failsafe_null", "0.940", [src(SW, (o) => all(o).failsafe_labelled_episodes.null_never_trigger.toFixed(3), "0.940")]);
fact("failsafe_g3", "22", [src(SW, (o) => Object.entries(o._failsafe_labels.triggers_by_edge.false).filter(([k]) => k.includes("G3")).reduce((a, [, n]) => a + n, 0), 22)]);
fact("mission", "0.577", [src(SW, (o) => all(o).mission_success_rate.toFixed(3), "0.577")]);
fact("repair_mag", "7.04 m/s", [src(SW, (o) => all(o).mean_repair_magnitude_mps.toFixed(2), "7.04")]);
fact("tts", "5.85 s", [src(SW, (o) => all(o).mean_time_to_safe_s.toFixed(2), "5.85")]);

// Components (progress_oct.json is regenerated from the code by build_progress_oct_data.py).
fact("wp1_rt", "29/29", [src(PO, (o) => `${o.wp1.round_trip_passed}/${o.wp1.policies}`, "29/29")]);
fact("wp1_tamper", "29/29", [src(PO, (o) => `${o.wp1.tamper_refused}/${o.wp1.tamper_run}`, "29/29")]);
fact("wp1_neg", "35/35", [src(PO, (o) => `${o.wp1.negative_refused}/${o.wp1.negative_run}`, "35/35")]);
fact("wp2_p0", "105/105", [src(PO, (o) => `${o.wp2.p0_covered}/${o.wp2.p0_in_scope}`, "105/105")]);
fact("wp2_tokens", "54-106", [src(PO, (o) => o.wp2.tokens_exact_range.join("-"), "54-106")]);
fact("wp2_budget", "256", [src("guardrail/compiler.py", "DEFAULT_BUDGET_TOKENS = 256")]);
fact("wp2_naive", "104/105", [src("docs/data/csp_kpis.json", (o) => `${Math.round(o.discriminating.p0_coverage_baseline * 105)}/105 at ${o.discriminating.budget_tokens}`, "104/105 at 120")]);
fact("check_ms", "0.31 ms", [src(PO, (o) => o.wp3_bench.grant_load["near/_check"].current.median_ms.toFixed(2), "0.31")]);
fact("replay_n", "1,301", [src(PO, (o) => o.wp3_profile.performance_cores.check.n, 1301)]);
fact("replay_check_p99", "3.38 ms", [src(PO, (o) => o.wp3_profile.performance_cores.check.p99_ms, 3.38)]);
fact("replay_tick_p99", "11.9 ms", [src(PO, (o) => o.wp3_profile.performance_cores.filter.p99_ms.toFixed(1), "11.9")]);
fact("paraphrases", "112", [src(PO, (o) => o.paraphraser.paraphrases, 112)]);
fact("fsm_n", "3", [src("guardrail/fsm.py", "n_violations: int = 3")]);
fact("fsm_window", "5 s", [src("guardrail/fsm.py", "window_s: float = 5.0")]);
fact("fsm_theta", "2.0 m / 0.5 m", [src("guardrail/fsm.py", "theta_lateral_m: float = 2.0"), src(PO, (o) => `${o.wp3_fsm.theta_lateral_m}/${o.wp3_fsm.theta_vertical_m}`, "2/0.5")]);
fact("fsm_recover", "2 s", [src("guardrail/fsm.py", "t_recover_s: float = 2.0")]);
fact("fsm_loiter_timeout", "30 s", [src("guardrail/fsm.py", "loiter_timeout_s: float | None = 30.0")]);
fact("eff_check_p99", "7.6 ms", [src(PO, (o) => o.wp3_profile.efficiency_cores.check.p99_ms.toFixed(1), "7.6")]);
fact("profile_date", "6 Oct", [src(PO, (o) => o.wp3_profile.performance_cores.created.slice(0, 10), "2026-10-06")]);
fact("sitl_brake", "14.97-15.01 m", [src("docs/DESIGN-escalation-fsm.md", "| BRAKE | 19.9 s | 14.97-15.01 m")]);
fact("sitl_loiter", "6.7 s", [src("docs/DESIGN-escalation-fsm.md", "| LOITER | 6.7 s | 15.0 m to the ground")]);

// The VLA.
// A standalone load probe on the desktop: two calls, no simulator; the first
// call includes warm-up. In flight beside Project AirSim on the same GPU the
// OpenVLA-7B-based backends ran far slower (the table's in-flight records).
const VBT = "deploy/evidence/vla_backend_table.md";
fact("vla_s", "0.86 s", [src(VBT, "warm 0.86 s = 1.16 Hz")]);
fact("vla_hz", "1.16 Hz", [src(VBT, "warm 0.86 s = 1.16 Hz")]);
fact("vla_first", "1.96 s", [src(VBT, "4-bit NF4: per-inference 1.96 s, 0.86 s")]);
fact("vla_probe", "standalone load probe on the lab desktop, no simulator", [src(VBT, "conditions: standalone load probe on the lab desktop, no simulator; the first call includes warm-up")]);
fact("vla_flight_a", "0.12-0.15 Hz", [src(VBT, "in flight: 0.12-0.15 Hz over 6 flights (102 inferences)")]);
fact("vla_flight_b", "0.11-0.13 Hz", [src(VBT, "in flight: 0.11-0.13 Hz over 2 flights (29 inferences)")]);
fact("vla_flight_cond", "Project AirSim rendering on the same GPU", [src(VBT, "conditions: in flight, Project AirSim rendering on the same GPU")]);
fact("vla_img", "224 x 224", [src("demo/real_vla_demo.py", "IMAGE_SIZE = (224, 224)")]);
fact("vla_img_tokens", "256", [src("demo/real_vla_demo.py", "OPENVLA_IMAGE_TOKENS = 256")]);
fact("vla_act_tokens", "7", [src("demo/real_vla_demo.py", "OPENVLA_ACTION_TOKENS = 7")]);
fact("vla_text_room", "1,784", [src("demo/real_vla_demo.py", "What remains, 1784")]);
fact("vla_lateral", "unverified", [src("demo/real_vla_demo.py", "LATERAL_SIGN = \"unverified: Bridge +y assumed to be the drone's right\"")]);
// OpenVLA's bin count per action token is not stated in a tracked file (only
// in the checkpoint's config.json, outside the repository), so the slide says
// "a discretised value per dimension" and prints no count.

// The grant's targets (Prof. Lai's pages; the tracked fallbacks restate them).
fact("t_failsafe", "≥ 0.99", [src(`${GRANT}/02-implementation/stress-testing.md`, "| Fail-safe trigger correctness | ≥ 99% |"), src(PO, (o) => o.wp3_fsm.failsafe_target, 0.99)]);
fact("t_p0", "0", [src(`${GRANT}/01-context/grant-overview.md`, "**P0 violation escape rate — target 0.**"), src("tools/deck/build_progress_oct_deck.js", "P0 violation escape rate (target 0)")]);
fact("t_check", "5 ms", [src(`${GRANT}/02-implementation/policy-dsl.md`, "| Monitor query latency at 10 Hz with 50 active rules | ≤ 5 ms"), src(PO, (o) => o.wp3_bench.grant_load.budgets_ms._check, 5)]);
fact("t_tick", "100 ms", [src(`${GRANT}/02-implementation/safety-shield.md`, "| Monitor tick budget | ≤ 100 ms (10 Hz) end-to-end |"), src(PO, (o) => o.wp3_bench.grant_load.budgets_ms.filter, 100)]);
fact("t_hotapply", "50 ms", [src(`${GRANT}/02-implementation/policy-dsl.md`, "| Hot-apply latency (dynamic_nfz spawn) | ≤ 50 ms |"), src("docs/DESIGN-shield-enforcement.md", "against the grant's 50 ms")]);
fact("t_p0cov", "100 %", [src(`${GRANT}/02-implementation/prefix-compiler.md`, "P0 coverage = 100% across the test fixtures"), src("docs/DESIGN-prefix-compiler.md", "| P0 coverage = 100 % |")]);
fact("t_final", "30 November 2026", [src(`${GRANT}/01-context/grant-overview.md`, "| **2026-11-30** | Final delivery |"), src("tools/deck/build_progress_oct_deck.js", "PLAN_TO = \"2026-11-30\"")]);

// The 13 Oct city flight (the video still's slide; dropped with the still).
const DM = `demo/out/${DEMO_TAG}`;
fact("demo_nfz_s", "0 s", [src(`${DM}/metrics.json`, (o) => o.nfz_s, 0)], { needsFlightData: true });
fact("demo_interventions", "1", [src(`${DM}/metrics.json`, (o) => o.interventions, 1)], { needsFlightData: true });
fact("demo_within30", "99.1 %", [src(`${DM}/metrics.json`, (o) => o.frac_within_30m, 0.991)], { needsFlightData: true });
fact("demo_ticks", "2,379", [src(`${DM}/metrics.json`, (o) => o.ticks, 2379)], { needsFlightData: true });
fact("demo_alt", "8 m", [src(`${DM}/metrics.json`, (o) => o.params.cruise_alt, 8)], { needsFlightData: true });
fact("demo_30m", "30 m", [src(`${DM}/metrics.json`, "\"frac_within_30m\": 0.991")], { needsFlightData: true });
fact("demo_sep", "20.6 m", [src(`${DM}/metrics.json`, (o) => o.sep_mean_m, 20.6)], { needsFlightData: true });
fact("demo_p0", "0", [src(`${DM}/kpi.json`, (o) => o.p0_escapes, 0)], { needsFlightData: true });
fact("demo_clamp", "5.02 to 5.0 m/s", [src(`${DM}/flight_log.jsonl`, "h-speed 5.02 -> 5.0")], { needsFlightData: true });
fact("demo_zone_d", "4 m", [src(`${DM}/flight_log.jsonl`, "\"fence_d\": 4.0, \"fence_scale\"")], { needsFlightData: true });
fact("demo_avoid", "689", [src(`${DM}/metrics.json`, (o) => o.policy_hud.banner_ticks.avoid, 689)], { needsFlightData: true });

/* The ten numbers of the cheatsheets, in order: the two facts, the English and
 * Indonesian labels, the value's format, and an Indonesian format where the
 * English one carries a word. */
const TOP10 = [
  ["on_p0", "off_p0", "Close approach 13 Oct, P0 escape ticks: on (of {on_shield_ticks} Shield-flown) vs off (of {off_ticks})",
    "Close approach 13 Okt, tick P0 escape: Shield on (dari {on_shield_ticks}) vs off (dari {off_ticks})", "{v1} vs {v2}"],
  ["von_p0", "voff_p0", "The same, recorded on video 14 Oct: on (of {von_shield_ticks}) vs off (of {voff_ticks})",
    "Sama, direkam video 14 Okt: on (dari {von_shield_ticks}) vs off (dari {voff_ticks})", "{v1} vs {v2}"],
  ["on_unsafe", "off_unsafe", "13 Oct, ticks inside the building margin; Shield on: all under BRAKE and RTL",
    "13 Okt, tick di dalam margin gedung; Shield on: semuanya saat BRAKE dan RTL", "{v1} vs {v2}"],
  ["brake_hold", null, "BRAKE hold's altitude in flight ({brake_ticks} ticks)",
    "Ketinggian hold BRAKE saat terbang ({brake_ticks} tick)", "{v1}"],
  ["g1_ekf", "g1_limit", "Gate G1 re-pass: ArduPilot EKF error p95",
    "Gate G1 lulus lagi: error EKF ArduPilot p95", "{v1} (limit {v2})"],
  ["moving_before", "moving_after", "Moving-zone rule check, 50 rules, desktop median",
    "Cek zona bergerak, 50 aturan, median desktop", "{v1} to {v2}", "{v1} menjadi {v2}"],
  ["failsafe", "t_failsafe", "Fail-safe trigger correctness, headless library ({failsafe_frac})",
    "Ketepatan pemicu fail-safe, pustaka headless ({failsafe_frac})", "{v1} (target {v2})"],
  ["lib_p0", "t_p0", "P0 escape rate, headless library ({lib_p0_ticks} ticks, {lib_p0_eps} episodes)",
    "Laju P0 escape, pustaka headless ({lib_p0_ticks} tick, {lib_p0_eps} episode)", "{v1} (target {v2})"],
  ["vla_s", "vla_hz", "OpenVLA-7B 4-bit, desktop, no simulator, warm",
    "OpenVLA-7B 4-bit, desktop tanpa simulator, warm", "{v1} = {v2}"],
  ["wp2_p0", null, "P0 rules the CSP carries over the 29 policies",
    "Aturan P0 yang dibawa CSP pada 29 policy", "{v1}"],
];

function readSource(file) {
  const p = path.isAbsolute(file) ? file : path.join(REPO, file);
  if (!fs.existsSync(p)) return null;
  return fs.readFileSync(p, "utf8");
}
function verifyFacts() {
  const cache = new Map();
  const problems = [];
  for (const f of Object.values(FACTS)) {
    for (const s of f.sources) {
      if (!cache.has(s.file)) cache.set(s.file, readSource(s.file));
      const text = cache.get(s.file);
      if (text == null) continue;
      f.file = s.file;
      if (s.phrase != null) {
        f.status = new RegExp(reEsc(s.phrase)).test(text) ? "verified" : "MISMATCH";
        if (f.status !== "verified") problems.push(`${f.id}: phrase not found in ${s.file}: "${s.phrase}"`);
      } else {
        let got;
        try {
          const doc = s.file.endsWith(".jsonl")
            ? text.split(/\r?\n/).filter((l) => l.trim()).map((l) => JSON.parse(l)) : JSON.parse(text);
          got = s.get(doc);
        } catch (e) { got = `<${e.message}>`; }
        const ok = typeof s.eq === "number" ? (typeof got === "number" && Math.abs(got - s.eq) < 1e-9) : got === s.eq;
        f.status = ok ? "verified" : "MISMATCH";
        if (!ok) problems.push(`${f.id}: ${s.file} gives ${JSON.stringify(got)}, the deck prints for ${JSON.stringify(s.eq)}`);
      }
      break;
    }
    if (!f.status) f.status = "unverified";
  }
  return problems;
}
const USED = new Set();
const V = (id) => { const f = FACTS[id]; if (!f) throw new Error(`unknown fact ${id}`); USED.add(id); return f.v; };
const fill = (t) => t.replace(/\{(\w+)\}/g, (_, id) => V(id));

/* ---- Wording guard ------------------------------------------------------------ */
// Hedges and filler the deck never uses (the user's style rule).
const HEDGES = ["we believe", "i believe", "we think", "seems", "seem to", "seemingly", "somewhat",
  "hopefully", "we hope", "perhaps", "maybe", "might", "arguably", "probably", "fairly", "quite",
  "rather", "kind of", "sort of", "a bit", "basically", "essentially", "it should be noted",
  "it is important to note", "in order to", "unfortunately", "sorry", "apologi", "regrettabl",
  "we tried", "try to", "attempt to", "very ", "really", "actually", "just "];
function hedgeHits(text) {
  const low = text.toLowerCase();
  return HEDGES.filter((h) => new RegExp(`(^|[^a-z])${h.replace(/ /g, "\\s+")}`).test(low));
}

/* ---- Formatting helpers --------------------------------------------------------- */
// Poppins Bold advance widths (1/1000 em) for ASCII 32-126, read from
// Poppins-Bold.ttf with fontTools; anything else counts as 0.6 em.
const POPPINS_BOLD = ("212,392,411,904,658,870,798,221,477,477,534,628,287,580,282,453,652,376,571,605,677,650,637,535," +
  "648,615,284,347,551,696,541,538,1080,737,659,762,727,541,547,762,731,295,578,697,477,918,752,786,624,788,652,615,591," +
  "705,730,1052,715,671,596,510,802,510,709,780,291,679,679,605,679,616,360,679,674,295,294,618,295,1059,674,637,679,679," +
  "428,558,406,674,626,866,588,632,496,499,291,499,636").split(",").map(Number);
const boldWidthIn = (t, pt) => [...t].reduce((a, ch) => {
  const c = ch.codePointAt(0);
  return a + (c >= 32 && c <= 126 ? POPPINS_BOLD[c - 32] : 600);
}, 0) / 1000 * pt / 72;

// Poppins has no U+2192; arrows are runs in Segoe UI (as the 30 Sept deck does).
// Nor U+2011: a range or date such as 7.99-8.01 or 16-23 gets a non-breaking
// hyphen in Segoe UI, so PowerPoint never breaks a line inside it.
const ARROW_FONT = "Segoe UI";
const UNIT_GAP = /(\d) (Hz|ms|s|m|m\/s|MB|px|tokens|ticks|%|Oct|Nov)(?![\w])/g;
const nbsp = (x) => x.replace(UNIT_GAP, "$1\u00a0$2").replace(/(\d)-(?=\d)/g, "$1\u2011");
const SEGOE = /(\u2192|\u2011)/;
function withArrows(t0) {
  const t = typeof t0 === "string" ? nbsp(t0) : t0.map((r) => ({ ...r, text: nbsp(r.text) }));
  const split = (txt, o) => txt.split(SEGOE).filter((x) => x !== "").map((x, n) => {
    const opts = { ...(o || {}), ...(SEGOE.test(x) ? { fontFace: ARROW_FONT } : {}) };
    if (n > 0) delete opts.bullet;          // a bullet starts a new paragraph in pptxgenjs
    return { text: x, options: opts };
  });
  if (typeof t === "string") return SEGOE.test(t) ? split(t, {}) : t;
  const out = [];
  for (const r of t) {
    if (SEGOE.test(r.text)) {
      const parts = split(r.text, r.options);
      if (r.options && r.options.breakLine) {
        parts.forEach((p, i) => { if (i < parts.length - 1) delete p.options.breakLine; });
      }
      out.push(...parts);
    } else out.push(r);
  }
  return out;
}

async function photo(file) {
  const buf = fs.readFileSync(file);
  const m = await sharp(buf).metadata();
  const kind = m.format === "png" ? "png" : "jpeg";
  return { data: `image/${kind};base64,${buf.toString("base64")}`, aspect: m.width / m.height };
}

/* The still: extracted from the video into the pack once (pack build only). */
function ensureStill() {
  const chosen = opt("--still");
  if (chosen) return fs.existsSync(chosen) ? path.resolve(chosen) : null;
  if (fs.existsSync(STILL_FPV)) return STILL_FPV;
  if (!PACK_BUILD || !fs.existsSync(DEMO_VIDEO)) return null;
  fs.mkdirSync(FIG_DIR, { recursive: true });
  try {
    execFileSync("ffmpeg", ["-v", "error", "-y", "-ss", STILL_VIDEO_T, "-i", DEMO_VIDEO, "-frames:v", "1", "-q:v", "2", STILL_FULL], { stdio: "inherit" });
    // The video is two 1280 x 720 panels: the drone's camera with the HUD on the left.
    execFileSync("ffmpeg", ["-v", "error", "-y", "-i", STILL_FULL, "-vf", "crop=iw/2:ih:0:0", "-q:v", "2", STILL_FPV], { stdio: "inherit" });
  } catch (e) {
    console.warn(`still: ffmpeg failed (${e.message}); the still slide is dropped`);
    return null;
  }
  return fs.existsSync(STILL_FPV) ? STILL_FPV : null;
}
function findOnOff(kind) {
  const chosen = opt(`--onoff-${kind}`);
  if (chosen) return fs.existsSync(chosen) ? path.resolve(chosen) : null;
  for (const ext of ["png", "jpg", "jpeg"]) {
    const p = path.join(FIG_DIR, `ardupilot_shield_${kind}.${ext}`);
    if (fs.existsSync(p)) return p;
  }
  return null;
}

/* ---- Build ------------------------------------------------------------------------ */
async function main() {
  const problems = verifyFacts();
  if (flag("--check-facts-only")) {
    problems.forEach((p) => console.error("  " + p));
    const unv = Object.values(FACTS).filter((f) => f.status === "unverified").map((f) => f.id);
    console.log(`facts: ${Object.keys(FACTS).length}, mismatched ${problems.length}, unverified ${unv.length}${unv.length ? ` (${unv.join(", ")})` : ""}`);
    process.exit(problems.length ? 1 : 0);
  }
  if (problems.length) {
    console.error("REFUSED: numbers on the slides no longer match their sources:");
    problems.forEach((p) => console.error("  " + p));
    process.exit(1);
  }
  const unverified = Object.values(FACTS).filter((f) => f.status === "unverified");
  if (unverified.length) {
    console.warn(`facts unverified here (source files absent): ${unverified.map((f) => f.id).join(", ")}`);
    if (!ALLOW_UNVERIFIED) {
      console.error("REFUSED: the pack build needs every fact checked against its file (--allow-unverified for a draft).");
      process.exit(1);
    }
  }

  const stillPath = ensureStill();
  const onPath = findOnOff("on"), offPath = findOnOff("off");
  const optional = { still: !!stillPath, onoff: !!(onPath && offPath) };
  for (const n of FIGS) if (!fs.existsSync(figPng(n))) throw new Error(`missing figure ${figPng(n)} (run build_explainer_figures.py --png)`);

  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "The Guardrail, End to End";
  pres.author = LAB;
  pres.company = LAB;
  pres.subject = "VLA-UAV Guardrail, ITRI ICL subcontract: how the system works and the progress of 9-14 October 2026";
  const H = helpers(pres);
  const { TEAL, TEAL_DK, TEAL_DEEP, TEAL_TINT, TEAL_TINT2, WHITE, INK, GREY, LGREY, LINE, FF, CHARCOAL, BLACK } = T;

  const ic = {
    shield: await iconPng(FaShieldAlt), sign: await iconPng(FaFileSignature), random: await iconPng(FaRandom),
    clip: await iconPng(FaClipboardList), ok: await iconPng(FaCheckCircle), plane: await iconPng(FaPlane),
    cogs: await iconPng(FaCogs), speed: await iconPng(FaTachometerAlt), net: await iconPng(FaNetworkWired),
    clock: await iconPng(FaClock), history: await iconPng(FaHistory), sync: await iconPng(FaSyncAlt),
    video: await iconPng(FaVideo), camera: await iconPng(FaCamera), brain: await iconPng(FaBrain),
    arrows: await iconPng(FaArrowsAlt), pause: await iconPng(FaPauseCircle), layers: await iconPng(FaLayerGroup),
  };

  /* Text record: every addText goes through the wording guard and into the
   * layout record that tests/test_explainer_deck.py measures for overflow. */
  const SLIDES = [];
  const LAYOUT = [];
  const WHERE = {};          // figure or still -> the slide it is on (for the pack README)
  let page = 0;
  function track(s, title, kind) {
    const rec = { n: page, title, kind, text: [], notes: "" };
    SLIDES.push(rec);
    const addText = s.addText.bind(s);
    s.addText = (t, o = {}) => {
      const runs = withArrows(t);
      const flat = Array.isArray(runs) ? runs.map((r) => r.text).join("") : String(runs);
      rec.text.push(Array.isArray(t) ? t.map((r) => r.text + (r.options && r.options.breakLine ? "\n" : "")).join("") : String(t));
      // Paragraphs of runs: a run with breakLine ends its paragraph; "\n" splits too.
      const paras = [[]];
      const list = Array.isArray(runs) ? runs : [{ text: String(runs), options: {} }];
      for (const r of list) {
        const ro = r.options || {};
        const pieces = String(r.text).split("\n");
        pieces.forEach((piece, i) => {
          if (i > 0) paras.push([]);
          paras[paras.length - 1].push({ t: piece, size: ro.fontSize || o.fontSize || 18, bold: !!(ro.bold != null ? ro.bold : o.bold), face: ro.fontFace || o.fontFace || FF });
        });
        if (ro.breakLine) paras.push([]);
      }
      if (!o.nofit) {
        LAYOUT.push({
          slide: page, x: o.x, y: o.y, w: o.w, h: o.h, lsm: o.lineSpacingMultiple || 1.0,
          margin: o.margin == null ? null : o.margin, valign: o.valign || "top", text: flat.slice(0, 80),
          bullet: list.some((r) => r.options && r.options.bullet), paraAfter: o.paraSpaceAfter || 0, cs: o.charSpacing || 0,
          paras: paras.filter((p, i) => p.length || i < paras.length - 1),
        });
      }
      const o2 = { ...o };
      delete o2.nofit;
      return addText(runs, o2);
    };
    const addNotes = s.addNotes.bind(s);
    s.addNotes = (n) => { rec.notes += n; return addNotes(n); };
    return s;
  }

  /* A finding headline on one line: the size steps down from `max` until the
   * line fits (Poppins Bold advance widths); longer headlines are refused. */
  const titleSize = (t, w = 8.9, max = 20, min = 18) => {
    for (let pt = max; pt >= min; pt -= 0.5) if (boldWidthIn(t, pt) <= w - 0.24) return pt;
    throw new Error(`headline too long for one line: "${t}" (${boldWidthIn(t, min).toFixed(2)} in at ${min} pt)`);
  };
  function content(eyebrow, title) {
    page++;
    const s = track(pres.addSlide(), title, "content");
    s.background = { color: WHITE };
    s.addText(eyebrow.toUpperCase(), { x: 0.55, y: 0.4, w: 8.9, h: 0.28, fontSize: 10, color: TEAL, fontFace: FF, bold: true, charSpacing: 2 });
    s.addText(title, { x: 0.55, y: 0.68, w: 8.9, h: 0.56, fontSize: titleSize(title), color: BLACK, fontFace: FF, bold: true, valign: "middle" });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 1.3, w: 0.9, h: 0, line: { color: TEAL, width: 2.5 } });
    return s;
  }
  /* A figure slide: a compact header (eyebrow, one-line headline, the teal
   * accent beside the eyebrow) so the figure can take the slide's full width;
   * the figure is fitted into the box below. tools/deck/build_explainer_figures.py
   * sizes its text for this box (SLIDE_BOX_IN), and tests/test_explainer_figures.py
   * reads bw and bh from this line. */
  async function figureSlide(eyebrow, title, fig, sourceTxt, notes) {
    page++;
    WHERE[fig] = page;
    const s = track(pres.addSlide(), title, "figure");
    s.background = { color: WHITE };
    s.addShape(pres.shapes.LINE, { x: 0.32, y: 0.19, w: 0.3, h: 0, line: { color: TEAL, width: 2.5 } });
    s.addText(eyebrow.toUpperCase(), { x: 0.68, y: 0.08, w: 8.9, h: 0.22, fontSize: 8.5, color: TEAL, fontFace: FF, bold: true, charSpacing: 2, valign: "middle", margin: 0 });
    s.addText(title, { x: 0.32, y: 0.3, w: 9.36, h: 0.38, fontSize: titleSize(title, 9.36, 17, 15), color: BLACK, fontFace: FF, bold: true, valign: "middle", margin: 0 });
    const im = await img(figPng(fig), { maxW: 3200 });
    const bx = 0.32, by = 0.7, bw = 9.36, bh = 4.44;
    let w = bw, h = bw / im.aspect;
    if (h > bh) { h = bh; w = bh * im.aspect; }
    const x = bx + (bw - w) / 2;
    s.addImage({ data: im.data, x, y: by, w, h });
    s.addText(sourceTxt, { x: 0.32, y: 5.2, w: 8.88, h: 0.3, fontSize: 7.5, color: LGREY, fontFace: FF, italic: true, valign: "middle" });
    H.badge(s, page);
    s.addNotes(notes);
    return s;
  }
  function source(s, txt) {
    s.addText(txt, { x: 0.55, y: 4.95, w: 8.65, h: 0.38, fontSize: 8.5, color: GREY, fontFace: FF, italic: true, valign: "middle" });
  }
  const cardBox = (s, x, y, w, h, fill = WHITE, line = LINE) => s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x, y, w, h, fill: { color: fill }, line: { color: line, width: 1 }, rectRadius: 0.08,
    shadow: { type: "outer", blur: 6, offset: 2, angle: 90, color: "D9DEE3", opacity: 0.4 },
  });
  function arrowR(s, x1, y, x2, color = CHARCOAL) {
    s.addShape(pres.shapes.LINE, { x: x1, y, w: x2 - x1, h: 0, line: { color, width: 1.25, endArrowType: "triangle" } });
  }
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
  function node(s, x, y, w, h, title, sub, { fill = WHITE, line = LINE, tag = null, titleSize: ts = 11, subSize = 9 } = {}) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill }, line: { color: line, width: 1.25 }, rectRadius: 0.07 });
    s.addText(title, { x: x + 0.04, y: y + 0.1, w: w - 0.08, h: 0.32, fontSize: ts, color: INK, fontFace: FF, bold: true, align: "center", valign: "middle" });
    if (sub) s.addText(sub, { x: x + 0.04, y: y + 0.42, w: w - 0.08, h: h - 0.48, fontSize: subSize, color: GREY, fontFace: FF, align: "center", valign: "top" });
    if (tag) {
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: x + 0.1, y: y - 0.14, w: 0.52, h: 0.26, fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.05 });
      s.addText(tag, { x: x + 0.1, y: y - 0.14, w: 0.52, h: 0.26, fontSize: 9, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle", margin: 0 });
    }
  }
  function tagCard(s, x, y, w, h, icon, tag, body, bodySize = 9.5, fill = WHITE, line = LINE) {
    cardBox(s, x, y, w, h, fill, line);
    s.addImage({ data: icon, x: x + 0.2, y: y + 0.18, w: 0.26, h: 0.26 });
    s.addText(tag.toUpperCase(), { x: x + 0.54, y: y + 0.15, w: w - 0.7, h: 0.32, fontSize: 9, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
    s.addText(body, { x: x + 0.14, y: y + 0.52, w: w - 0.28, h: h - 0.6, fontSize: bodySize, color: INK, fontFace: FF, lineSpacingMultiple: 1.0, valign: "top" });
  }
  function bigStat(s, x, y, w, value, label, { size = 26, color = TEAL, labelSize = 10, labelH = 0.5 } = {}) {
    s.addText(value, { x, y, w, h: 0.55, fontSize: size, color, fontFace: FF, bold: true, valign: "bottom" });
    s.addText(label, { x, y: y + 0.57, w, h: labelH, fontSize: labelSize, color: GREY, fontFace: FF, valign: "top" });
  }
  function tableV(s, rows, { x = 0.55, y, w = 8.9, colFrac, rhs, fontSize = 9, emph = [], bold0 = true }) {
    const colW = colFrac.map((f) => w * f);
    let ry = y;
    rows.forEach((r, ri) => {
      const header = ri === 0, zebra = !header && ri % 2 === 0;
      let cx = x;
      r.forEach((cell, ci) => {
        s.addShape(pres.shapes.RECTANGLE, { x: cx, y: ry, w: colW[ci], h: rhs[ri], fill: { color: header ? TEAL : zebra ? TEAL_TINT : WHITE }, line: { color: LINE, width: 1 } });
        const e = !header && emph.some(([er, ec]) => er === ri && ec === ci);
        s.addText(String(cell), {
          x: cx + 0.02, y: ry, w: colW[ci] - 0.04, h: rhs[ri], fontSize: header ? fontSize + 0.5 : fontSize,
          color: header ? WHITE : e ? TEAL_DK : INK, bold: header || e || (bold0 && ci === 0), fontFace: FF,
          align: "left", valign: "middle",
        });
        cx += colW[ci];
      });
      ry += rhs[ri];
    });
    return ry;
  }
  const strip = (s, txt, y = 4.62) => s.addText(txt, { x: 0.55, y, w: 8.9, h: 0.32, fontSize: 10, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle" });

  /* 01 COVER ---------------------------------------------------------------- */
  {
    page++;
    const s = track(pres.addSlide(), "The Guardrail, End to End", "cover");
    H.darkBase(s, "ITRI ICL subcontract · VLA-UAV Guardrail · NTUT AIoT Lab");
    s.addText("The Guardrail,\nEnd to End", { x: 0.55, y: 1.1, w: 8.4, h: 1.9, fontSize: 40, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 0.95, valign: "middle" });
    s.addText("How the safety layer works, and what 9-14 October delivered", { x: 0.55, y: 3.1, w: 8.6, h: 0.5, fontSize: 15, color: TEAL_TINT, fontFace: FF, valign: "middle" });
    s.addShape(pres.shapes.LINE, { x: 0.55, y: 4.05, w: 4.6, h: 0, line: { color: TEAL, width: 1 } });
    s.addText(`Status as of ${STATUS_DATE} · simulation and desktop measurements`, { x: 0.55, y: 4.2, w: 8.6, h: 0.35, fontSize: 12, color: TEAL_TINT, fontFace: FF, valign: "middle" });
    s.addNotes("This deck explains the Guardrail from end to end and then reports the work of 9 to 14 October. " +
      "Every number on a slide names its source file at the bottom of the slide. All results are from simulation or the development desktop; " +
      "the KPI-grade campaign runs on the Jetson Orin in November.");
  }

  /* 02 WHAT THE GUARDRAIL IS ------------------------------------------------ */
  {
    const s = content("What it is", "Every VLA command is checked before ArduPilot flies it");
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.55, y: 1.5, w: 8.9, h: 0.78, fill: { color: TEAL_TINT }, line: { color: TEAL_TINT }, rectRadius: 0.08 });
    s.addText("The Guardrail is the software layer between a vision-language-action model (VLA) and ArduPilot. It turns one signed flight policy into rules the VLA reads and a Safety Shield that checks, repairs or stops every command, ten times a second.", {
      x: 0.72, y: 1.5, w: 8.56, h: 0.78, fontSize: 11, color: TEAL_DEEP, fontFace: FF, bold: true, valign: "middle",
    });
    const y = 2.62, h = 0.92, gap = 0.24, w = (8.9 - 4 * gap) / 5;
    const nodes = [
      ["Policy DSL", "rules in, signed bundle out", "WP1"],
      ["Prefix Compiler", "rules as text for the VLA prompt", "WP2"],
      ["VLA", "image + text in, 4-D action out", null],
      ["Safety Shield", "check, repair, escalate at 10 Hz", "WP3"],
      ["ArduPilot", "flies the setpoint; GeoFence backstop", null],
    ];
    nodes.forEach(([t, sub, tag], i) => {
      const x = 0.55 + i * (w + gap);
      node(s, x, y, w, h, t, sub, tag ? { fill: TEAL_TINT, line: TEAL, tag } : {});
      if (i < nodes.length - 1) arrowR(s, x + w + 0.02, y + h / 2, x + w + gap - 0.02);
    });
    const yt = y + h, xa = 0.55 + w / 2, xb = 0.55 + 3 * (w + gap) + w / 2, yb = yt + 0.24;
    vline(s, xa, yt, yb, TEAL); hline(s, xa, yb, xb, TEAL); arrowD(s, xb, yb, yt + 0.02, TEAL);
    s.addText("the same rules, enforced", { x: xa + 1.55, y: yb - 0.02, w: 3.0, h: 0.26, fontSize: 9.5, color: TEAL_DK, fontFace: FF, italic: true, align: "center", valign: "top" });
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 0.55, y: 4.08, w: 8.9, h: 0.4, fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.07 });
    s.addText([{ text: "WP4  ", options: { bold: true } },
      { text: "Stress testing of the whole chain: scenarios, seeds, paraphrases, KPI report, signed replay" }],
    { x: 0.7, y: 4.08, w: 8.6, h: 0.4, fontSize: 10, color: WHITE, fontFace: FF, valign: "middle" });
    s.addText("ArduPilot runs its own inner loop; the Guardrail never drives the motors.", { x: 0.55, y: 4.55, w: 8.9, h: 0.36, fontSize: 10, color: INK, fontFace: FF, valign: "middle" });
    source(s, "Source: grant pages 'Grant overview' and 'Architecture constraints'; guardrail/shield.py (10 Hz monitor, 4-D action).");
    H.badge(s, page);
    s.addNotes("One sentence: the Guardrail sits between the VLA and ArduPilot. The operator writes one policy. WP1 turns it into a signed bundle. " +
      "WP2 compiles the same rules into a short text, the Constraint Summary Pack, that goes in front of the VLA's instruction. " +
      "The VLA proposes a 4-D velocity command. WP3, the Safety Shield, checks that command against the same rules ten times a second, repairs it or stops it, " +
      "and escalates through Brake, Loiter, RTL and Land when repairs are not enough. ArduPilot flies the result, and its own GeoFence stays as the final backstop. " +
      "WP4 stress-tests the whole chain and produces the KPI report.");
  }

  /* 03 TERMS ------------------------------------------------------------------ */
  {
    const s = content("Terms", "Thirteen terms, one line each");
    const terms = [
      ["VLA", "Vision-language-action model: camera image and instruction in, action out."],
      ["CSP", "Constraint Summary Pack: the policy's rules as short text for the VLA prompt."],
      ["Safety Shield", "Checks each command against the policy; repairs it or stops it."],
      ["P0 / P1 / P2", "Rule priority. P0 rules must never be broken: zones, clearance, stand-off."],
      ["P0 escape", "A tick on which the Shield flies a command that breaks a P0 rule."],
      ["Repair", "The Shield changes the command to the nearest one that keeps every rule."],
      ["Theta", "The repair size above which the Shield stops trusting repairs and escalates."],
      ["Tick", "One control step of 0.1 s; the Shield runs at 10 Hz."],
      ["Escalation FSM", "The Shield's ladder: Normal, Brake, Loiter, RTL, Land."],
      ["SITL", "ArduPilot's real flight code running on a PC against a simulator."],
      ["dev / hil / flight", "The grant's topologies: one desktop; desktop + Jetson Orin; Orin on the drone."],
      ["KPI-grade", "A run the grant counts: hil topology, real-time, signed evidence."],
      ["Null", "A trivial baseline printed beside a score: hover, pass-through, never-trigger."],
    ];
    const cw = 4.35, tw = 1.3, rh = 0.46, y0 = 1.5;
    terms.forEach(([t, d], i) => {
      const col = i < 7 ? 0 : 1, row = i < 7 ? i : i - 7;
      const x = 0.55 + col * (cw + 0.2), y = y0 + row * rh;
      s.addShape(pres.shapes.RECTANGLE, { x, y, w: cw, h: rh, fill: { color: row % 2 ? WHITE : TEAL_TINT }, line: { color: LINE, width: 0.75 } });
      s.addText(t, { x: x + 0.02, y, w: tw, h: rh, fontSize: 9.5, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(d, { x: x + tw, y, w: cw - tw - 0.02, h: rh, fontSize: 9, color: INK, fontFace: FF, valign: "middle" });
    });
    source(s, "Source: grant pages 'Grant overview', 'Architecture constraints', 'Safety Shield', 'Stress Testing'; guardrail/kpi.py; guardrail/fsm.py; guardrail/manifest.py.");
    H.badge(s, page);
    s.addNotes("These are the only terms the rest of the deck needs. The two to keep in mind: a P0 escape is a tick on which the Shield flies a command " +
      "that breaks a top-priority rule, and the grant wants that at zero. KPI-grade means a run the grant counts for its KPIs: the hil topology with the Jetson Orin, " +
      "real-time simulation and signed evidence. None of today's runs is KPI-grade.");
  }

  /* 04 THE VLA ------------------------------------------------------------------ */
  {
    const s = content("The pilot", "A VLA turns one frame and one instruction into one action");
    const y0 = 1.5, ch = 1.86, cw = 2.72, gap = 0.37;
    const cols = [
      [ic.camera, "In", [`Camera frame, ${V("vla_img")} px`, "Instruction in OpenVLA's template: \"What action should the robot take to ...?\"",
        `With the CSP on, the rules come first (budget ${V("wp2_budget")} tokens)`]],
      [ic.brain, "OpenVLA-7B", [`${V("vla_img_tokens")} image tokens plus the prompt`, `Room for ${V("vla_text_room")} text tokens`,
        "4-bit on the desktop GPU; runs beside the 10 Hz loop"]],
      [ic.arrows, "Out", [`${V("vla_act_tokens")} action tokens, a discretised value each`, `Read as forward, lateral (sign ${V("vla_lateral")}) and up`,
        "The grant's contract: 4-D (vx, vy, vz, yaw_rate)"]],
    ];
    cols.forEach(([icon, tag, lines], i) => {
      const x = 0.55 + i * (cw + gap);
      cardBox(s, x, y0, cw, ch, i === 1 ? TEAL_TINT : WHITE, i === 1 ? TEAL : LINE);
      s.addImage({ data: icon, x: x + 0.2, y: y0 + 0.17, w: 0.28, h: 0.28 });
      s.addText(tag.toUpperCase(), { x: x + 0.56, y: y0 + 0.14, w: cw - 0.7, h: 0.34, fontSize: 10, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
      s.addText(lines.map((l, k) => ({ text: l, options: { bullet: { indent: 11 }, breakLine: k < lines.length - 1 } })), {
        x: x + 0.1, y: y0 + 0.55, w: cw - 0.2, h: ch - 0.62, fontSize: 9, color: INK, fontFace: FF, valign: "top", paraSpaceAfter: 3,
      });
      if (i < 2) arrowR(s, x + cw + 0.05, y0 + ch / 2, x + cw + gap - 0.05, TEAL);
    });
    // Rates: 2 s of Shield ticks against one OpenVLA decision.
    const ty = 3.72, lx = 0.55, tx0 = 3.0, tx1 = 9.3, secs = 2.0, X = (t) => tx0 + (tx1 - tx0) * t / secs;
    s.addText("Safety Shield: a check every 0.1 s", { x: lx, y: ty - 0.06, w: 2.4, h: 0.3, fontSize: 9, color: INK, fontFace: FF, bold: true, valign: "middle" });
    for (let k = 0; k <= 20; k++) s.addShape(pres.shapes.RECTANGLE, { x: X(k * 0.1) - 0.025, y: ty, w: 0.05, h: 0.18, fill: { color: TEAL }, line: { color: TEAL } });
    const vy = ty + 0.42;
    s.addText(`OpenVLA-7B 4-bit probe: ${V("vla_s")} warm, ${V("vla_first")} first call`, { x: lx, y: vy - 0.13, w: 2.4, h: 0.44, fontSize: 9, color: INK, fontFace: FF, bold: true, valign: "middle" });
    s.addShape(pres.shapes.RECTANGLE, { x: X(0), y: vy, w: X(0.86) - X(0), h: 0.18, fill: { color: TEAL_DK }, line: { color: TEAL_DK } });
    s.addShape(pres.shapes.RECTANGLE, { x: X(0.86), y: vy, w: X(1.96) - X(0.86), h: 0.18, fill: { color: TEAL_TINT2 }, line: { color: TEAL_DK, width: 0.75, dashType: "dash" } });
    s.addText(V("vla_s"), { x: X(0.86) - 0.62, y: vy - 0.04, w: 0.58, h: 0.26, fontSize: 8, color: WHITE, fontFace: FF, bold: true, align: "right", valign: "middle", margin: 0 });
    s.addText(V("vla_first"), { x: X(1.96) - 0.62, y: vy - 0.04, w: 0.58, h: 0.26, fontSize: 8, color: TEAL_DEEP, fontFace: FF, bold: true, align: "right", valign: "middle", margin: 0 });
    s.addText("0 s", { x: X(0) - 0.3, y: vy + 0.2, w: 0.6, h: 0.2, fontSize: 7.5, color: GREY, fontFace: FF, align: "center", valign: "middle" });
    s.addText("2 s", { x: X(2) - 0.3, y: vy + 0.2, w: 0.6, h: 0.2, fontSize: 7.5, color: GREY, fontFace: FF, align: "center", valign: "middle" });
    s.addText("The control loop flies the latest action; the Shield checks it on every tick.", { x: 3.0, y: vy + 0.22, w: 6.0, h: 0.26, fontSize: 9, color: TEAL_DK, fontFace: FF, italic: true, align: "center", valign: "middle" });
    s.addText(`Probe: standalone, no simulator, two calls. In flight beside Project AirSim on one GPU: 0.11-0.15 Hz (OpenVLA-7B-based backends).`, {
      x: 0.55, y: vy + 0.5, w: 8.9, h: 0.3, fontSize: 8.5, color: GREY, fontFace: FF, valign: "middle",
    });
    [V("vla_flight_a"), V("vla_flight_b"), V("vla_probe"), V("vla_flight_cond")];   // the facts behind that line
    source(s, "Source: demo/real_vla_demo.py (prompt, image size, token counts, lateral sign); deploy/evidence/vla_backend_table.md (probe and in-flight records); grant 'Architecture constraints'.");
    H.badge(s, page);
    s.addNotes("The VLA is the pilot. It takes one camera frame and one instruction, in OpenVLA's own prompt template, and returns one action. " +
      "With the Constraint Summary Pack switched on, the rules are written in front of the instruction. OpenVLA writes its answer as seven action tokens; " +
      "the first three are read as forward, lateral and up, where the sign of the lateral axis is still marked unverified in the code, and the grant's contract is the 4-D velocity command. " +
      `In a standalone load probe on our desktop, with no simulator and two calls, OpenVLA-7B in 4-bit took ${V("vla_s")} once warm, ${V("vla_hz")}, and ${V("vla_first")} on the first call. ` +
      `In flight, with Project AirSim rendering on the same GPU, the OpenVLA-7B-based backends ran at ${V("vla_flight_b")} and ${V("vla_flight_a")}. The Shield runs every 0.1 seconds whatever the pilot's rate. ` +
      "So the model runs in its own thread, the 10 Hz loop always flies the latest action, and the Shield checks that action on every tick. " +
      "The backend table defines the rate protocol; OpenVLA's protocol rate and every Orin rate will be measured in the hil bring-up, and the KPI pilot will be chosen from that table.");
  }

  /* 05-07 FIGURES ---------------------------------------------------------------- */
  await figureSlide("End to end", "One signed policy drives the prompt, the Shield and the evidence", "explainer_flowchart_end_to_end",
    "Source: docs/img/explainer_flowchart_end_to_end.png; guardrail/shield.py (5 s at 0.1 s, 50 poses); guardrail/bundle.py; guardrail/fsm.py.",
    "Read it in three bands. Before take-off: the policy is written in the grant's YAML form, validated, merged across regulation, site and mission layers, " +
    "and signed into a bundle with its SHA-256 policy hash. The loop, every 0.1 seconds: camera frame and task go to the VLA, the VLA proposes a 4-D action, " +
    "the Shield forecasts 5 seconds ahead in 50 poses, checks and repairs, the escalation FSM decides the mode, the MAVLink adapter converts to ArduPilot's frame, and ArduPilot flies it. " +
    "Hot-apply events can change the rules at any time in flight; each one becomes a new policy generation. After the run: a run folder, a signed replay bundle, the KPI report.");
  await figureSlide("Architecture", "Four work packages share one rule format and one policy hash", "explainer_arch_components",
    "Source: docs/img/explainer_arch_components.png; module files in guardrail/, sitl/, tools/ and experiments/ (folders in the notes).",
    "Each box is a real module in the repository. They live in guardrail/, except geo_to_policy.py, kpi_report.py and export_episodes.py in tools/, " +
    "sweep_scenarios.py in experiments/, and the two ROS 2 nodes in sitl/. WP1 owns the rule model, the projection to local metres, the compiled zone index and the signed bundle. " +
    "WP2 ranks the rules into the token budget and writes the CSP. WP3 is the Shield itself: forecast, checks, repair operators, the FSM, the RTL path check, " +
    "the GeoFence export, the REST hot-apply and the two ROS 2 nodes. WP4 runs, scores and packages: scenarios, the harness, the gRPC service, the manifest, the KPI report and the exporters. " +
    "All of them read the same policy object and record the same policy hash.");
  await figureSlide("The Shield, per tick", "Every 0.1 s the Shield looks 5 s ahead and repairs before ArduPilot flies", "explainer_arch_shield",
    "Source: docs/img/explainer_arch_shield.png; guardrail/shield.py, guardrail/ir.py, guardrail/repair.py, guardrail/fsm.py (FSMConfig defaults).",
    "One tick. The Shield reads the vehicle state and the raw action, and the rules in force at this instant. It forecasts 5 seconds in 0.1-second steps. " +
    "A spatial index picks only the zones that forecast can reach. Rules are checked hard before soft and P0 before P1 and P2. " +
    "Violations go to the repair chain: the kinematic and altitude clamps first, then up to three passes of clearance, stand-off, corridor and lateral repairs. " +
    "The repaired action is checked again. The FSM then decides the mode, and the emitted action and its audit record leave the Shield.");

  /* 08 ESCALATION -------------------------------------------------------------------- */
  {
    const s = content("Escalation", "Five FSM states; the hold flies as ArduCopter BRAKE");
    const states = ["Normal", "Brake", "Loiter", "RTL", "Land"];
    const why = ["rules satisfied", "repairs inside GUIDED", `${V("fsm_n")} onsets in ${V("fsm_window")}, or a repair over theta`,
      `${V("fsm_n")} more onsets, ${V("fsm_loiter_timeout")} in Loiter, or a position that stays illegal or worsens`, "RTL cannot complete"];
    const y = 1.86, h = 0.5, gap = 0.36, w = (8.9 - 4 * gap) / 5;
    states.forEach((st, i) => {
      const x = 0.55 + i * (w + gap);
      const fill = i === 0 ? TEAL_TINT : i >= 3 ? TEAL_DK : TEAL;
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill }, line: { color: i === 0 ? TEAL : fill, width: 1.25 }, rectRadius: 0.08 });
      s.addText(st, { x, y, w, h, fontSize: 14, color: i === 0 ? TEAL_DEEP : WHITE, fontFace: FF, bold: true, align: "center", valign: "middle" });
      s.addText(why[i], { x: x - 0.12, y: y + h + 0.04, w: w + 0.24, h: 0.6, fontSize: 8.5, color: GREY, fontFace: FF, align: "center", valign: "top" });
      if (i < states.length - 1) arrowR(s, x + w + 0.04, y + h / 2, x + w + gap - 0.04);
    });
    const xn = 0.55 + w / 2, xl = 0.55 + 2 * (w + gap) + w / 2, xbk = 0.55 + (w + gap) + w / 2, yr = y - 0.2;
    vline(s, xl, y, yr, TEAL); vline(s, xbk, y, yr, TEAL); hline(s, xn, yr, xl, TEAL); arrowD(s, xn, yr, y - 0.01, TEAL);
    s.addText(`clear for ${V("fsm_recover")}: back to Normal`, { x: xl + 0.1, y: yr - 0.14, w: 2.8, h: 0.26, fontSize: 9, color: TEAL_DK, fontFace: FF, italic: true, valign: "middle" });
    const cy = 3.06, cw = (8.9 - 2 * 0.2) / 3, chh = 1.48;
    tagCard(s, 0.55, cy, cw, chh, ic.pause, "Why BRAKE",
      `ArduCopter LOITER takes its climb rate from the throttle stick; with no pilot it sinks. SITL: LOITER fell from 15 m to the ground in ${V("sitl_loiter")}, BRAKE held ${V("sitl_brake")}.`, 9);
    tagCard(s, 0.55 + cw + 0.2, cy, cw, chh, ic.plane, "First flight evidence",
      `13 Oct, close approach: the hold flew as BRAKE and held altitude at ${V("brake_hold")} over ${V("brake_ticks")} ticks. Its stopping drift reached the building margin; X7 then sent the aircraft home.`, 9, TEAL_TINT, TEAL);
    tagCard(s, 0.55 + 2 * (cw + 0.2), cy, cw, chh, ic.cogs, "Grant defaults",
      `${V("fsm_n")} onsets in ${V("fsm_window")}; theta ${V("fsm_theta")} (lateral / vertical); Loiter timeout ${V("fsm_loiter_timeout")}; back to Normal after ${V("fsm_recover")} clear. Set per mission in YAML.`, 9);
    strip(s, "Every mode change goes out over MAVLink; --hold-mode LOITER keeps the literal mode for a flight with a pilot.");
    source(s, "Source: guardrail/fsm.py (EDGES, FSMConfig); docs/DESIGN-escalation-fsm.md (SITL probe, Copter-4.5.7); CHANGELOG.md, 13 Oct (later); demo/out/pas_ap_close_on.");
    H.badge(s, page);
    s.addNotes("Small, trusted repairs keep the aircraft in GUIDED: that is Brake. Three violation onsets in five seconds, or a repair larger than theta, " +
      "step up to Loiter. Three more onsets, thirty seconds in Loiter, or a position that stays illegal or keeps getting worse sends the aircraft home, and if RTL cannot complete, it lands. " +
      "Why BRAKE: the grant names LOITER, but ArduCopter's LOITER takes its climb rate from the pilot's throttle stick, so with no pilot it descends. " +
      `In SITL, LOITER fell from 15 metres to the ground in ${V("sitl_loiter")} while BRAKE held altitude. ` +
      `On 13 October the hold flew as BRAKE for the first time in flight and held altitude at ${V("brake_hold")}. ` +
      "BRAKE stops the aircraft with the position controller, so it drifts while it stops. In the second hold that drift carried the aircraft into the building-clearance margin, " +
      "and edge X7 sent it home. A hold that stops clear of the margins is the next FSM fix.");
  }

  /* 09-10 FIGURES ---------------------------------------------------------------------- */
  await figureSlide("Topologies", "dev runs today; hil is defined and waits for the Orin", "explainer_arch_topologies",
    "Source: docs/img/explainer_arch_topologies.png; docs/DESIGN-topologies.md (ports, JetPack 6.2, Copter-4.5.7); grant 'Architecture constraints'.",
    "The grant names three topologies. dev is one desktop, and that is where everything runs today. We flew it as two rails: SITL with MAVROS 2 and the Shield node, " +
    "which records itself as dev, and Project AirSim with SITL over pymavlink, which is none of the grant's topologies. " +
    "On the desktop, Mission Planner's UDP port received the telemetry stream on 8 October, measured by a listener in its place; attaching Mission Planner itself is a next step. " +
    "hil, the grant's KPI configuration, keeps the simulators, ArduPilot SITL, the router and Mission Planner on the desktop and puts MAVROS 2, the Shield node and the VLA on the Jetson Orin. " +
    "It is defined in deploy/, its open items are listed in docs/DESIGN-projectairsim-ardupilot.md, and it runs when the Orin is up. flight moves the same Orin image onto the drone when the airframe exists.");
  await figureSlide("Evidence", "Every run leaves signed, replayable evidence", "explainer_arch_evidence",
    "Source: docs/img/explainer_arch_evidence.png; guardrail/protos/harness.proto (10 RPCs + GetStatus); tools/kpi_report.py; guardrail/manifest.py.",
    "Runs come from the flight rails, from the stress harness, and from the gRPC control plane that drives the harness. Each run writes the flight log, the audit records, " +
    "the events, every policy generation with its CSP, and the six-field manifest the grant asks for. The replay bundle digests and signs all of it and re-derives its own KPIs. " +
    "The KPI report keeps KPI-grade runs apart, prints n of m where nothing ran, never a silent zero, and puts a null beside each score.");

  /* 11 SECTION ------------------------------------------------------------------------------ */
  {
    page++;
    const s = track(pres.addSlide(), "Progress, 9-14 October", "section");
    H.darkBase(s, "Progress · 9-14 October 2026");
    s.addText("What the week delivered", { x: 0.55, y: 0.95, w: 8.6, h: 0.7, fontSize: 30, color: WHITE, fontFace: FF, bold: true, valign: "middle" });
    const items = [
      optional.onoff
        ? `Close approach, Shield on vs off: ${V("on_p0")} vs ${V("off_p0")} P0 escape ticks; ${V("von_p0")} vs ${V("voff_p0")} on video`
        : `Close approach: ${V("on_p0")} P0 escape ticks with the Shield on, ${V("off_p0")} of ${V("off_ticks")} with it off`,
      `Gate G1 re-passed after the GeoFence change: EKF error p95 ${V("g1_ekf")}`,
      `The harness flies the grant's 5 s forecast; moving-zone checks take ${V("moving_after")}`,
      `The grant's ${V("rpcs")} gRPC RPCs, REST hot-apply on the flying rails, signed bundles flown`,
      ...(optional.still ? [optional.onoff ? "Videos with the on-screen policy indicator: city (13 Oct), ArduPilot (14 Oct)"
        : "A city flight video with the on-screen policy indicator (13 Oct)"] : []),
    ];
    items.forEach((t, i) => {
      const yy = 1.88 + i * 0.56;
      s.addShape(pres.shapes.OVAL, { x: 0.6, y: yy + 0.06, w: 0.32, h: 0.32, fill: { color: TEAL }, line: { color: TEAL } });
      s.addText(String(i + 1), { x: 0.6, y: yy + 0.06, w: 0.32, h: 0.32, fontSize: 11, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle", margin: 0 });
      s.addText(t, { x: 1.08, y: yy, w: 8.3, h: 0.44, fontSize: 13, color: WHITE, fontFace: FF, valign: "middle" });
    });
    s.addText("Source: CHANGELOG.md, 12 Oct to 14 Oct; demo/out flight folders named on each slide.", { x: 0.55, y: 5.05, w: 8.6, h: 0.3, fontSize: 8.5, color: TEAL_TINT, fontFace: FF, italic: true, valign: "middle" });
    s.addNotes(`${optional.still ? "Five" : "Four"} results from the week, each on its own slide next: the Shield on and off on the same close approach` +
      `${optional.onoff ? ", flown on 13 October and recorded on video on 14 October" : ""}, Gate G1 passing again, the harness on the grant's forecast ` +
      `with a faster Shield, the interfaces for control, hot-apply and signatures${optional.still ? `, and ${optional.onoff ? "the flight videos" : "the city flight"} with the on-screen policy indicator` : ""}.`);
  }

  /* 12 CLOSE APPROACH ------------------------------------------------------------------------ */
  {
    // A P0 escape is a command the Shield flew; where the aircraft was is its own count.
    const s = content("Result · 13 October", "The Shield flew no P0-breaking command; the hold is the next fix");
    const y0 = 1.5, cw = 4.35, ch = 1.38;
    [[`${V("on_p0")}`, `of the ${V("on_shield_ticks")} ticks the Shield flew broke a P0 rule`, "Shield on", TEAL_TINT, TEAL],
      [`${V("off_p0")}`, `of ${V("off_ticks")} ticks flew a command that broke a P0 rule`, "Shield off", WHITE, LINE]].forEach(([big, lab, tag, fill, line], i) => {
      const x = 0.55 + i * (cw + 0.2);
      cardBox(s, x, y0, cw, ch, fill, line);
      s.addText(tag.toUpperCase(), { x: x + 0.24, y: y0 + 0.12, w: cw - 0.48, h: 0.3, fontSize: 10, color: TEAL, fontFace: FF, bold: true, charSpacing: 2, valign: "middle" });
      s.addText(big, { x: x + 0.24, y: y0 + 0.42, w: 1.7, h: 0.8, fontSize: 40, color: i === 0 ? TEAL_DK : CHARCOAL, fontFace: FF, bold: true, valign: "middle" });
      s.addText(lab, { x: x + 1.9, y: y0 + 0.42, w: cw - 2.1, h: 0.8, fontSize: 12, color: INK, fontFace: FF, valign: "middle" });
    });
    const cy = 3.0, fw = (8.9 - 2 * 0.2) / 3, fh = 1.56;
    tagCard(s, 0.55, cy, fw, fh, ic.shield, "Shield on, in detail",
      `${V("on_interventions")} interventions; Loiter twice, on repairs of ${V("loiter_repairs")}. The other ${V("on_ap_ticks")} ticks ArduPilot flew: two BRAKE holds, then RTL, home at ${V("rtl_home")}.`, 9);
    tagCard(s, 0.55 + fw + 0.2, cy, fw, fh, ic.clip, "Shield off, by rule",
      `P0 rules broken: building clearance ${V("off_bld_only")} ticks, stand-off ${V("off_so_only")}, both ${V("off_both")}. The P1 kinematic caps alone: ${V("off_kin_only")} more, not P0 escapes.`, 9);
    tagCard(s, 0.55 + 2 * (fw + 0.2), cy, fw, fh, ic.pause, "Position and next fix",
      `Inside the building margin: ${V("on_unsafe")} ticks with the Shield on, all under ArduPilot's hold and RTL; ${V("off_unsafe")} with it off. Next: a hold that stops clear of the margin.`, 9, TEAL_TINT, TEAL);
    strip(s, "Same pilot, route and signed bundle. Project AirSim + ArduPilot SITL over pymavlink: no grant topology, not KPI-grade.", 4.6);
    source(s, `Source: ${ON}/kpi.json and ${OFF}/kpi.json (P0 escapes, unsafe positions); flight_log.jsonl and metrics.json of both; CHANGELOG.md, 13 Oct (later).`);
    H.badge(s, page);
    s.addNotes("This is the first Shield-off control on the perception rail. The pilot is our follow controller tuned to be aggressive: it keeps closing until its detection box fills half the frame. " +
      "Both flights fly the same signed policy bundle. Two counts matter, and they are different. A P0 escape is a tick on which the Shield flew a command that breaks a P0 rule. " +
      `With the Shield on, the Shield flew ${V("on_shield_ticks")} ticks and none of them was an escape. With the Shield off, ${V("off_p0")} of ${V("off_ticks")} ticks were. ` +
      `With the Shield on it intervened ${V("on_interventions")} times and escalated to Loiter twice, when a repair exceeded theta. The hold flies as ArduCopter BRAKE, so for the other ${V("on_ap_ticks")} ticks ArduPilot itself flew the aircraft. ` +
      `The second position count is where the aircraft was. During the second hold, BRAKE's stopping drift carried the aircraft into the building-clearance margin, and the depth kept growing, so edge X7 sent it home. ` +
      `That gives ${V("on_unsafe")} ticks inside the margin with the Shield on, all ${V("on_unsafe_ap")} of them flown by ArduPilot's BRAKE and RTL, against ${V("off_unsafe")} with the Shield off. ` +
      `A hold that stops clear of the margin is the next FSM fix. The Shield-on flight ended at ${V("rtl_home")} when RTL reached home; the Shield-off flight ran the full 120 seconds. ` +
      `With the Shield off, ${V("off_kin_only")} more ticks broke only the P1 kinematic caps; they are not P0 escapes, so the P0 figure is ${V("off_p0")}.`);
  }

  /* 13 G1 --------------------------------------------------------------------------------------- */
  {
    const s = content("Result · 13 October", "Gate G1 passed again after the GeoFence change");
    const sx = 0.55, sw = 2.8, rows = [
      [[`${V("g1_runs")}`, "runs passed all four G1 conditions"], [`${V("g1_ekf")}`, `ArduPilot EKF error p95; limit ${V("g1_limit")}, frozen-position null ${V("g1_null")}`]],
      [[`${V("g1_climb")}`, "climb from rest on ArduPilot's own motor output"], [`${V("g1_land")}`, "landing distance from the take-off point"]],
      [[`${V("g1_fence")}`, "fence breaches"], ["0", "airborne collisions"]],
    ];
    rows.forEach((r, ri) => r.forEach(([v, l], ci) => {
      bigStat(s, sx + ci * (sw + 0.2), 1.42 + ri * 1.08, sw, v, l, { size: 24, labelSize: 9.5, labelH: 0.46 });
    }));
    const kx = 6.55, kw = 2.9;
    tagCard(s, kx, 1.5, kw, 1.6, ic.ok, "What G1 proves",
      "ArduPilot flies the city drone on Project AirSim's physics and sensors. Re-run after the parameters changed to FENCE_TYPE 7 and AVOID_ENABLE 0, so the rail can upload the policy's zones to the GeoFence.", 9, TEAL_TINT, TEAL);
    tagCard(s, kx, 3.24, kw, 1.28, ic.net, "Telemetry port",
      `8 Oct: Mission Planner's port, UDP 14550, received telemetry at ${V("mp_hz")} for the whole 120 s mission, measured by a listener in its place.`, 8.5);
    strip(s, "G1 is the closed loop the grant's perception-rail integration needs.", 4.6);
    source(s, "Source: demo/out/g1_20261013_203404 (g1_verdict.json); docs/data/progress_oct.json (g1); CHANGELOG.md, 13 Oct (later) and 8 Oct.");
    H.badge(s, page);
    s.addNotes("Gate G1 is the grant's bridge gate: ArduPilot, not the simulator's own controller, flies the city drone on Project AirSim's physics and sensors. " +
      `It first passed on 8 October. On 13 October the drone's parameter file changed so the rail can upload the policy's zones to the GeoFence, so we ran it again: three of three runs, EKF error p95 ${V("g1_ekf")} ` +
      `against a 1 metre limit, where a frozen-position estimate would score about 19 metres. Landing within ${V("g1_land")} of take-off, no fence breach, no collision.`);
  }

  /* 14 HARNESS ------------------------------------------------------------------------------------ */
  {
    const s = content("Result · 13 October", "The stress harness now flies the grant's 5 s forecast");
    const lx = 0.55, lw = 4.1;
    cardBox(s, lx, 1.5, lw, 1.5);
    s.addText("FORECAST IN EVERY HARNESS SHIELD", { x: lx + 0.2, y: 1.58, w: lw - 0.4, h: 0.3, fontSize: 9, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
    s.addText([{ text: `Before: ${V("forecast_old")}`, options: { color: GREY, fontSize: 11, breakLine: true } },
      { text: `Now: ${V("forecast")}`, options: { color: TEAL_DK, fontSize: 15, bold: true } }],
    { x: lx + 0.2, y: 1.9, w: lw - 0.4, h: 1.0, fontFace: FF, valign: "middle" });
    cardBox(s, lx, 3.15, lw, 1.32);
    s.addText("CLOSED-LOOP WIND MODEL", { x: lx + 0.2, y: 3.23, w: lw - 0.4, h: 0.3, fontSize: 9, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
    s.addText(`Every harness velocity goes through a model of ArduCopter's position controller; steady wind is rejected up to ${V("wind")} (derived from the SITL frame).`, {
      x: lx + 0.2, y: 3.55, w: lw - 0.4, h: 0.86, fontSize: 9.5, color: INK, fontFace: FF, valign: "top",
    });
    // The library sweep as one stacked bar.
    const rx = 4.95, rw = 4.5;
    s.addText(`Library sweep: ${V("sweep_eps")} episodes`, { x: rx, y: 1.5, w: rw, h: 0.34, fontSize: 12, color: INK, fontFace: FF, bold: true, valign: "middle" });
    const parts = [[+V("sweep_pass"), "pass", TEAL_DK, WHITE], [+V("sweep_tracked"), "tracked", TEAL, WHITE], [+V("sweep_known"), "known failure", TEAL_TINT2, TEAL_DEEP]];
    const total = +V("sweep_eps");
    let bx = rx;
    parts.forEach(([n, , fill, fg]) => {
      const bw = rw * n / total;
      s.addShape(pres.shapes.RECTANGLE, { x: bx, y: 1.95, w: bw, h: 0.46, fill: { color: fill }, line: { color: WHITE, width: 1 } });
      s.addText(String(n), { x: bx, y: 1.95, w: bw, h: 0.46, fontSize: 11, color: fg, fontFace: FF, bold: true, align: "center", valign: "middle", margin: 0 });
      bx += bw;
    });
    parts.forEach(([, lab, fill], i) => {
      const lxx = rx + i * 1.5;
      s.addShape(pres.shapes.RECTANGLE, { x: lxx, y: 2.55, w: 0.16, h: 0.16, fill: { color: fill }, line: { color: fill } });
      s.addText(lab, { x: lxx + 0.2, y: 2.47, w: 1.28, h: 0.32, fontSize: 9, color: GREY, fontFace: FF, valign: "middle" });
    });
    s.addText(`${V("sweep_fail")} unexpected failures. Tracked: GPS noise, GPS dropout, wind beyond authority, gusts; gated on what must hold, never counted as a pass. Known failure: a recorded defect that fails its gates.`, {
      x: rx, y: 2.86, w: rw, h: 0.62, fontSize: 9, color: INK, fontFace: FF, valign: "top",
    });
    s.addText("A labelled 3 s arm is kept apart for comparison and never enters a headline.", { x: rx, y: 3.55, w: rw, h: 0.5, fontSize: 9, color: GREY, fontFace: FF, valign: "top" });
    strip(s, "Headless and faster than real time: functional evidence for the hil campaign, not KPI-grade.", 4.6);
    source(s, "Source: docs/data/scenario_sweep.json (python experiments/sweep_scenarios.py --publish); CHANGELOG.md, 13 Oct; guardrail/shield.py.");
    H.badge(s, page);
    s.addNotes("The harness is where the KPI numbers will come from, so it must fly the Shield the grant describes. Until 13 October its Shields forecast 3 seconds in 0.5-second steps, 6 poses; " +
      "the grant locks 5 seconds in 0.1-second steps, 50 poses. All three harness Shields now use the grant's values. " +
      "The harness also flies every velocity through a closed-loop model of ArduCopter's position controller, so wind is rejected as ArduPilot would reject it. " +
      `The republished library: ${V("sweep_eps")} episodes, ${V("sweep_pass")} pass, ${V("sweep_tracked")} tracked, ${V("sweep_known")} known failures, which are recorded defects, and ${V("sweep_fail")} unexpected failures. The KPI slide carries its numbers.`);
  }

  /* 15 SHIELD SPEED ------------------------------------------------------------------------------------ */
  {
    const s = content("Result · 13 October", `Moving-zone checks fell from ${V("moving_before")} to ${V("moving_after")}`);
    const y0 = 1.5, cw = (8.9 - 2 * 0.2) / 3, ch = 1.45;
    [[`${V("moving_after")}`, `moving zones, from ${V("moving_before")}: rule-check median at the grant's 50-rule load`],
      [`${V("static_ms")}`, "static zones, same load and session"],
      [`${V("zones_built")}`, "zone polygons built in 200 checks; every zone was rebuilt on every check before"]].forEach(([v, l], i) => {
      const x = 0.55 + i * (cw + 0.2);
      cardBox(s, x, y0, cw, ch, i === 0 ? TEAL_TINT : WHITE, i === 0 ? TEAL : LINE);
      s.addText(v, { x: x + 0.15, y: y0 + 0.12, w: cw - 0.3, h: 0.62, fontSize: 20, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle" });
      s.addText(l, { x: x + 0.15, y: y0 + 0.76, w: cw - 0.3, h: 0.62, fontSize: 9.5, color: GREY, fontFace: FF, valign: "top" });
    });
    const fy = 3.12, fh = 1.36;
    tagCard(s, 0.55, fy, cw, fh, ic.ok, "Same decisions", "Identical to the old path on every compared sample.", 9.5);
    tagCard(s, 0.55 + cw + 0.2, fy, cw, fh, ic.layers, "The null", `The per-fence loop the index replaced takes ${V("null_loop")}.`, 9.5);
    tagCard(s, 0.55 + 2 * (cw + 0.2), fy, cw, fh, ic.speed, "The budget", `The grant allows ${V("t_check")} per check. Desktop figures; the Orin profile waits for the board (bring-up window 7-24 Oct).`, 9.5);
    strip(s, "How: anchor_reach builds a moving zone's polygon only where the 5 s forecast can meet it.", 4.6);
    source(s, "Source: CHANGELOG.md, 13 Oct (moving-zone fix, same-session medians); guardrail/ir.py anchor_reach; grant 'Policy DSL' acceptance KPIs.");
    H.badge(s, page);
    s.addNotes("The grant allows 5 milliseconds per rule check at 50 active rules. On GitHub's runner the moving-zone path measured 5.71 milliseconds, over budget, because 70 percent of the time went into rebuilding every zone's polygon on every check. " +
      `The Shield now builds a moving zone only where the forecast can reach it: ${V("zones_built")} in 200 checks. The median fell from ${V("moving_before")} to ${V("moving_after")} on the desktop, with identical decisions. ` +
      "These are desktop numbers. The same script is ready for the Orin; the board is not available yet, its bring-up window is 7 to 24 October, and the Orin figure is the one the grant measures.");
  }

  /* 16 INTERFACES -------------------------------------------------------------------------------------- */
  {
    const s = content("Result · 9-13 October", "Control, hot-apply and trust interfaces are in place");
    const cw = (8.9 - 2 * 0.2) / 3, ch = 1.48, gy = 0.16;
    const cards = [
      [ic.net, "gRPC control plane", `The grant's ${V("rpcs")} RPCs plus GetStatus, over mutual TLS. Drives the headless harness.`],
      [ic.sync, "REST hot-apply", "On both SITL rails' Shield: a zone, window switch or corridor swap lands between ticks as a new policy generation."],
      [ic.sign, "Signed bundles", "Ed25519 over the manifest, which carries the policy hash. 13 Oct: the first signed flight on the perception rail."],
      [ic.pause, "Action watchdog", `No pilot action for ${V("watchdog")}, five ticks: the ROS 2 node holds the aircraft through the FSM.`],
      [ic.clock, "One rule clock", "The Shield and every CSP read the same clock, so a time window switches the rules on the flying rails. Verified headless; a flight with a window is next."],
      [ic.history, "Re-runnable episodes", "Self-contained harness bundles; tools/rerun_episode.py re-flies one and compares the log by hash."],
    ];
    cards.forEach(([icon, tag, body], i) => {
      const col = i % 3, row = Math.floor(i / 3);
      tagCard(s, 0.55 + col * (cw + 0.2), 1.5 + row * (ch + gy), cw, ch, icon, tag, body, 9);
    });
    source(s, "Source: CHANGELOG.md, 12 Oct and 13 Oct; guardrail/protos/harness.proto; docs/DESIGN-policy-identity.md; sitl/ros2_shield_node.py.");
    H.badge(s, page);
    s.addNotes("Six interfaces the grant asks for. The gRPC control plane now has all ten of the grant's RPCs, over mutual TLS, and drives the headless harness. " +
      "The REST hot-apply is built into the flying Shield of both SITL rails: an event lands between ticks, under the rail's lock, and is recorded as a new policy generation with its receipt-to-swap time. " +
      "It is tested headless; the first in-flight event is a next step. " +
      "Policy bundles are signed with Ed25519, and on 13 October the perception rail flew a signed bundle for the first time, signature verified. " +
      `If the pilot's action stream stops for ${V("watchdog")}, the ROS 2 node holds the aircraft the same way the FSM does. ` +
      "The Shield and the CSP read one rule clock, so a time window switches the rules on the flying rails; that is verified headless, and the first flight with a time window is next. " +
      "And any harness episode can be re-flown from its bundle and compared by hash.");
  }

  /* 17 STILL FROM THE 13 OCT VIDEO (optional) -------------------------------------------------------------- */
  if (optional.still) {
    const s = content("City flight · 13 October", "The policy indicator shows the pilot routing around the zone");
    WHERE.still = page;
    const im = await photo(stillPath);
    const stats = [[V("demo_nfz_s"), "inside the no-fly zone"], [V("demo_p0"), "P0 escape ticks"],
      [V("demo_interventions"), "Shield correction: a speed clamp"], [V("demo_within30"), `of ticks with the red car within ${V("demo_30m")}`]];
    const pilot = `Pilot: our follow controller on Project AirSim alone (no ArduPilot); ${V("demo_ticks")} ticks at ${V("demo_alt")}, every command checked.`;
    // The HUD's banner and rule panel are unreadable at frame size, so they are
    // shown again, enlarged, cut from the 1280 x 720 camera panel of the still.
    const meta = await sharp(stillPath).metadata();
    if (meta.width === 1280 && meta.height === 720) {
      const crop = async (r) => {
        const b = await sharp(stillPath).extract(r).png().toBuffer();
        return { data: `image/png;base64,${b.toString("base64")}`, aspect: r.width / r.height };
      };
      const banner = await crop({ left: 336, top: 6, width: 610, height: 44 });
      const panel = await crop({ left: 796, top: 50, width: 480, height: 186 });
      const fw = 4.3;
      const f = H.figure(s, im, 0.55, 1.45, fw, fw / im.aspect);
      s.addText(`Frame at t = ${STILL_HUD_T} s (HUD clock): the camera view with the policy indicator; the rule panel shows the zone ${V("demo_zone_d")} away, status AVOID.`, {
        x: 0.55, y: f.y + f.h + 0.03, w: fw, h: 0.42, fontSize: 8.5, color: GREY, fontFace: FF, italic: true, valign: "top",
      });
      const rx = 5.05, rw = 4.4;
      s.addText("BANNER AND RULE PANEL, ENLARGED", { x: rx, y: 1.4, w: rw, h: 0.28, fontSize: 9, color: TEAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
      const b = H.figure(s, banner, rx, 1.72, rw, rw / banner.aspect);
      const p = H.figure(s, panel, rx, b.y + b.h + 0.08, rw, rw / panel.aspect);
      s.addText(pilot, { x: rx, y: p.y + p.h + 0.04, w: rw, h: 0.42, fontSize: 8.5, color: TEAL_DK, fontFace: FF, bold: true, valign: "top" });
      const ws = [1.8, 1.8, 2.6, 2.7];
      let sx = 0.55;
      stats.forEach(([v, l], i) => {
        s.addText(v, { x: sx, y: 4.3, w: ws[i], h: 0.34, fontSize: 16, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle" });
        s.addText(l, { x: sx, y: 4.64, w: ws[i], h: 0.26, fontSize: 8.5, color: GREY, fontFace: FF, valign: "top" });
        sx += ws[i];
      });
    } else {
      const fw = 5.3;
      const f = H.figure(s, im, 0.55, 1.48, fw, fw / im.aspect);
      s.addText(`Frame at t = ${STILL_HUD_T} s: the rule panel shows the zone ${V("demo_zone_d")} away (AVOID).`, {
        x: 0.55, y: f.y + f.h + 0.03, w: fw, h: 0.45, fontSize: 8.5, color: GREY, fontFace: FF, italic: true, valign: "top",
      });
      const rx = 6.1, rw = 3.35;
      stats.forEach(([v, l], i) => {
        const yy = 1.4 + i * 0.68;
        s.addText(v, { x: rx, y: yy, w: rw, h: 0.42, fontSize: 18, color: TEAL_DK, fontFace: FF, bold: true, valign: "middle" });
        s.addText(l, { x: rx, y: yy + 0.4, w: rw, h: 0.28, fontSize: 9, color: GREY, fontFace: FF, valign: "top" });
      });
      s.addText(pilot, { x: rx, y: 4.16, w: rw, h: 0.76, fontSize: 9, color: TEAL_DK, fontFace: FF, bold: true, valign: "top" });
    }
    source(s, `Source: ${DM}/metrics.json, kpi.json and flight_log.jsonl; video ${DEMO_SHARE_VIDEO}.`);
    H.badge(s, page);
    s.addNotes("A frame from the 13 October city flight video. The left panel is the drone's camera with the policy indicator ITRI asked for on 30 September: " +
      "the banner names the most important rule event, the panel lists every rule with its live value, and the map shows the zone in red. " +
      `The pilot here is our follow controller, which saw the zone ahead and routed around it; the banner says it is routing around the zone, and the panel marks the zone AVOID. ` +
      `Over the flight the drone spent ${V("demo_nfz_s")} in the zone, the Shield corrected one command, a speed clamp from ${V("demo_clamp")}, and the car stayed within 30 metres on ${V("demo_within30")} of ticks. ` +
      `The policy was loaded from its YAML file for this run. The full video is ${DEMO_SHARE_VIDEO}.`);
  }

  /* 18 ARDUPILOT SHIELD ON / OFF STILLS (optional; the lead adds the images) --------------------------- */
  if (optional.onoff) {
    const s = content("Recorded close approach · 14 October", "Same pilot and route: the Shield corrects, the control arm breaks P0");
    WHERE.onoff = page;
    const a = await photo(onPath), b = await photo(offPath);
    const cw = 4.35;
    let bottom = 0;
    [[a, `Shield on · ${V("von_p0")} P0 escapes in ${V("von_shield_ticks")} Shield ticks`], [b, `Shield off · ${V("voff_p0")} P0 escapes in ${V("voff_ticks")} ticks`]].forEach(([im, lab], i) => {
      const x = 0.55 + i * (cw + 0.2);
      s.addText(lab.toUpperCase(), { x, y: 1.45, w: cw, h: 0.32, fontSize: 10, color: i === 0 ? TEAL : CHARCOAL, fontFace: FF, bold: true, charSpacing: 1.5, valign: "middle" });
      // The box is the still's own height (2.45 in for a 16:9 still at this
      // width), so it sits right under its label and ends at 4.27 in.
      const f = H.figure(s, im, x, 1.82, cw, cw / im.aspect);
      bottom = Math.max(bottom, f.y + f.h);
    });
    // Below the stills, both counts the deck keeps apart: P0 escapes (the
    // labels) and where the aircraft was (this strip), which on 14 Oct includes
    // ticks inside the margin while the Shield still flew. The PDF has no
    // notes, so it is on the slide.
    s.addText([
      { text: `Inside the building margin: ${V("von_unsafe")} ticks with the Shield on, ${V("von_unsafe_shield")} of them while it flew (up to ${V("von_depth")} deep) and ${V("von_unsafe_ap")} under RTL; ${V("voff_unsafe")} with it off.`, options: { breakLine: true } },
      { text: "Not KPI-grade (pymavlink, no grant topology). The detector boxed a red traffic signal: this tests the Shield, not tracking." },
    ], {
      x: 0.55, y: bottom + 0.08, w: 8.9, h: 0.5, fontSize: 9.5, color: TEAL_DK, fontFace: FF, bold: true, valign: "top",
    });
    source(s, `Source: demo/out/pas_ap_video_{on,off}_20261014 (kpi.json, metrics.json, flight_log.jsonl); stills at video t = ${ONOFF_ON_T} s and t = ${ONOFF_OFF_T} s.`);
    H.badge(s, page);
    s.addNotes("The recorded close approach of 14 October. Left, the Shield on: FSM Brake, the banner reporting the command pushed away from the building, and the detector's box over the top of a red traffic signal, its lit lamp directly below; two seconds earlier the box sat on the signal head beside the lit lamp. Right, the Shield off: the raw command flown into the building-clearance margin. " +
      "Same pilot, route and signed policy bundle as on 13 October. " +
      `With the Shield on, the Shield flew ${V("von_shield_ticks")} of ${V("von_ticks")} ticks and none was a P0 escape. ` +
      `At ${V("von_correct_t")} it held the stand-off from the detector's target, the red traffic signal, and the FSM stepped to Brake; from the next tick it pushed each command away from the building. ` +
      `The aircraft was flying at ${V("von_speed")} and carried ${V("von_carry")} further while it slowed. The Shield's forecast is ${V("shield_cv")}: it advances the commanded velocity, not the aircraft's own, ` +
      `so each push-away read as recovering, ${V("von_forecast")} on the first, while the aircraft was still stopping. ` +
      `It entered the building margin by up to ${V("von_depth")} under the Shield's own corrected commands; the illegal depth kept growing, so edge X7 sent it home, with no hold in between, and the run ends when RTL reaches home. ` +
      `With the Shield off, ${V("voff_p0")} of ${V("voff_ticks")} ticks flew a P0-breaking command: building clearance alone ${V("voff_bld_only")}, stand-off alone ${V("voff_so_only")}, both ${V("voff_both")}. ` +
      `Where the aircraft was is the second count: inside the building margin on ${V("von_unsafe")} ticks with the Shield on, ${V("von_unsafe_shield")} of them while the Shield still flew and ${V("von_unsafe_ap")} under ArduPilot's RTL, and on ${V("voff_unsafe")} ticks with it off. ` +
      "Two fixes come before the next close approach: a hold that stops clear of the margin, and a forecast that starts from the aircraft's own velocity. " +
      "The subject's ground truth was right; the detector boxed a red traffic signal, not the car, and this rail has no start gate, so the pair demonstrates the Shield, not the tracker. " +
      `Across the four close flights ${V("est_near_car")} estimates came within 6 metres of the car. On this pair the car was in the camera's view on ${V("von_view_in")} of ${V("von_view_est")} estimate ticks with the Shield on and ${V("voff_view_in")} of ${V("voff_view_est")} with it off. ` +
      `The 13 Oct pair read ${V("on_p0")} and ${V("off_p0")} of ${V("off_ticks")}.`);
  }

  /* 19 ACCEPTANCE KPIs ------------------------------------------------------------------------------------- */
  {
    const s = content(`Acceptance KPIs · status on ${STATUS_DAY}`, "Where each acceptance KPI stands, and the next step");
    const rows = [
      ["Grant KPI", "Target", "Current value (headless or desktop)", "Evidence", "Next step"],
      ["Mission success rate", "tracked", `${V("mission")} over ${V("lib_eps")} Shield-on episodes (of ${V("sweep_eps")}; ${V("lib_controls")} Shield-off control)`, "scenario_sweep.json", "hil campaign, 16-23 Nov"],
      ["P0 violation escape rate", V("t_p0"), `${V("lib_p0")}: ${V("lib_p0_ticks")} ticks in ${V("lib_p0_eps")} episodes of one known-failure family; close approach ${V("on_p0")} in ${V("on_shield_ticks")} Shield ticks`, "scenario_sweep.json, pas_ap_close_on/", "fix the window-opens-over-vehicle family; target 0"],
      ["Fail-safe trigger correctness", V("t_failsafe"), `${V("failsafe")} (${V("failsafe_frac")}); never-trigger null ${V("failsafe_null")}`, "scenario_sweep.json", `fix the FSM paths behind ${V("failsafe_false")} false triggers; target ${V("t_failsafe")}`],
      ["Mean repair magnitude", "tracked", `${V("repair_mag")} (scripted stress pilots)`, "scenario_sweep.json", "hil campaign with the KPI pilot"],
      ["Mean time to safe", "tracked", `${V("tts")}; close approach ${V("on_tts")}, under ArduPilot's hold and RTL`, "scenario_sweep.json, pas_ap_close_on/", "hil campaign, 16-23 Nov"],
    ];
    tableV(s, rows, { y: 1.48, colFrac: [0.19, 0.08, 0.3, 0.2, 0.23], rhs: [0.36, 0.5, 0.6, 0.5, 0.46, 0.5], fontSize: 8.5, emph: [[2, 2], [3, 2]] });
    strip(s, "None of these values is KPI-grade; the KPI-grade values come from the hil campaign on the Jetson Orin.", 4.58);
    source(s, "Source: docs/data/scenario_sweep.json (_rollup._all_shield_on, 5 s forecast); demo/out/pas_ap_close_on/kpi.json; grant 'Grant overview' and 'Stress Testing' (targets).");
    H.badge(s, page);
    s.addNotes("The grant's five acceptance KPIs, each with its current development value. All of them come from the headless harness library or a desktop flight, so none is KPI-grade. " +
      "Two hard targets are open, and both fixes come before the campaign on 16 November. " +
      `The P0 escape rate in the library is ${V("lib_p0")} against a target of 0: ${V("lib_p0_ticks")} ticks in ${V("lib_p0_eps")} episodes, the ${V("lib_twin_cells")} cells of one known-failure family where a time window opens over the aircraft and no action clears the forecast. ` +
      `Fail-safe trigger correctness is ${V("failsafe")} against the grant's 0.99, and below the never-trigger null of ${V("failsafe_null")}. ` +
      `${V("failsafe_g3")} of the ${V("failsafe_false")} false triggers pass through edge G3, a repair larger than theta, which tells us where to work. ` +
      `Mission success, repair magnitude and time to safe are tracked without a hard target. The close approach's ${V("on_tts")} time to safe is how long the aircraft took to leave the building margin, under ArduPilot's hold and RTL.`);
  }

  /* 20 COMPONENT KPIs ---------------------------------------------------------------------------------------- */
  {
    const s = content(`Component KPIs · status on ${STATUS_DAY}`, "Component KPIs against the grant's thresholds");
    const rows = [
      ["Component KPI", "Target", "Current value", "Evidence", "Next step"],
      ["WP1 policy round trip", "100 %", `${V("wp1_rt")} policies; ${V("wp1_tamper")} tampered bundles and ${V("wp1_neg")} broken policies refused`, "wp1_roundtrip.json", "re-run on every policy change"],
      ["WP1 hot-apply latency", `≤ ${V("t_hotapply")}`, `${V("hotapply_ms")} median, p99 ${V("hotapply_p99")} (desktop, 300 spawns)`, "DESIGN-shield-enforcement.md", "repeat on the Orin"],
      ["WP1 monitor query, 50 rules", `≤ ${V("t_check")}`, `${V("check_ms")} median near a zone (progress profile); moving zones ${V("moving_after")} (13 Oct session)`, "progress_oct.json, CHANGELOG", "Orin profile"],
      ["WP2 P0 coverage", V("t_p0cov"), `${V("wp2_p0")} at ${V("wp2_budget")} tokens; at 120 tokens still 105/105, where a naive cut keeps ${V("wp2_naive").replace(" at 120", "")}`, "csp_kpis.json", "CSP on/off arms in the campaign"],
      ["WP3 monitor tick", `≤ ${V("t_tick")}`, `p99 ${V("replay_tick_p99")} over ${V("replay_n")} replayed simulator flight ticks (${V("profile_date")} profile, desktop)`, "shield_tick_profile.json", "Orin profile"],
      ["WP4 per-paraphrase robustness", "tracked", `${V("paraphrases")} stored paraphrases ready; needs a text-reading pilot`, "validation_report.json", "hil campaign with the chosen VLA"],
    ];
    tableV(s, rows, { y: 1.48, colFrac: [0.2, 0.1, 0.32, 0.215, 0.165], rhs: [0.36, 0.5, 0.46, 0.5, 0.5, 0.5, 0.46], fontSize: 8.5 });
    source(s, "Source: docs/data/progress_oct.json, wp1_roundtrip.json, csp_kpis.json; deploy/evidence/dev/shield_tick_profile.json; experiments/paraphrases/validation_report.json; docs/DESIGN-shield-enforcement.md.");
    H.badge(s, page);
    s.addNotes("The component KPIs each work package owns. Policy round trip and refusals are at 100 percent. Hot-apply takes about a millisecond against 50. " +
      `The rule check is well under 5 milliseconds on the desktop, and the whole tick's p99 over ${V("replay_n")} replayed simulator flight ticks, from the ${V("profile_date")} profile, is ${V("replay_tick_p99")} against 100. ` +
      "The CSP carries every P0 rule, and at a tight 120-token budget it still carries all of them where a naive cut drops one. " +
      "Per-paraphrase robustness needs a pilot that reads text, so it will be measured in the campaign with the chosen VLA. Every timing here will be repeated on the Orin in the hil bring-up.");
  }

  /* 21 NEXT STEPS --------------------------------------------------------------------------------------------- */
  {
    const s = content(`Plan to ${V("t_final")}`, "Next steps to the final delivery");
    const rows = [
      ["Step", "When", "Target"],
      ["Jetson Orin bring-up (hil), once the board is available", "7-24 Oct", "first-boot check passes; Shield profiled against 5 ms / 100 ms"],
      ["Fly the FSM and GeoFence backstop on the MAVROS 2 rail (code in place)", "13-31 Oct", "escalation and fence upload recorded in a dev-rail flight"],
      ["Stop clear of P0 margins: the hold, and a forecast from the aircraft's own velocity", "before the next close approach", "0 P0-unsafe ticks while the Shield or the hold flies"],
      ["Fail-safe false triggers; the window-opens family", "before 16 Nov", `correctness ${V("t_failsafe")}; 0 P0 escape ticks in the library`],
      ["A start gate on the perception rail, as follow_vlm has", "before this rail scores tracking", "the follow starts on the car, not on a red traffic signal"],
      ["VLA reading the CSP on the ArduPilot rail", "27 Oct-13 Nov", "CSP on/off and paraphrase arms flown"],
      ["hil KPI stress campaign", "16-23 Nov", "the five acceptance KPIs, KPI-grade"],
      ["KPI report, final demo, signed final report", "23-30 Nov", `final delivery, ${V("t_final")}`],
    ];
    tableV(s, rows, { y: 1.44, colFrac: [0.4, 0.19, 0.41], rhs: [0.34, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39], fontSize: 8.5 });
    source(s, "Source: project plan of 6 Oct (tools/deck/build_progress_oct_deck.js, PLAN); grant 'Grant overview' (delivery dates); CHANGELOG.md, 13 Oct ('Not fixed'), 13 Oct (later) and 14 Oct; demo/out/pas_ap_video_on_20261014.");
    H.badge(s, page);
    s.addNotes("The path to 30 November. The Jetson Orin is not available yet, so nothing has run on it; its bring-up window is 7 to 24 October. " +
      "The Orin bring-up and flying the Shield node's escalation and GeoFence backstop on the MAVROS 2 rail are planned in parallel through October; the code for both is in place in the ROS 2 node. " +
      "Before the next close approach, two fixes so the aircraft stops clear of the P0 margins. On 13 October the BRAKE hold's stopping drift carried it in, so the hold gets a stop that stays clear. " +
      `On 14 October it entered the margin by up to ${V("von_depth")} while the Shield still flew, because the Shield's ${V("shield_cv")} forecast advances the commanded velocity, not the aircraft's own; the second fix starts the forecast from the aircraft's own velocity. ` +
      `The perception rail's ground truth was right: Car_10's poses lie on follow_vlm's validated track, median ${V("truth_track")} apart. Its detector boxed a red traffic signal, not the car. ` +
      `The rail starts with the car ${V("car_start")} away and has no start gate, while follow_vlm waits until it sees the car. Until the rail has that gate, its follow scores stay at 0 and its flights test the Shield, not tracking. ` +
      "Before the campaign starts on 16 November, the fail-safe false triggers and the time-window family get fixed. " +
      "From late October the VLA flies the ArduPilot rail with the CSP in its prompt. The KPI campaign runs on the Orin from 16 to 23 November, followed by the report, the final demo and the signed final report: " +
      "the four components, the KPI report, the perception-rail integration and the signed final report make the final delivery.");
  }

  /* 22 CLOSING -------------------------------------------------------------------------------------------------- */
  {
    page++;
    const s = track(pres.addSlide(), "Three things to remember", "closing");
    H.darkBase(s, "Summary");
    s.addText("Three things to remember", { x: 0.55, y: 0.9, w: 8.6, h: 0.7, fontSize: 30, color: WHITE, fontFace: FF, bold: true, valign: "middle" });
    const lines = [
      "The Guardrail checks every VLA command against one signed policy, ten times a second, before ArduPilot flies it.",
      optional.onoff
        ? `Same pilot, same route: ${V("on_p0")} P0 escape ticks with the Shield on, ${V("off_p0")} of ${V("off_ticks")} with it off (${V("voff_p0")} of ${V("voff_ticks")} on video); next, stop clear of the margins.`
        : `Same pilot, same route: ${V("on_p0")} P0 escape ticks with the Shield on, ${V("off_p0")} of ${V("off_ticks")} with it off; next, stop clear of the margins.`,
      `KPI-grade numbers come from the hil campaign on the Jetson Orin, 16-23 Nov. Before it: fail-safe correctness ${V("failsafe")} to ${V("t_failsafe")}, library P0 escape ${V("lib_p0")} to ${V("t_p0")}.`,
    ];
    lines.forEach((t, i) => {
      const yy = 1.9 + i * 0.82;
      s.addText(String(i + 1), { x: 0.55, y: yy, w: 0.5, h: 0.6, fontSize: 26, color: TEAL_TINT, fontFace: FF, bold: true, valign: "middle" });
      s.addText(t, { x: 1.1, y: yy, w: 8.3, h: 0.66, fontSize: 13, color: WHITE, fontFace: FF, valign: "middle" });
    });
    s.addText("Thank you", { x: 0.55, y: 4.6, w: 8.6, h: 0.5, fontSize: 20, color: TEAL_TINT, fontFace: FF, bold: true, valign: "middle" });
    s.addNotes(`If you keep three things: where the Guardrail sits and what it does every tick; the on-and-off result on the same close approach${optional.onoff ? ", recorded again on 14 October" : ""}, with stopping clear of the margins as the next fix; ` +
      "and that the KPI-grade numbers come from the Orin campaign in November, with two hard targets to close before it: fail-safe correctness and the library's P0 escape rate.");
  }

  /* ---- Guard the text, write, repair, audit -------------------------------------------- */
  fs.mkdirSync(BUILD, { recursive: true });
  const dump = SLIDES.map((r) => ({ n: r.n, title: r.title, kind: r.kind, text: r.text.join("\n"), notes: r.notes }));
  const dumpFile = path.join(BUILD, "slides_text.json");
  fs.writeFileSync(dumpFile, JSON.stringify(dump, null, 1) + "\n", "utf8");
  fs.writeFileSync(path.join(BUILD, "layout.json"), JSON.stringify(LAYOUT, null, 1) + "\n", "utf8");
  const hedges = dump.flatMap((r) => hedgeHits(`${r.title}\n${r.text}\n${r.notes}`).map((h) => `slide ${r.n}: "${h}"`));
  if (hedges.length) {
    console.error("REFUSED: hedging or filler words on the slides:");
    hedges.forEach((h) => console.error("  " + h));
    process.exit(1);
  }
  claimsGuard(dumpFile);
  const missingNotes = SLIDES.filter((r) => !r.notes.trim()).map((r) => r.n);
  if (missingNotes.length) throw new Error(`slides without speaker notes: ${missingNotes.join(", ")}`);
  const unusedTop = TOP10.flatMap(([a, b]) => [a, b]).filter((id) => id && !FACTS[id]);
  if (unusedTop.length) throw new Error(`cheatsheet facts missing: ${unusedTop.join(", ")}`);

  if (fs.existsSync(OUT)) fs.rmSync(OUT);
  await pres.writeFile({ fileName: OUT });
  repairAndAudit(OUT, REPO, PY);
  if (!NO_OFFICE) {
    await embedFonts(OUT);
    await setLastModifiedBy(OUT, LAB);
  }
  const mb = fs.statSync(OUT).size / 1048576;
  if (mb > MAX_MB) {
    console.error(`REFUSED: deck is ${mb.toFixed(1)} MB (limit ${MAX_MB} MB).`);
    process.exit(1);
  }
  // The verified phrase goes with each fact: the numbers around a value in its
  // source sentence (200 checks, 300 spawns) are checked numbers too.
  const factsOut = Object.values(FACTS).map((f) => {
    const s0 = f.sources.find((x) => x.file === f.file);
    return { id: f.id, value: f.v, source: f.file || f.sources.map((x) => x.file).join(" | "), status: f.status,
      phrase: f.status === "verified" && s0 && s0.phrase ? s0.phrase : null, used: USED.has(f.id) };
  });
  fs.writeFileSync(path.join(BUILD, "facts.json"), JSON.stringify(factsOut, null, 1) + "\n", "utf8");
  console.log(`wrote ${OUT}`);
  console.log(`slides: ${page} (still: ${optional.still}, on/off: ${optional.onoff})`);
  console.log(`size  : ${mb.toFixed(2)} MB`);

  if (PACK_BUILD) {
    fs.mkdirSync(FIG_DIR, { recursive: true });
    for (const n of FIGS) fs.copyFileSync(figPng(n), path.join(FIG_DIR, `${n}.png`));
    for (const lang of ["EN", "ID"]) writeNumberTable(path.join(PACK, `CHEATSHEET-${lang}.md`), lang);
  }
  if (flag("--render") && !NO_OFFICE) renderSlides(OUT, path.join(BUILD, "render"));
  if (flag("--pdf") && !NO_OFFICE) {
    execFileSync("powershell", ["-NoProfile", "-ExecutionPolicy", "Bypass", "-File", path.join(REPO, "tools/office_to_pdf.ps1"), "-Path", OUT], { stdio: "inherit" });
  }
  // After the PDF, so the README gives this build's PDF size.
  if (PACK_BUILD) writeReadme(page, optional, WHERE);
  fs.writeFileSync(path.join(BUILD, "build_stamp.json"), JSON.stringify({
    built: new Date().toISOString().replace(/\.\d+Z$/, "Z"),
    builder: "tools/deck/build_explainer_deck.js", builder_sha256: BUILDER_SHA,
    deck: path.basename(OUT), deck_sha256: crypto.createHash("sha256").update(fs.readFileSync(OUT)).digest("hex"),
    slides: page, optional, office: !NO_OFFICE,
    facts: { total: Object.keys(FACTS).length, verified: factsOut.filter((f) => f.status === "verified").length, unverified: factsOut.filter((f) => f.status === "unverified").map((f) => f.id) },
  }, null, 1) + "\n", "utf8");
}

/* tools/check_claims.py on every slide's text and notes: a withdrawn claim stops the build. */
function claimsGuard(dumpFile) {
  const script = [
    "import json, sys",
    "sys.path.insert(0, sys.argv[1])",
    "import check_claims as cc",
    "rows = json.load(open(sys.argv[2], encoding='utf-8'))",
    "bad = []",
    "for r in rows:",
    "    for part in ('title', 'text', 'notes'):",
    "        for h in cc.scan_text(r[part], 'slide.md'):",
    "            bad.append('slide %s %s: %s [%s]' % (r['n'], part, h.text, h.claim))",
    "print('\\n'.join(bad))",
    "sys.exit(1 if bad else 0)",
  ].join("\n");
  try {
    execFileSync(PY, ["-c", script, path.join(REPO, "tools"), dumpFile], { stdio: "inherit" });
  } catch (e) {
    console.error("REFUSED: a slide states a withdrawn claim (tools/check_claims.py).");
    process.exit(1);
  }
}

/* The cheatsheets' number tables, between their markers, from the same facts. */
function writeNumberTable(file, lang) {
  if (!fs.existsSync(file)) { console.warn(`cheatsheet absent, table not written: ${file}`); return; }
  const raw = fs.readFileSync(file, "utf8");
  const eol = raw.includes("\r\n") ? "\r\n" : "\n";
  const start = "<!-- numbers:start -->", end = "<!-- numbers:end -->";
  const i = raw.indexOf(start), j = raw.indexOf(end);
  if (i < 0 || j < i) throw new Error(`${file}: number-table markers missing`);
  const head = lang === "EN" ? ["#", "Number", "What it is", "Source"] : ["#", "Angka", "Artinya", "Sumber"];
  // Dashes in proportion to the columns: pandoc sizes a wrapping pipe table by
  // them, so the "#" column does not take a quarter of the page.
  const lines = [`| ${head.join(" | ")} |`, "|--|--------|-------------------|----------|"];
  // Short source names keep the sheet on one page: the grant's pages by file
  // name, flight folders by folder and file.
  const short = (f) => (f.startsWith(GRANT) ? `${path.basename(f)} (grant)` : f.replace(/^demo\/out\//, "").replace(/^docs\/data\//, ""));
  TOP10.forEach(([a, b, en, id, fmt, idFmt], k) => {
    const value = (lang === "ID" && idFmt ? idFmt : fmt).replace("{v1}", V(a)).replace("{v2}", b ? V(b) : "");
    const srcs = [...new Set([a, b].filter(Boolean).map((x) => short(FACTS[x].file || FACTS[x].sources[0].file)))];
    lines.push(`| ${k + 1} | **${value}** | ${fill(lang === "EN" ? en : id)} | ${srcs.map((x) => "`" + x + "`").join(", ")} |`);
  });
  const out = raw.slice(0, i + start.length) + eol + lines.join(eol) + eol + raw.slice(j);
  if (out !== raw) fs.writeFileSync(file, out, "utf8");
}

/* The videos that go with the pack. They are local and large and never
 * committed (demo/out is gitignored), so they travel beside the pack; length,
 * frame size and file size are read from the files on every pack build. */
const VIDEOS = [
  [`demo/out/${DEMO_TAG}/${DEMO_TAG}_demo.mp4`,
    `City flight on the simple_flight rail (no ArduPilot): the CityLife red car, the no-fly zone and the policy indicator. Left the drone's camera with the HUD, right the chase camera. The deck's city still is its frame at ${STILL_VIDEO_T} s.`],
  [`demo/out/${DEMO_TAG}/${DEMO_TAG}_demo_share.mp4`, "The same video, re-encoded smaller to send."],
  [`demo/out/${DEMO_TAG}/${DEMO_TAG}_demo_phone.mp4`, "The same video, re-encoded for a phone."],
  ["demo/out/pas_ap_video_on_20261014/pas_ap_video_on_20261014_demo.mp4",
    "ArduPilot rail, the close approach with the Shield on: FSM Brake, then RTL home, where the run ends. Left the drone's camera with the HUD, right the chase camera."],
  ["demo/out/pas_ap_video_on_20261014/pas_ap_video_on_20261014_demo_phone.mp4", "The same video, re-encoded for a phone."],
  ["demo/out/pas_ap_video_off_20261014/pas_ap_video_off_20261014_demo.mp4",
    "The same close approach with the Shield off (the control arm): the raw command flown, banner SHIELD OFF."],
  ["demo/out/pas_ap_video_off_20261014/pas_ap_video_off_20261014_demo_phone.mp4", "The same video, re-encoded for a phone."],
  ["demo/out/pas_ap_video_compare_20261014.mp4",
    "The two close-approach runs stacked, Shield on above Shield off, both from take-off, each at its own recorded rate (`tools/make_compare_video.py`)."],
  ["demo/out/pas_ap_video_compare_20261014_phone.mp4", "The stacked video, re-encoded for a phone."],
];
const MB = (bytes) => `${(bytes / 1e6).toFixed(1)} MB`;
const CODEC = { mpeg4: "MPEG-4 Part 2", h264: "H.264" };
function probeVideo(rel) {
  const p = path.join(REPO, rel);
  if (!fs.existsSync(p)) return null;
  const out = { bytes: fs.statSync(p).size, s: null, w: null, h: null, codec: null };
  try {
    const j = JSON.parse(execFileSync("ffprobe", ["-v", "error", "-select_streams", "v:0", "-show_entries",
      "format=duration:stream=width,height,codec_name", "-of", "json", p], { encoding: "utf8" }));
    out.s = Number(j.format.duration);
    out.w = j.streams[0].width; out.h = j.streams[0].height;
    out.codec = CODEC[j.streams[0].codec_name] || j.streams[0].codec_name;
  } catch (e) { /* no ffprobe here: length, frame size and codec print as "?" */ }
  return out;
}

function writeReadme(slides, optional, where) {
  const fileSize = (p) => (fs.existsSync(p) ? MB(fs.statSync(p).size) : "absent");
  const pdf = path.join(PACK, `${DECK_NAME}.pdf`);
  const pdfNote = !fs.existsSync(pdf) ? "not built yet (`--pdf`)"
    : fs.statSync(pdf).mtimeMs + 2000 < fs.statSync(OUT).mtimeMs ? `${fileSize(pdf)}; older than the deck: rebuild with \`--pdf\``
      : fileSize(pdf);
  const qFile = path.join(PACK, "LIKELY-QUESTIONS.md");
  const nQ = fs.existsSync(qFile) ? (fs.readFileSync(qFile, "utf8").match(/^### /gm) || []).length : 0;
  const slideOf = (k) => (where[k] ? `slide ${where[k]}` : "no slide in this build");
  const figWhat = {
    explainer_flowchart_end_to_end: "The end-to-end flowchart: policy file to KPI report",
    explainer_arch_components: "The four work packages, their modules and the data between them",
    explainer_arch_shield: "One Shield tick: forecast, checks, repair, FSM",
    explainer_arch_topologies: "dev, hil and flight: what runs where",
    explainer_arch_evidence: "From a run to a KPI number: logs, manifest, signed replay, report",
  };
  const figRows = [];
  const figs = fs.existsSync(FIG_DIR) ? fs.readdirSync(FIG_DIR).filter((f) => /\.(png|jpe?g)$/i.test(f)).sort() : [];
  for (const f of figs) {
    const stem = f.replace(/\.(png|jpe?g)$/i, "");
    let what;
    if (figWhat[stem]) what = `${figWhat[stem]} (${slideOf(stem)}); a copy of \`docs/img/${f}\`.`;
    else if (f === path.basename(STILL_FULL)) what = `Frame of the city video at ${STILL_VIDEO_T} s (HUD clock ${STILL_HUD_T} s), both panels.`;
    else if (f === path.basename(STILL_FPV)) what = `Its left panel, the drone's camera with the HUD (${slideOf("still")}).`;
    else if (/^ardupilot_shield_on\./.test(f)) what = `Still of the Shield-on recording at video t = ${ONOFF_ON_T} s (HUD clock ${ONOFF_ON_HUD_T} s): FSM Brake, the detector's box over the top of the red traffic signal, its lit lamp directly below the box (${slideOf("onoff")}).`;
    else if (/^ardupilot_shield_off\./.test(f)) what = `Still of the Shield-off recording at video t = ${ONOFF_OFF_T} s (${slideOf("onoff")}).`;
    else what = "Not used by the deck.";
    figRows.push(`| \`figures/${f}\` | ${what} |`);
  }
  const vidRows = VIDEOS.map(([rel, what]) => {
    const v = probeVideo(rel);
    if (!v) return `| \`${rel}\` | - | - | - | - | Not on this machine. ${what} |`;
    const len = v.s == null ? "?" : `${Math.round(v.s)} s`;
    const frame = v.w == null ? "?" : `${v.w} x ${v.h}`;
    return `| \`${rel}\` | ${len} | ${MB(v.bytes)} | ${frame} | ${v.codec || "?"} | ${what} |`;
  });
  const L = [
    "# Share pack: the Guardrail explained end to end (14 Oct 2026)",
    "",
    "Written by `tools/deck/build_explainer_deck.js` on every pack build; do not edit it by hand.",
    `Audience: the PI and ITRI. Status date on the slides: ${STATUS_DATE}. Each result slide names its date:`,
    `the results of 9-13 October, and the close approach recorded on 14 October (${slideOf("onoff")}). The deck's`,
    "file name keeps its first date, 13Oct2026. Nothing here is KPI-grade: simulation and desktop results only.",
    "",
    "## Files in this folder",
    "",
    "| File | What it is |",
    "|---|---|",
    "| `README.md` | This file. |",
    `| \`${DECK_NAME}.pptx\` | The deck: ${slides} slides, a speaker note on every slide${NO_OFFICE ? "" : ", Poppins embedded"}; ${fileSize(OUT)}. |`,
    `| \`${DECK_NAME}.pdf\` | The same deck as PDF; ${pdfNote}. |`,
    "| `CHEATSHEET-EN.md` | One page: the mental model, ten numbers with sources, lines to say and not to say. |",
    "| `CHEATSHEET-ID.md` | The same page in Indonesian. |",
    `| \`LIKELY-QUESTIONS.md\` | ${nQ} questions the PI or ITRI may ask, with short model answers in English and Indonesian. |`,
    ...figRows,
    "| `_build/facts.json` | Every number on the slides: printed value, source file, and whether it was checked. |",
    "| `_build/slides_text.json` | Every slide's text and notes, as the wording checks read them. |",
    "| `_build/layout.json` | Every text box, for the overflow test. |",
    "| `_build/build_stamp.json` | Build time, builder and deck SHA-256, fact counts. |",
    "| `_build/render/slide_NN.png` | Every slide as a 1600 x 900 PNG (`--render`). |",
    "",
    "## Videos (local, not in this folder, never committed)",
    "",
    "`demo/out/` is gitignored and these files are too large for the repository: send them beside the",
    "pack. Length, frame size and file size are read from the files on every pack build (1 MB = 10^6",
    "bytes). The ArduPilot runs are Project AirSim + ArduPilot SITL over pymavlink: none of the grant's",
    "topologies, not KPI-grade. On that rail the detector boxed a red traffic signal, not the car, so",
    "those videos show the Shield, not tracking.",
    "",
    "To send, use the H.264 files, which play in a browser and on a phone: the city `_share` (1920 x 540),",
    "the stacked compare video and every `_phone` file. The `_demo.mp4` masters are MPEG-4 Part 2 at",
    "2560 x 720, which browsers do not play; they are for a desktop player and for editing. Each `_phone`",
    "file is 1280 x 360, so each panel is 640 x 360 and the HUD text is small; for the two ArduPilot",
    "runs on a larger screen, send the stacked compare video.",
    "",
    "| Video | Length | Size | Frame | Codec | What it shows |",
    "|---|---|---|---|---|---|",
    ...vidRows,
    "",
    "How they are made: a run recorded with `--save-view` becomes `<tag>_demo.mp4` with",
    "`python tools/make_demo_video.py --tag <tag> --height 720`. The city flight is `demo/follow_vlm.py`",
    "with `--save-view`. The ArduPilot pair is",
    "`scripts/run_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag <tag>`",
    "with `-NodeArgs \"--want-width\",\"0.5\"` for the Shield-on run and",
    "`-NodeArgs \"--want-width\",\"0.5\",\"--shield\",\"off\"` for the control; step 4 of \"How to run it\" in",
    "`docs/DESIGN-projectairsim-ardupilot.md` has the full lines.",
    "`python tools/make_compare_video.py --a pas_ap_video_on_20261014 --b pas_ap_video_off_20261014 --out demo/out/pas_ap_video_compare_20261014.mp4`",
    "stacks the two runs; it refuses an existing file unless `--force`. The `_share` and `_phone` files are",
    "smaller H.264 re-encodes made with ffmpeg; no script in the repository writes them, and their",
    "settings are not recorded.",
    "",
    "## Rebuild",
    "",
    "```",
    "C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_explainer_figures.py --png",
    "node tools/deck/build_explainer_deck.js --render --pdf",
    "C:/Users/natha/.conda/envs/vla-real/python.exe tests/test_explainer_deck.py",
    "```",
    "",
    "The first line draws the five figures into `docs/img/`. The second builds the deck, embeds the fonts",
    "through PowerPoint, renders every slide (`--render`), writes the PDF (`--pdf`), copies the figures",
    "here, writes the ten-number tables of both cheatsheets and writes this README. The third builds a",
    "test copy without PowerPoint and checks it.",
    "",
    "Optional slides take images; a missing image drops its slide and nothing else: the city-video still",
    `(\`--still PATH\`; by default \`figures/${path.basename(STILL_FPV)}\`, extracted with ffmpeg from the`,
    `video at ${STILL_VIDEO_T} s when the video is on disk) and the ArduPilot Shield-on/off stills`,
    "(`--onoff-on PATH --onoff-off PATH`; by default `figures/ardupilot_shield_on.png` and `_off.png`,",
    `cut with ffmpeg from each run's \`_demo.mp4\` at ${ONOFF_ON_T} s and ${ONOFF_OFF_T} s; the commands are in \`tools/deck/README.md\`).`,
    "",
    "Every number on a slide is checked against its source file at build time; a mismatch stops the",
    "build. A fact whose source is absent (flight output under `demo/out` on another machine) stops the",
    "pack build unless `--allow-unverified` is given. Every slide's text and notes pass",
    "`tools/check_claims.py` and a hedge-word scan. `--check-facts-only` checks the facts and exits;",
    "`--out DIR --no-office` writes a test build elsewhere without PowerPoint.",
    "",
  ];
  fs.writeFileSync(path.join(PACK, "README.md"), L.join("\n"), "utf8");
}

/* Every slide to PNG through PowerPoint, for review by eye. */
function renderSlides(pptx, dir) {
  fs.mkdirSync(dir, { recursive: true });
  for (const f of fs.readdirSync(dir)) if (/^slide_\d+\.png$/.test(f)) fs.rmSync(path.join(dir, f));
  const ps = [
    "$ErrorActionPreference = 'Stop'",
    "$before = @(Get-Process POWERPNT -ErrorAction SilentlyContinue)",
    "$app = New-Object -ComObject PowerPoint.Application",
    "$pres = $null",
    "try { $pres = $app.Presentations.Open($env:R_IN, $true, $false, $false);" +
      " for ($i = 1; $i -le $pres.Slides.Count; $i++) { $pres.Slides.Item($i).Export((Join-Path $env:R_OUT ('slide_{0:D2}.png' -f $i)), 'PNG', 1600, 900) } }" +
      " finally { if ($pres) { $pres.Close() }; if ($before.Count -eq 0) { try { $app.Quit() } catch {} };" +
      " [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app) }",
  ].join("; ");
  execFileSync("powershell", ["-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps],
    { stdio: "inherit", env: { ...process.env, R_IN: pptx, R_OUT: dir } });
  const n = fs.readdirSync(dir).filter((f) => /^slide_\d+\.png$/.test(f)).length;
  console.log(`render: ${n} slides to ${dir}`);
}

/* As build_progress_oct_deck.js: PowerPoint's save records the Windows user. */
async function setLastModifiedBy(pptx, who) {
  const JSZip = require("jszip");
  const zip = await JSZip.loadAsync(fs.readFileSync(pptx));
  const xml = await zip.file("docProps/core.xml").async("string");
  const esc = who.replace(/&/g, "&amp;").replace(/</g, "&lt;");
  let out = xml.replace(/<cp:lastModifiedBy>[^<]*<\/cp:lastModifiedBy>/, `<cp:lastModifiedBy>${esc}</cp:lastModifiedBy>`)
    .replace(/<dc:creator>[^<]*<\/dc:creator>/, `<dc:creator>${esc}</dc:creator>`);
  if (!/<cp:lastModifiedBy>/.test(out)) out = out.replace("</cp:coreProperties>", `<cp:lastModifiedBy>${esc}</cp:lastModifiedBy></cp:coreProperties>`);
  zip.file("docProps/core.xml", out, { createFolders: false });
  fs.writeFileSync(pptx, await zip.generateAsync({ type: "nodebuffer", compression: "DEFLATE", compressionOptions: { level: 6 } }));
  execFileSync(PY, [path.join(REPO, "tools/audit_pptx.py"), pptx], { stdio: "inherit" });
}

/* As build_progress_oct_deck.js: Poppins is a per-user font on this PC, so
 * PowerPoint re-saves the audited deck with its fonts embedded. */
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
  if (!fs.existsSync(tmp) || fs.statSync(tmp).mtimeMs < started - 2000) throw new Error(`font embedding wrote no new file at ${tmp}`);
  const zip = await JSZip.loadAsync(fs.readFileSync(tmp));
  const presXml = await zip.file("ppt/presentation.xml").async("string");
  const lst = /<p:embeddedFontLst>([\s\S]*?)<\/p:embeddedFontLst>/.exec(presXml);
  const faces = lst ? [...lst[1].matchAll(/typeface="([^"]+)"/g)].map((m) => m[1]) : [];
  if (!faces.includes(T.FF)) throw new Error(`fonts not embedded (faces: ${faces.join(", ") || "none"})`);
  fs.copyFileSync(tmp, pptx);
  fs.unlinkSync(tmp);
  execFileSync(PY, [path.join(REPO, "tools/audit_pptx.py"), pptx], { stdio: "inherit" });
  console.log(`fonts : embedded ${faces.join(", ")}`);
}

main().catch((e) => { console.error(e); process.exit(1); });
