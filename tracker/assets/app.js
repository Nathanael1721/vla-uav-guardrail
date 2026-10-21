// Grant tracker: one page, hash routes, content from /api/content, board state from /api/state.
(() => {
  "use strict";

  const URL_LANG = new URLSearchParams(location.search).get("lang");
  const S = {
    lang: (URL_LANG && ["en", "id", "zh"].includes(URL_LANG)) ? URL_LANG : (safeGet("lang") || "en"),
    content: null,
    board: { cards: {}, updated: null },
    filters: { q: "", area: "", verdict: "", prio: "", gate: false },
    flowTab: "built",
    flowSel: null,
    hudSel: null,
    updTab: "log",
    updF: { q: "", kind: "", retracted: false, media: false },
    updOpen: {},
    dash: null, dashErr: false, dashEtag: null, live: null, contentEtag: null,
    todoF: { owner: "", status: "notdone", q: "" },
  };
  const CATS = ["retracted", "fixed", "added", "changed", "flown", "found", "limits", "confirmed", "tests", "docs", "share"];
  const CATCOL = { retracted: "var(--missing)", fixed: "var(--done)", added: "var(--accent)", changed: "var(--pi)", flown: "#0b7285",
    found: "var(--partial)", limits: "var(--partial)", confirmed: "var(--todo)", tests: "var(--done)", docs: "var(--todo)", share: "var(--drift)" };
  const LOCALE = { en: "en-GB", id: "id-ID", zh: "zh-TW" };
  const FINAL = new Date("2026-11-30T23:59:59+08:00");
  const AREAS = ["WP1", "WP2", "WP3", "WP4", "ARCH", "CROSS", "CLAIMS"];
  const VERDICTS = ["DONE", "PARTIAL", "MISSING", "DRIFT"];
  const STATES = ["pi", "todo", "doing", "done", "waived"];
  const PRIOS = ["gate", "high", "medium", "low"];
  const VCOL = { DONE: "var(--done)", PARTIAL: "var(--partial)", MISSING: "var(--missing)", DRIFT: "var(--drift)" };
  const SCOL = { pi: "var(--pi)", todo: "var(--todo)", doing: "var(--doing)", done: "var(--done)", waived: "var(--waived)" };

  const MEDIA = {
    hil_nfz: ["img", "mideval/hil_nfz_off_on.png"], simultaneity: ["img", "fig2_simultaneity.png"],
    ft_comparison: ["img", "ft_comparison.png"], ft_loss: ["img", "ft_loss_curve.png"],
    hud_nfz2: ["img", "policy_hud_nfz2.jpg"], before_fpv: ["img", "progress_0930_before_fpv.jpg"],
    after_fpv: ["img", "progress_0930_after_fpv.jpg"], est_on_car: ["img", "progress_0930_estimate_on_car.png"],
    within30: ["img", "progress_0930_within_30m.png"], signals: ["img", "citylife_signals_junction_4100_4100.png"],
    retarget: ["img", "mideval/retarget_car_person.jpg"], flow_tracking: ["img", "flowchart_tracking.svg"],
    flow_system: ["img", "flowchart_system.svg"], g1_airborne: ["img", "g1_pas_ardupilot_airborne.png"],
    v_nfz_hud: ["video", "citylife_redcar_nfz_policy_hud.mp4"], v_identity: ["video", "citylife_redcar_identity.mp4"],
    v_before_after: ["video", "progress_0930_before_after.mp4"], v_demo_nfz: ["video", "demo_nfz.mp4"],
    v_retarget: ["video", "retarget_smooth.mp4"], v_demo_follow: ["video", "demo_follow.mp4"],
    v_ped: ["video", "citylife_ped_final.mp4"],
    fig_e2e: ["img", "explainer_flowchart_end_to_end.png"], fig_components: ["img", "explainer_arch_components.png"],
    fig_topologies: ["img", "explainer_arch_topologies.png"], fig_shield: ["img", "explainer_arch_shield.png"],
    fig_evidence: ["img", "explainer_arch_evidence.png"],
    v_close_compare: ["run", "pas_ap_video_compare_20261014.mp4"],
    v_close_on: ["run", "pas_ap_video_on_20261014/pas_ap_video_on_20261014_demo_phone.mp4"],
    v_close_off: ["run", "pas_ap_video_off_20261014/pas_ap_video_off_20261014_demo_phone.mp4"],
    v_city_nfz_1013: ["run", "citylife_redcar_nfz_20261013/citylife_redcar_nfz_20261013_demo_share.mp4"],
    v_ovf_compare: ["run", "pas_ap_ovf_compare_20261021_phone.mp4"],
    v_city_motion_compare: ["run", "citylife_motion_compare_20261019_phone.mp4"],
  };

  // ------------------------------------------------------------ helpers
  function safeGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
  function safeSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } }
  const $ = (s, el = document) => el.querySelector(s);
  const $$ = (s, el = document) => Array.from(el.querySelectorAll(s));
  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const t = (k) => (UI[S.lang] && UI[S.lang][k]) || UI.en[k] || k;
  const L = (o) => { if (!o) return {}; const i = o.i18n || o; return i[S.lang] || i.en || {}; };
  const Lstr = (o) => { if (!o) return ""; const i = o.i18n || o; const v = i[S.lang] ?? i.en; return typeof v === "string" ? v : ""; };
  const list = (a, cls = "") => (a && a.length ? `<ul class="${cls}">${a.map((x) => `<li>${esc(x)}</li>`).join("")}</ul>` : `<p class="muted">${t("empty")}</p>`);
  const pill = (v) => `<span class="pill v-${v}" title="${esc(t("verdict_help_" + v))}">${esc(t("verdict_" + v))}</span>`;
  const pct = (x) => (x == null ? t("empty") : `${(x * 100).toFixed(1)} %`);
  // A MEDIA entry is [folder, file]: img, video and share are under docs/, run is demo/out (the
  // recorded runs, served read-only). Whether it shows as a picture or a video follows the file.
  const isVideo = (f) => /\.(mp4|webm)$/i.test(f);
  function mediaOf(id) {
    if (id && id.includes("/")) { const kind = isVideo(id) ? "video" : "img"; return { id, kind, src: `/media/${id}` }; }
    const m = MEDIA[id]; return m ? { id, kind: isVideo(m[1]) ? "video" : "img", src: `/media/${m[0]}/${m[1]}` } : null;
  }
  // Figures drawn from code (content_flow.json "figures"): a picture on white with its caption;
  // a click opens it full size.
  function figureCard(f) {
    const m = mediaOf(f.media); const x = L(f); if (!m) return "";
    return `<div class="panel fig"><div class="media" data-media="${esc(m.id)}" data-cap="${esc(x.title)}"><img src="${m.src}" alt="${esc(x.title)}" loading="lazy"></div>
      <div class="row"><h3 style="margin:0">${esc(x.title)}</h3><span class="spacer"></span>${f.date ? `<span class="chip">${esc(f.date)}</span>` : ""}</div>
      <p class="small">${fmt(x.caption)}</p>${f.source ? `<div class="small muted"><code>${esc(f.source)}</code></div>` : ""}</div>`;
  }
  function figures() { const f = S.content.flow || {}; return Array.isArray(f.figures) ? f.figures : []; }
  const fmt = (s) => esc(s).replace(/`([^`]+)`/g, "<code>$1</code>");
  const tf = (k, o) => t(k).replace(/\{(\w+)\}/g, (_, x) => (o && o[x] != null ? o[x] : ""));
  const fdate = (d) => { try { return new Date(d + "T00:00:00").toLocaleDateString(LOCALE[S.lang], { day: "numeric", month: "short", year: "numeric" }); } catch (e) { return d; } };
  const ftime = (iso) => { try { return new Date(iso).toLocaleString(LOCALE[S.lang], { day: "numeric", month: "short", hour: "2-digit", minute: "2-digit" }); } catch (e) { return iso; } };
  function updates() { const u = S.content && S.content.updates; return (u && Array.isArray(u.entries)) ? u.entries : []; }
  function verLabel(e) {
    return e.kind === "release" ? tf("upd_release", { v: e.version }) : tf("upd_norelease", { v: e.base_version || (e.version || "").split("+")[0] });
  }
  function toast(msg, bad) {
    const el = $("#toast"); el.textContent = msg; el.className = "toast show" + (bad ? " bad" : "");
    clearTimeout(toast._t); toast._t = setTimeout(() => (el.className = "toast"), 2200);
  }
  function cards() { return (S.content && S.content.cards || []).filter((c) => !c.load_error); }
  function stack(counts, keys, colors) {
    const total = keys.reduce((s, k) => s + (counts[k] || 0), 0) || 1;
    return `<div class="stack">${keys.map((k) => counts[k] ? `<span style="width:${(100 * counts[k]) / total}%;background:${colors[k]}" title="${counts[k]}"></span>` : "").join("")}</div>`;
  }
  function count(arr, f) { const o = {}; arr.forEach((x) => { const k = f(x); o[k] = (o[k] || 0) + 1; }); return o; }

  // ------------------------------------------------------------ board state
  function defaultState(c) { return c.status === "DONE" ? "done" : c.needs_pi ? "pi" : "todo"; }
  function cstate(c) {
    const s = S.board.cards[c.id] || {};
    return { state: s.state || defaultState(c), note: s.note || "", history: s.history || [], note_t: s.note_t || null, note_log: s.note_log || [] };
  }
  function setCard(c, patch) {
    const cur = cstate(c); const next = Object.assign({}, cur, patch);
    if (patch.state && patch.state !== cur.state) next.history = cur.history.concat([{ t: new Date().toISOString(), from: cur.state, to: patch.state }]);
    S.board.cards[c.id] = next; save();
  }
  let saveTimer = null;
  function save(immediate) {
    clearTimeout(saveTimer);
    saveTimer = setTimeout(async () => {
      S.board.updated = new Date().toISOString();
      safeSet("board", JSON.stringify(S.board));
      try {
        const r = await fetch("/api/state", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(S.board) });
        if (!r.ok) throw new Error(r.status);
        toast(t("saved")); renderBadge();
      } catch (e) { toast(t("save_failed"), true); }
    }, immediate ? 0 : 400);
  }

  // ------------------------------------------------------------ routing
  // The Dashboard is the first page and the default route (2026-10-21): it is the progress monitor.
  const PAGES = { dashboard: renderDashboard, overview: renderOverview, flow: renderFlow, grant: renderGrant, board: renderBoard, evidence: renderEvidence, glossary: renderGlossary, updates: renderUpdates };
  function route() { const h = (location.hash || "#dashboard").slice(1).split("?")[0]; return PAGES[h] ? h : "dashboard"; }
  function renderNav() {
    $("#nav").innerHTML = Object.keys(PAGES).map((k) => `<a href="#${k}" class="${route() === k ? "on" : ""}">${esc(t("nav_" + k))}</a>`).join("");
    $$("#lang button").forEach((b) => b.classList.toggle("on", b.dataset.lang === S.lang));
    $("#app-title").textContent = t("app_title");
    $("#reload").title = t("reload"); $("#reload").setAttribute("aria-label", t("reload"));
    renderBadge();
    document.title = t("app_title");
    document.documentElement.lang = S.lang === "zh" ? "zh-Hant" : S.lang;
  }
  function render() {
    renderNav();
    if (!S.content) { $("#main").innerHTML = `<p class="muted pad">${esc(t("loading"))}</p>`; return; }
    S.page = route();
    $("#main").innerHTML = PAGES[S.page]();
    bind(S.page);
    if (!/[?&]v=/.test(location.hash)) window.scrollTo(0, 0);
  }

  // ------------------------------------------------------------ icons
  const ICON = {
    driver: '<svg viewBox="0 0 48 48"><circle cx="24" cy="24" r="20" fill="none" stroke="var(--accent)" stroke-width="4"/><circle cx="24" cy="24" r="5" fill="var(--accent)"/><path d="M6 22h13M29 22h13M24 29v15" stroke="var(--accent)" stroke-width="4"/></svg>',
    rulebook: '<svg viewBox="0 0 48 48"><path d="M8 8h14a4 4 0 014 4v28a3 3 0 00-3-3H8z M40 8H26a4 4 0 00-4 4v28a3 3 0 013-3h15z" fill="var(--accent-soft)" stroke="var(--accent)" stroke-width="3"/><path d="M12 16h6M12 22h6M30 16h6M30 22h6" stroke="var(--accent)" stroke-width="2.5"/></svg>',
    instructor: '<svg viewBox="0 0 48 48"><path d="M24 4l17 6v12c0 11-7 18-17 22C14 40 7 33 7 22V10z" fill="var(--accent)"/><path d="M16 24l6 6 11-12" stroke="#fff" stroke-width="4" fill="none" stroke-linecap="round"/></svg>',
    car: '<svg viewBox="0 0 48 48"><circle cx="10" cy="12" r="6" fill="none" stroke="var(--accent)" stroke-width="3"/><circle cx="38" cy="12" r="6" fill="none" stroke="var(--accent)" stroke-width="3"/><circle cx="10" cy="36" r="6" fill="none" stroke="var(--accent)" stroke-width="3"/><circle cx="38" cy="36" r="6" fill="none" stroke="var(--accent)" stroke-width="3"/><rect x="17" y="17" width="14" height="14" rx="3" fill="var(--accent)"/><path d="M14 16l5 5M34 16l-5 5M14 32l5-5M34 32l-5-5" stroke="var(--accent)" stroke-width="3"/></svg>',
    blackbox: '<svg viewBox="0 0 48 48"><rect x="6" y="12" width="36" height="28" rx="4" fill="var(--ink)"/><path d="M6 20h36" stroke="var(--panel)" stroke-width="2"/><circle cx="36" cy="16" r="2" fill="var(--missing)"/><path d="M12 28h18M12 33h12" stroke="var(--panel)" stroke-width="2.5"/></svg>',
  };

  // ------------------------------------------------------------ overview
  function renderOverview() {
    const ov = S.content.overview || {};
    const p = L(ov.project);
    const cs = cards();
    const vc = count(cs, (c) => c.status);
    const sc = count(cs, (c) => cstate(c).state);
    const days = Math.max(0, Math.ceil((FINAL - new Date()) / 86400000));
    const areaStats = AREAS.map((a) => ({ a, n: cs.filter((c) => c.area === a), vc: count(cs.filter((c) => c.area === a), (c) => c.status) }));
    const wps = ov.wps || [];
    return `
      <section class="hero">
        <div class="panel">
          <h1>${esc(p.one_liner || t("app_title"))}</h1>
          <div class="grid g2" style="margin-top:12px">
            <div><h3>${esc(t("problem"))}</h3><p>${esc(p.problem)}</p></div>
            <div><h3>${esc(t("solution"))}</h3><p>${esc(p.solution)}</p></div>
          </div>
          <h3>${esc(t("contract"))}</h3>${list(p.contract_facts)}
        </div>
        <div class="panel grid" style="align-content:start">
          <div class="stat"><span class="big-number">${days}</span><span class="muted">${esc(t("days_left"))} · ${esc(t("final_gate"))}</span></div>
          <div><b>${cs.length}</b> <span class="muted">${esc(t("cards"))}: ${cs.length - cs.filter((c) => c.area === "CLAIMS").length} ${esc(t("grant_items"))} + ${cs.filter((c) => c.area === "CLAIMS").length} ${esc(t("area_CLAIMS"))}</span></div>
          <div><div class="small muted">${esc(t("by_verdict"))}</div>${stack(vc, VERDICTS, VCOL)}
            <div class="legend">${VERDICTS.map((v) => `<span title="${esc(t("verdict_help_" + v))}"><i style="background:${VCOL[v]}"></i>${esc(t("verdict_" + v))} ${vc[v] || 0}</span>`).join("")}</div></div>
          <div><div class="small muted">${esc(t("by_state"))}</div>${stack(sc, STATES, SCOL)}
            <div class="legend">${STATES.map((s) => `<span><i style="background:${SCOL[s]}"></i>${esc(t("state_" + s))} ${sc[s] || 0}</span>`).join("")}</div></div>
          <a class="btn primary" href="#board" style="text-align:center;text-decoration:none">${esc(t("nav_board"))} →</a>
          ${latestPanel()}
        </div>
      </section>

      ${figures().filter((f) => f.overview).map((f) => `<h2>${esc(t("fig_overview"))}</h2>${figureCard(f)}`).join("")}

      <h2>${esc(t("mental_model"))}</h2>
      <div class="grid g3">${(ov.mental_model || []).map((m) => { const x = L(m); return `
        <div class="panel role">${ICON[m.role_key] || ""}<div>
          <h3>${esc(x.analogy)}</h3><p><b>${esc(t("in_project"))}:</b> ${esc(x.in_project)}</p>
          <p class="small muted">${esc(x.detail)}</p>${m.file ? `<code>${esc(m.file)}</code>` : ""}</div></div>`; }).join("")}</div>

      <h2>${esc(t("work_packages"))}</h2>
      <div class="grid g2">${areaStats.map(({ a, n, vc }) => {
        const wp = wps.find((w) => w.key === a); const x = L(wp);
        return `<div class="panel wp"><h3><span>${esc(t("area_" + a))}</span><span class="muted small">${n.length} ${esc(t("cards"))}</span></h3>
          ${x.plain ? `<p>${esc(x.plain)}</p>` : ""}
          ${stack(vc, VERDICTS, VCOL)}
          <div class="legend">${VERDICTS.filter((v) => vc[v]).map((v) => `<span><i style="background:${VCOL[v]}"></i>${esc(t("verdict_" + v))} ${vc[v]}</span>`).join("")}</div>
          ${x.status ? `<p class="small" style="margin-top:8px"><b>${esc(t("our_status"))}:</b> ${esc(x.status)}</p>` : ""}
          <a href="#board" data-area="${a}" class="go-area small">${esc(t("open_board"))}</a></div>`; }).join("")}</div>

      <h2>${esc(t("gates"))}</h2>
      <div class="grid g2">${(ov.gates || []).map((g) => `
        <div class="panel"><h3>${esc(t(g.key === "final" ? "final" : "midterm"))}</h3>
          <ul class="gate-list">${(g.items || []).map((it) => { const x = L(it); return `<li>${pill(it.status)}<div><b>${esc(x.name)}</b><div class="small muted">${esc(x.note)}</div></div></li>`; }).join("")}</ul></div>`).join("")}</div>

      <div class="grid g2" style="margin-top:14px">
        <div class="panel callout warn"><h3>${esc(t("honesty"))}</h3>${list(p.honesty)}</div>
        <div class="panel callout"><h3>${esc(t("decisions"))}</h3><ol>${(ov.decisions || []).map((d) => { const x = L(d); return `<li><b>${esc(x.question)}</b><div class="small muted">${esc(x.why)}</div></li>`; }).join("")}</ol></div>
      </div>`;
  }

  // ------------------------------------------------------------ dashboard
  // Every number here comes from tracker/data/dashboard.json, which tools/build_dashboard.py derives
  // from the run folders, docs/data, the cards, the update log and the CHANGELOG, each with its source
  // and date. A missing value prints "not measured", never 0 (silence reads as success). The live
  // panel polls /api/live every 5 s and the data every 60 s, only while this page is visible.
  const PILOTS = ["vla", "stub", "detector", "none", "unknown", "missing"];
  const PCOL = { vla: "var(--done)", stub: "var(--pi)", detector: "var(--partial)", none: "var(--todo)", unknown: "var(--drift)", missing: "var(--line)" };
  const TCOL = { open: "var(--todo)", doing: "var(--doing)", blocked: "var(--missing)", done: "var(--done)" };
  const TSTATES = ["open", "doing", "blocked", "done"];
  const OWNERS = ["user", "agent", "pi"];
  async function loadDash(force, quiet) {
    try {
      const r = await fetch("/api/dashboard", { headers: (!force && S.dashEtag) ? { "If-None-Match": S.dashEtag } : {} });
      if (r.status === 304) return;
      if (!r.ok) throw new Error(r.status);
      S.dashEtag = r.headers.get("ETag"); S.dash = await r.json(); S.dashErr = false;
    } catch (e) { S.dashErr = true; }
    if (!quiet && S.content && route() === "dashboard") { const y = window.scrollY; render(); window.scrollTo(0, y); }
  }
  async function loadLive(quiet) {
    try { const r = await fetch("/api/live"); if (!r.ok) throw new Error(r.status); S.live = await r.json(); }
    catch (e) { S.live = { error: "unreachable", jobs: [] }; }
    if (!quiet) { const el = $("#dash-live"); if (el) el.innerHTML = liveHTML(); }
  }
  let dashTimers = [];
  function bindDashboard() {
    dashTimers.forEach(clearInterval);
    const on = () => route() === "dashboard" && document.visibilityState === "visible";
    dashTimers = [setInterval(() => { if (on()) loadLive(); }, 5000), setInterval(() => { if (on()) loadDash(false); }, 60000)];
    const upd = (k, v) => { S.todoF[k] = v; const el = $("#dash-todo"); if (el) { el.innerHTML = todoHTML(); bindTodo(); } };
    const bindTodo = () => {
      const o = $("#t-owner"); if (o) o.onchange = (e) => upd("owner", e.target.value);
      const s = $("#t-status"); if (s) s.onchange = (e) => upd("status", e.target.value);
      const q = $("#t-q"); if (q) q.oninput = (e) => { S.todoF.q = e.target.value; clearTimeout(bindTodo._t); bindTodo._t = setTimeout(() => { upd("q", S.todoF.q); const el = $("#t-q"); el.focus(); el.setSelectionRange(el.value.length, el.value.length); }, 250); };
    };
    bindTodo();
    $$("#main [data-play]").forEach((el) => (el.onclick = () => openMedia(el.dataset.play, el.dataset.cap)));
  }
  const dnum = (v, d = 2) => (v == null ? t("not_measured") : (+v).toFixed(d));
  function fmtKpi(v, unit) {
    if (v == null) return `<span class="nm">${esc(t("not_measured"))}</span>`;
    if (unit === "ratio") { const p = 100 * v; return esc((p === 0 ? "0" : p < 0.1 ? p.toPrecision(2) : p.toFixed(1)) + " %"); }
    if (unit === "ms") return esc((v < 10 ? v.toFixed(2) : v.toFixed(1)) + " ms");
    if (unit === "s") return esc(v.toFixed(2) + " s");
    if (unit === "mps") return esc(v.toFixed(2) + " m/s");
    return esc(String(Math.round(v * 100) / 100));
  }
  function fmtTarget(k) { if (!k.target) return esc(t("tracked")); return esc(`${k.target.op === "<=" ? "≤" : "≥"} `) + fmtKpi(k.target.value, k.unit); }
  const srcCode = (s) => (s ? `<code class="src" title="${esc(s)}">${esc(s.replace(/^docs\/data\//, "").replace(/^demo\/out\//, ""))}</code>` : "");
  function dashHeader(d) {
    const days = Math.max(0, Math.ceil((FINAL - new Date()) / 86400000));
    const m = d.meta || {}; const sv = d.server || {};
    const badges = [];
    // A failed /api/dashboard read keeps the last data on screen; its server block is then as old as
    // the data, so it must not decide the badges, and "Data current" is never shown in that state
    // (review of 8035d0a, P3: a dead server read as current data, with a green badge).
    if (S.dashErr) badges.push(`<span class="badge bad">${esc(tf("dash_offline", { t: m.generated ? ftime(m.generated) : "?" }))}</span>`);
    else {
      if (sv.stale) badges.push(`<span class="badge bad" title="${esc((sv.stale_reasons || []).join("\n"))}">${esc(t("dash_stale"))}</span>`);
      if (sv.rebuild === "started" || sv.rebuild === "running") badges.push(`<span class="badge">${esc(t("dash_rebuilding"))}</span>`);
      if (sv.rebuild === "deferred_flight") badges.push(`<span class="badge">${esc(t("dash_rebuild_deferred"))}</span>`);
    }
    if (m.content_stale) badges.push(`<span class="badge warn" title="${esc(tf("dash_cards_stale_help", { cl: m.changelog_newest, cr: m.cards_newest_review }))}">${esc(t("dash_cards_stale"))}</span>`);
    return `<section class="panel dash-head">
      <div class="stat"><span class="big-number">${days}</span><span class="muted small">${esc(t("days_left"))}</span></div>
      <div class="dh-meta small"><div><b>${esc(t("project"))}</b> ${esc(m.version || "?")} · git <code>${esc(m.git_head || "?")}</code>${m.git_dirty ? " (dirty)" : ""}</div>
        <div class="muted">${esc(t("dash_generated"))} ${esc(m.generated ? ftime(m.generated) : "?")} · ${esc(t("dash_mode_" + (m.mode || "quick")))}</div>
        <div class="muted">${esc(tf("dash_sources", { cl: m.changelog_newest || "?", cr: m.cards_newest_review || "?" }))}</div></div>
      <div class="dh-badges">${badges.join("") || (S.dashErr ? "" : `<span class="badge ok">${esc(t("dash_fresh"))}</span>`)}</div></section>`;
  }
  function liveHTML() {
    const L0 = S.live || { jobs: [] };
    const jobs = L0.jobs || [];
    const g = L0.gpu;
    const row = (j) => `<tr class="ls-${esc(j.state)}"><td><span class="chip">${esc(t("live_kind_" + j.kind))}</span>${j.phase ? ` <span class="chip">${esc(t("live_phase_" + j.phase))}</span>` : ""}</td><td><b>${esc(j.name)}</b>${j.run_tag ? `<div class="small muted"><code>demo/out/${esc(j.run_tag)}</code></div>` : ""}${j.detail ? `<div class="small muted">${esc(j.detail)}</div>` : ""}</td>
      <td class="small">${esc(ftime(j.started))}</td><td>${j.progress == null ? "" : `<div class="bar" style="width:90px"><span style="width:${100 * j.progress}%"></span></div>`}</td>
      <td><span class="lstate ${esc(j.state)}">${esc(t("live_state_" + j.state))}</span><div class="small muted">${esc(tf("live_beat", { s: Math.round(j.age_s) }))}</div></td></tr>`;
    const gpu = g && g.gpus && g.gpus.length ? g.gpus.map((x) => `<div class="gpu"><b>${esc(x.name)}</b> · ${esc(t("live_util"))} ${x.util_pct == null ? "?" : x.util_pct + " %"} · ${esc(t("live_vram"))} ${x.mem_used_mib == null ? "?" : (x.mem_used_mib / 1024).toFixed(1)} / ${x.mem_total_mib == null ? "?" : (x.mem_total_mib / 1024).toFixed(1)} GB
        <div class="bar" style="margin-top:4px"><span style="width:${x.mem_total_mib ? (100 * x.mem_used_mib) / x.mem_total_mib : 0}%"></span></div></div>`).join("")
        + (g.apps && g.apps.length ? `<div class="small muted">${esc(t("live_apps"))}: ${g.apps.map((a) => `${esc(a.name)}${a.mem_mib != null ? ` (${(a.mem_mib / 1024).toFixed(1)} GB)` : ""}`).join(", ")}${g.other_apps ? ` + ${g.other_apps}` : ""}</div>` : "")
        + (g.reduced_for_flight ? `<div class="small muted">${esc(t("live_gpu_reduced"))}</div>` : "")
      // A failed /api/live read knows nothing about the GPU either: no "no GPU" line then (P3).
      : (L0.error ? "" : `<div class="small muted">${esc(t("live_gpu_none"))}</div>`);
    // A failed read says only that it failed: it must not also assert "Nothing is running", which
    // would hide a flight and could never show the overlap banner (review of 8035d0a, P3).
    const none = L0.error ? "" : `<p class="muted small">${esc(t(L0.dir_ok === false ? "live_dir_missing" : "live_none"))}</p>`;
    return `<h2 class="ptitle">${esc(t("live_title"))}</h2>
      ${L0.overlap ? `<div class="callout bad-banner">${esc(t("live_overlap"))}</div>` : ""}
      ${L0.error ? `<div class="callout warn"><p class="small">${esc(t("live_error"))}</p></div>` : ""}
      ${jobs.length ? `<div class="table-wrap"><table class="live"><thead><tr><th>${esc(t("col_kind"))}</th><th>${esc(t("col_name"))}</th><th>${esc(t("col_started"))}</th><th>${esc(t("col_progress"))}</th><th>${esc(t("col_state"))}</th></tr></thead><tbody>${jobs.map(row).join("")}</tbody></table></div>`
        : none}
      ${(L0.done || []).length ? `<div class="small muted">${esc(t("live_recent"))}: ${(L0.done || []).map((j) => `${esc(j.name)} (${esc(t("live_kind_" + j.kind))}, ${esc(j.status || "")}${j.ended ? ", " + esc(ftime(j.ended)) : ""})`).join(" · ")}</div>` : ""}
      ${gpu ? `<h3 style="margin-top:12px">${esc(t("live_gpu"))}</h3>${gpu}` : ""}
      <p class="small muted" style="margin-top:8px">${esc(t("live_help"))}</p>`;
  }
  const todoTitle = (i) => (i.i18n && i.i18n[S.lang] && i.i18n[S.lang].title) || (i.i18n && i.i18n.en && i.i18n.en.title) || i.id;
  const tpill = (s) => `<span class="tpill" style="background:${TCOL[s] || "var(--todo)"}">${esc(t("tstatus_" + s))}</span>`;
  function vlaPanel(d) {
    const v = d.vla || {}; const c = v.counts || {};
    const items = ((d.todo || {}).items || []).filter((i) => i.milestone === "vla");
    const last = v.last_vla_flight;
    return `<section class="panel vla"><h2 class="ptitle">${esc(t("vla_title"))}</h2>
      <p class="small">${esc(t("vla_goal"))}</p>
      <div class="vla-last ${last ? "ok" : "no"}"><div class="small muted">${esc(t("vla_last"))}</div>
        <b>${last ? `${esc(last.tag)} · ${esc(fdate(last.date))} · <code>${esc(last.id)}</code>` : esc(t("vla_none_yet"))}</b></div>
      <div class="small muted" style="margin-top:10px">${esc(tf("vla_counts", { n: v.total || 0 }))}</div>
      ${stack(c, PILOTS, PCOL)}
      <div class="legend">${PILOTS.filter((k) => c[k]).map((k) => `<span><i style="background:${PCOL[k]}"></i>${esc(t("pilot_" + k))} ${c[k]}</span>`).join("")}${c.vla ? "" : `<span><i style="background:${PCOL.vla}"></i>${esc(t("pilot_vla"))} 0</span>`}</div>
      <h3 style="margin-top:12px">${esc(t("vla_by_rail"))}</h3>
      <ul class="plain">${(v.latest_by_rail || []).map((r) => `<li><span class="chip">${esc(t("rail_" + r.rail))}</span> <span class="pdot" style="background:${PCOL[r.pilot_class]}"></span>${esc(t("pilot_" + r.pilot_class))} · <code>${esc(r.tag)}</code> <span class="muted small">${esc(fdate(r.date))}</span></li>`).join("")}</ul>
      <h3 style="margin-top:12px">${esc(t("vla_milestones"))}</h3>
      ${items.length ? `<ol class="miles">${items.map((i) => `<li class="${i.status_effective === "done" ? "done" : ""}">${tpill(i.status_effective)} <b>${esc(todoTitle(i))}</b>
        <span class="small muted">${esc(t("owner_" + i.owner))}${i.due ? " · " + esc(fdate(i.due)) : ""}</span>
        ${i.probe ? `<span class="chip ${i.probe_ok ? "pok" : "pno"}" title="${esc(i.probe.file + " ∋ " + (i.probe.contains || ""))}">${esc(t(i.probe_ok ? "probe_found" : "probe_missing"))}</span>` : ""}
        ${i.auto_evidence ? `<div class="small">${esc(tf("auto_done", { run: i.auto_evidence.run }))}</div>` : ""}</li>`).join("")}</ol>`
        : `<p class="muted small">${esc(t("todo_none"))}</p>`}
      <p class="small muted">${esc(t("vla_help"))}</p></section>`;
  }
  function wpPanel(d) {
    const w = d.wps || {}; const cs = cards();
    return `<section class="panel"><h2 class="ptitle">${esc(t("wp_title"))}</h2>
      <div class="wpgrid">${(w.areas || []).map((a) => {
        const sc = count(cs.filter((c) => c.area === a.area), (c) => cstate(c).state);
        return `<div class="wprow"><div class="wpname"><b>${esc(t("area_" + a.area))}</b><span class="muted small">${a.total} ${esc(t("cards"))}</span></div>
          <div>${stack(a.counts, VERDICTS, VCOL)}<div class="thin">${stack(sc, STATES, SCOL)}</div>
          <div class="small muted">${VERDICTS.filter((v) => a.counts[v]).map((v) => `${esc(t("verdict_" + v))} ${a.counts[v]}`).join(" · ")}
            ${a.gate_open.length ? ` · <span class="gate-n" title="${esc(a.gate_open.join(", "))}">${esc(tf("wp_gate_open", { n: a.gate_open.length }))}</span>` : ""}
            · ${esc(t("wp_newest"))} ${esc(a.newest_review ? fdate(a.newest_review) : t("empty"))}
            ${a.needs_review.length ? ` · <a href="#board" data-area="${esc(a.area)}" class="go-area rev" title="${esc(a.needs_review.join(", "))}">${esc(tf("wp_needs_review", { n: a.needs_review.length }))}</a>` : ""}</div></div></div>`; }).join("")}</div>
      <div class="legend">${VERDICTS.map((v) => `<span><i style="background:${VCOL[v]}"></i>${esc(t("verdict_" + v))}</span>`).join("")}<span>${esc(t("wp_board"))}: ${STATES.map((s) => `<i style="background:${SCOL[s]}"></i>${esc(t("state_" + s))}`).join(" ")}</span></div>
      <p class="small muted">${esc(t("wp_help"))}</p></section>`;
  }
  function kpiPanel(d) {
    const rows = d.kpis || [];
    const cls = (k) => (k.status !== "measured" ? "k-nm" : k.green ? "k-green" : k.meets_target === false || k.beats_null === false ? "k-red" : "k-amber");
    return `<section class="panel"><h2 class="ptitle">${esc(t("kpi_dash_title"))}</h2>
      <div class="table-wrap"><table class="kpis"><thead><tr><th>${esc(t("kpi_name"))}</th><th>${esc(t("kpi_target"))}</th><th>${esc(t("kpi_value"))}</th><th>${esc(t("kpi_null"))}</th><th>${esc(t("kpi_grade"))}</th><th>${esc(t("kpi_as_of"))}</th><th>${esc(t("kpi_source"))}</th></tr></thead><tbody>
      ${rows.map((k) => { const p = k.primary || {};
        const extra = [];
        if (p.n != null && p.k == null) extra.push(`n = ${esc(p.n)}`);
        if (p.k != null) extra.push(`${esc(p.k)}/${esc(p.n)}`);
        if (p.set_aside_unresolved) extra.push(esc(tf("set_aside", { n: p.set_aside_unresolved })));
        if (p.correct != null) extra.push(`${esc(p.correct)}/${esc(p.n)} · ${esc(tf("fs_detail", { f: p.false_triggers, m: p.missed }))}`);
        if (p.discriminates === false) extra.push(esc(t("kpi_not_discriminating")));
        if (p.run) extra.push(`<code>${esc(p.run)}</code>`);
        return `<tr class="${cls(k)}"><td><b>${esc(t("kpi_l_" + k.key))}</b> <span class="chip">${esc(k.contract === "acceptance" ? t("contract_acceptance") : k.contract.toUpperCase())}</span>
            <div class="small muted"><code title="${esc(k.grant_source)}">${esc(k.grant_source.replace(/^kuanting-vla-uav-guardrail\/docs\/02-implementation\//, ""))}</code></div></td>
          <td class="small">${fmtTarget(k)}</td>
          <td><b>${fmtKpi(k.value, k.unit)}</b>${k.meets_target === true ? ` <span class="mk ok">✓</span>` : k.meets_target === false ? ` <span class="mk no">✗</span>` : ""}
            ${extra.length ? `<div class="small muted">${extra.join(" · ")}</div>` : ""}
            ${(k.also || []).map((a) => `<div class="small muted">${esc(t("kpi_also"))}: ${fmtKpi(a.value, k.unit)} · ${esc(t("grade_" + a.grade))} · ${esc(a.as_of || "")} ${srcCode(a.source)}</div>`).join("")}</td>
          <td class="small">${p.null == null ? esc(t("empty")) : fmtKpi(p.null, k.unit) + (k.beats_null === false ? ` <span class="mk no" title="${esc(t("beats_null_no"))}">✗</span>` : k.beats_null ? ` <span class="mk ok" title="${esc(t("beats_null_yes"))}">✓</span>` : "")}</td>
          <td class="small">${k.status === "measured" ? esc(t("grade_" + p.grade)) : esc(t("empty"))}</td>
          <td class="small">${esc(p.as_of ? fdate(p.as_of) : t("empty"))}</td><td class="small">${srcCode(p.source)}</td></tr>`; }).join("")}
      </tbody></table></div><p class="small muted">${esc(t("kpi_green_rule"))}</p></section>`;
  }
  function videoTile(v, poster, cap) {
    if (!v) return `<div class="ftile none">${poster ? `<img src="/${esc(poster)}" alt="" loading="lazy">` : ""}<div class="small">${esc(t("flights_no_video"))}</div></div>`;
    return `<div class="ftile" data-play="${esc(v.path)}" data-cap="${esc(cap)}">${poster ? `<img src="/${esc(poster)}" alt="" loading="lazy">` : `<video src="/media/${esc(v.path)}#t=4" preload="metadata" muted></video>`}<div class="play">▶</div></div>`;
  }
  function flightCard(f) {
    const p0 = f.p0;
    const stat = (lab, val) => `<div><span class="muted">${esc(lab)}</span> <b>${val}</b></div>`;
    return `<div class="fcard">${videoTile(f.videos[0], f.poster, f.tag)}
      <div class="fbody"><div class="row"><code class="ftag">${esc(f.tag)}</code></div>
        <div class="row small"><span class="chip">${esc(t("rail_" + f.rail))}</span><span class="chip"><span class="pdot" style="background:${PCOL[f.pilot.class]}"></span>${esc(t("pilot_" + f.pilot.class))}</span>
          ${f.shield ? `<span class="chip">${esc(t("shield_" + f.shield))}</span>` : ""}${f.variant && f.pair ? `<span class="chip" title="${esc(f.pair)}">${esc(t("variant"))} ${esc(f.variant)}</span>` : ""}
          <span class="chip ${f.kpi_grade ? "kg" : ""}" title="${esc(f.kpi_grade_reasons.join(", "))}">${esc(t(f.kpi_grade ? "f_kpi_grade" : "f_not_kpi"))}</span></div>
        ${f.stored_kpi_grade === true && !f.kpi_grade ? `<div class="small warn-text" title="${esc(f.stored_kpi_grade_reasons.join(" | "))}">${esc(t("f_stored_claims"))}</div>` : ""}
        <div class="fstats small">${stat(t("f_ticks"), f.ticks == null ? esc(t("not_measured")) : esc(f.ticks))}${stat(t("f_loop"), f.loop_hz == null ? esc(t("not_measured")) : esc(dnum(f.loop_hz, 2)) + " Hz")}
          ${stat(t("f_p0"), p0 ? `${esc(p0.escapes == null ? "?" : p0.escapes)}${p0.rate != null ? ` (${fmtKpi(p0.rate, "ratio")})` : ""}` : esc(t("not_measured")))}
          ${stat(t("f_unsafe"), f.unsafe_p0_ticks == null ? esc(t("not_measured")) : esc(f.unsafe_p0_ticks))}
          ${stat(t("f_within"), f.frac_within_30m == null ? esc(t("not_measured")) : fmtKpi(f.frac_within_30m, "ratio"))}
          ${stat(t("f_nfz"), f.nfz_s == null ? esc(t("not_measured")) : esc(dnum(f.nfz_s, 1)) + " s")}</div>
        ${f.end_reason ? `<div class="small muted">${esc(t("f_end"))}: ${esc(f.end_reason)}</div>` : ""}
        <div class="small muted"><code>${esc(f.pilot.id || "")}</code> · seed ${esc(f.seed ?? "?")} · <code>${esc(f.code_revision || "?")}</code></div></div></div>`;
  }
  function flightsPanel(d) {
    const fl = (d.flights || []).slice(0, 12); const cmp = (d.compares || []).slice(0, 4);
    const days = []; fl.forEach((f) => { const k = f.date || "?"; let g = days.find((x) => x.d === k); if (!g) days.push((g = { d: k, fs: [] })); g.fs.push(f); });
    return `<section class="panel"><h2 class="ptitle">${esc(t("flights_title"))}</h2>
      <p class="small muted">${esc(tf("flights_all", { n: (d.meta || {}).flights_total || 0 }))}</p>
      ${cmp.length ? `<h3>${esc(t("flights_compare"))}</h3><div class="fgrid">${cmp.map((c) => `<div class="fcard">${videoTile(c.videos[0], c.poster, c.key)}
        <div class="fbody"><code class="ftag">${esc(c.key)}</code><div class="small muted">${esc(fdate(c.date))} · ${c.runs.map((r) => `<code>${esc(r)}</code>`).join(" · ")}</div></div></div>`).join("")}</div>` : ""}
      ${days.map((g) => `<h3>${esc(g.d === "?" ? t("empty") : fdate(g.d))}</h3><div class="fgrid">${g.fs.map(flightCard).join("")}</div>`).join("")}
      <p class="small muted">${esc(t("flights_help"))}</p></section>`;
  }
  function timelinePanel(d) {
    const tl = d.timeline || {}; const ov = (S.content && S.content.overview) || {};
    const a = new Date(tl.start || "2026-02-01"), b = new Date(tl.end || "2026-12-01");
    const pos = (x) => Math.max(0, Math.min(100, (100 * (new Date(x) - a)) / (b - a)));
    const months = []; for (let m = new Date(a); m < b; m = new Date(m.getFullYear(), m.getMonth() + 1, 1)) months.push(m);
    const today = `<div class="today" style="left:${pos(new Date())}%"></div>`;
    const gates = (tl.gates || []).map((g) => `<div class="gatemark" style="left:${pos(g.date)}%" title="${esc(t(g.key))}"></div>`).join("");
    const upd = (S.content && S.content.updates && S.content.updates.entries) || [];
    const ttl = (seq) => { const e = upd.find((x) => x.seq === seq); return e ? L(e).title : ""; };
    const lane = (name, inner) => `<div class="grow"><div class="gname">${esc(name)}</div><div class="track">${inner}${gates}${today}</div></div>`;
    const plan = (tl.plan || []).map((p) => { const g = (ov.timeline || [])[p.index] || {}; const x = L(g);
      return lane(x.name || "", `<div class="gbar ${p.overdue ? "late" : ""}" style="left:${pos(p.start)}%;width:${Math.max(1.2, pos(p.end) - pos(p.start))}%;background:${VCOL[p.status] || "var(--todo)"}" title="${esc(x.name || "")} · ${esc(t("verdict_" + p.status))}${p.overdue ? " · " + esc(t("tl_overdue")) : ""}"></div>`); }).join("");
    const maxN = Math.max(1, ...(tl.flights || []).map((f) => f.n));
    return `<section class="panel gantt dash-tl"><h2 class="ptitle">${esc(t("tl_title"))}</h2>
      <div class="axis">${months.map((m) => `<div>${esc(m.toLocaleDateString(LOCALE[S.lang], { month: "short" }))}</div>`).join("")}</div>
      ${plan}
      ${lane(t("tl_updates"), (tl.updates || []).map((u) => `<a class="tdot k-${esc(u.kind)}" href="#updates?v=${u.seq}" style="left:${pos(u.date)}%" title="#${u.seq} · ${esc(fdate(u.date))} · ${esc(ttl(u.seq))}"></a>`).join(""))}
      ${lane(t("tl_flights"), (tl.flights || []).map((f) => `<span class="ftick ${f.vla ? "vla" : ""}" style="left:${pos(f.date)}%;height:${30 + (70 * f.n) / maxN}%" title="${esc(fdate(f.date))}: ${f.n}${f.vla ? " · VLA " + f.vla : ""}"></span>`).join(""))}
      ${lane(t("tl_todo"), (tl.todo_due || []).map((x) => `<span class="tdue ${x.status === "done" ? "done" : new Date(x.due) < new Date() ? "late" : ""}" style="left:${pos(x.due)}%" title="${esc(x.id)} · ${esc(fdate(x.due))}"></span>`).join(""))}
      <div class="legend"><span><i style="background:var(--missing);width:2px"></i>${esc(t("today"))}</span><span>┆ ${esc(t("midterm"))} / ${esc(t("final"))}</span><span><i class="late-sw"></i>${esc(t("tl_overdue"))}</span></div></section>`;
  }
  function todoHTML() {
    const td = (S.dash && S.dash.todo) || { items: [] };
    const f = S.todoF; const q = f.q.trim().toLowerCase();
    const vis = (td.items || []).filter((i) => (!f.owner || i.owner === f.owner)
      && (f.status === "" || (f.status === "notdone" ? i.status_effective !== "done" : i.status_effective === f.status))
      && (!q || JSON.stringify([i.id, i.i18n, i.cards, i.wp]).toLowerCase().includes(q)))
      .sort((x, y) => (y.overdue - x.overdue) || ((x.due || "9999") < (y.due || "9999") ? -1 : (x.due || "9999") > (y.due || "9999") ? 1 : x.id.localeCompare(y.id, undefined, { numeric: true })));
    const opt = (vals, cur, all, pre) => `<option value="">${esc(t(all))}</option>` + vals.map((v) => `<option value="${v}" ${cur === v ? "selected" : ""}>${esc(t(pre + v))}</option>`).join("");
    return `<div class="toolbar"><input type="search" id="t-q" placeholder="${esc(t("search"))}" value="${esc(f.q)}">
        <select id="t-owner">${opt(OWNERS, f.owner, "todo_all_owners", "owner_")}</select>
        <select id="t-status"><option value="notdone" ${f.status === "notdone" ? "selected" : ""}>${esc(t("todo_not_done"))}</option>${opt(TSTATES, f.status, "todo_all_status", "tstatus_")}</select>
        <span class="spacer"></span><span class="small muted">${vis.length} / ${(td.items || []).length}${td.overdue ? ` · <b class="late-t">${esc(tf("todo_overdue_n", { n: td.overdue }))}</b>` : ""}</span></div>
      ${(td.items || []).length ? `<div class="table-wrap"><table class="todo"><thead><tr><th>${esc(t("col_id"))}</th><th>${esc(t("col_title"))}</th><th>${esc(t("col_owner"))}</th><th>${esc(t("col_status"))}</th><th>${esc(t("col_wp"))}</th><th>${esc(t("col_cards"))}</th><th>${esc(t("col_due"))}</th></tr></thead><tbody>
        ${vis.map((i) => `<tr class="${i.overdue ? "late" : ""}"><td><code>${esc(i.id)}</code></td><td>${esc(todoTitle(i))}${(i.needs || []).length ? `<div class="small muted">${i.needs.map((n) => esc(t("need_" + n))).join(" · ")}</div>` : ""}${i.auto_evidence ? `<div class="small">${esc(tf("auto_done", { run: i.auto_evidence.run }))}</div>` : ""}</td>
          <td class="small">${esc(t("owner_" + i.owner))}</td><td>${tpill(i.status_effective)}</td><td><span class="chip">${esc(t("area_" + i.wp))}</span></td>
          <td class="small">${(i.cards || []).map((c) => `<a class="cchip" href="#board?card=${encodeURIComponent(c)}">${esc(c)}</a>`).join(" ")}</td>
          <td class="small">${i.due ? esc(fdate(i.due)) + (i.overdue ? ` <b class="late-t">${esc(t("overdue"))}</b>` : "") : esc(t("empty"))}</td></tr>`).join("")}</tbody></table></div>`
        : `<p class="muted small">${esc(t("todo_none"))}</p>`}`;
  }
  function renderDashboard() {
    const d = S.dash;
    if (!d || d.missing || d.error) {
      return `<h1>${esc(t("nav_dashboard"))}</h1><div class="panel callout warn"><p>${esc(t(S.dashErr || (d && d.error) ? "dash_unreachable" : "dash_missing"))}</p><code>python tools/build_dashboard.py</code></div>
        <section class="panel" id="dash-live" style="margin-top:14px">${liveHTML()}</section>`;
    }
    return `${dashHeader(d)}
      <div class="dgrid">
        ${vlaPanel(d)}
        <section class="panel" id="dash-live">${liveHTML()}</section>
      </div>
      ${wpPanel(d)}
      ${kpiPanel(d)}
      ${flightsPanel(d)}
      ${timelinePanel(d)}
      <section class="panel"><h2 class="ptitle">${esc(t("todo_title"))}</h2><div id="dash-todo">${todoHTML()}</div></section>`;
  }

  // ------------------------------------------------------------ diagrams
  function svgDiagram(nodes, edges, opt) {
    const W = opt.cols * opt.w + (opt.cols - 1) * opt.gx + 2 * opt.m;
    const H = opt.rows * opt.h + (opt.rows - 1) * opt.gy + 2 * opt.m;
    const pos = {}; nodes.forEach((n) => (pos[n.key] = { x: opt.m + n.c * (opt.w + opt.gx), y: opt.m + n.r * (opt.h + opt.gy) }));
    const anchor = (k, side) => { const p = pos[k]; if (!p) return [0, 0];
      return { l: [p.x, p.y + opt.h / 2], r: [p.x + opt.w, p.y + opt.h / 2], t: [p.x + opt.w / 2, p.y], b: [p.x + opt.w / 2, p.y + opt.h] }[side]; };
    const e = edges.map(([a, sa, b, sb, lbl, dash]) => {
      const [x1, y1] = anchor(a, sa), [x2, y2] = anchor(b, sb);
      const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
      const d = (sa === "l" || sa === "r") && (sb === "l" || sb === "r") ? `M${x1},${y1} C${mx},${y1} ${mx},${y2} ${x2},${y2}` : `M${x1},${y1} C${x1},${my} ${x2},${my} ${x2},${y2}`;
      return `<path class="edge${dash ? " dash" : ""}" d="${d}"/>${lbl ? `<text class="lbl" x="${mx + 4}" y="${my - 4}">${esc(lbl)}</text>` : ""}`;
    }).join("");
    const n = nodes.map((nd, i) => { const p = pos[nd.key]; const x = L(nd);
      // Chinese glyphs are about twice as wide: cut the one-line hint by width, not by count.
      const maxSub = S.lang === "zh" ? 15 : 30;
      const label = x.label || nd.key; const sub = (x.short || "").slice(0, maxSub) + ((x.short || "").length > maxSub ? "…" : "");
      return `<g class="node ${nd.built ? "b-" + nd.built : ""} ${S.flowSel === nd.key ? "on" : ""}" data-key="${esc(nd.key)}" transform="translate(${p.x},${p.y})">
        <rect width="${opt.w}" height="${opt.h}"/>
        ${opt.numbers ? `<circle class="num" cx="0" cy="0" r="12"/><text class="num-t" x="0" y="4" text-anchor="middle">${i + 1}</text>` : ""}
        <text x="${opt.w / 2}" y="${opt.h / 2 - 4}" text-anchor="middle">${esc(label)}</text>
        <text class="sub" x="${opt.w / 2}" y="${opt.h / 2 + 15}" text-anchor="middle">${esc(sub)}</text></g>`; }).join("");
    return `<svg class="diagram" viewBox="-14 -14 ${W + 28} ${H + 28}" role="img"><defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="var(--muted)"/></marker></defs>${e}${n}</svg>`;
  }
  const BUILT_LAYOUT = { camera: [0, 0], detector: [1, 0], identity: [2, 0], tracker: [3, 0], pilot: [3, 1], shield: [2, 1], autopilot: [1, 1], fenceguard: [3, 2], hud: [2, 2] };
  const BUILT_EDGES = [["camera", "r", "detector", "l"], ["detector", "r", "identity", "l"], ["identity", "r", "tracker", "l"], ["tracker", "b", "pilot", "t"],
    ["pilot", "l", "shield", "r"], ["shield", "l", "autopilot", "r"], ["fenceguard", "t", "pilot", "b"], ["shield", "b", "hud", "t"], ["autopilot", "t", "camera", "b", "", true]];
  const GRANT_LAYOUT = { operator: [0, 0], vla: [1, 0], prefix_compiler: [2, 0], policy_bundle: [3, 0], shield: [1, 1], stress_harness: [3, 1], mavros: [1, 2], gcs: [2, 3], ardupilot: [1, 3], geofence: [0, 3] };
  const GRANT_EDGES = [["operator", "r", "vla", "l"], ["prefix_compiler", "l", "vla", "r", "prefix"], ["policy_bundle", "l", "prefix_compiler", "r"],
    ["vla", "b", "shield", "t", "4-D 10 Hz"], ["policy_bundle", "b", "shield", "r", "IR"], ["shield", "b", "mavros", "t"], ["mavros", "b", "ardupilot", "t", "MAVLink"],
    ["geofence", "r", "ardupilot", "l", "", true], ["ardupilot", "r", "gcs", "l", "", true], ["stress_harness", "l", "shield", "r", "", true]];

  function renderFlow() {
    const f = S.content.flow || {};
    const built = S.flowTab === "built";
    const src = built ? (f.demo_pipeline || []) : (f.grant_architecture || []);
    const lay = built ? BUILT_LAYOUT : GRANT_LAYOUT;
    const nodes = src.filter((n) => lay[n.key]).map((n) => Object.assign({}, n, { c: lay[n.key][0], r: lay[n.key][1], built: built ? null : n.built }));
    const svg = svgDiagram(nodes, built ? BUILT_EDGES : GRANT_EDGES, { cols: 4, rows: built ? 3 : 4, w: 210, h: 70, gx: 60, gy: 46, m: 10, numbers: built });
    const sel = src.find((n) => n.key === S.flowSel) || src[0];
    const sx = L(sel);
    const hud = f.hud_callouts || [];
    const hsel = hud.find((h) => h.key === S.hudSel);
    return `
      <div class="tabs"><button data-tab="built" class="${built ? "on" : ""}">${esc(t("flow_built"))}</button><button data-tab="grant" class="${built ? "" : "on"}">${esc(t("flow_grant"))}</button></div>
      <div class="diagram-wrap">
        <div class="panel">${svg}
          ${built ? "" : `<div class="legend"><span><i style="background:var(--done)"></i>${esc(t("built_yes"))}</span><span><i style="background:var(--partial)"></i>${esc(t("built_partial"))}</span><span><i style="background:var(--missing)"></i>${esc(t("built_no"))}</span></div>`}
          <p class="small muted" style="margin-top:8px">${esc(t("flow_click"))}</p></div>
        <div class="panel detail">${sel ? `<h3>${esc(sx.label || sel.key)}</h3>
          <div class="meta row">${sel.built ? `<span class="chip">${esc(t("built_" + sel.built))}</span>` : ""}${sel.rate ? `<span class="chip">${esc(sel.rate)}</span>` : ""}${sel.file ? `<code>${esc(sel.file)}</code>` : ""}</div>
          <p><b>${esc(sx.short)}</b></p><p>${esc(sx.detail)}</p>` : ""}</div>
      </div>

      ${figures().length ? `<h2>${esc(t("fig_title"))}</h2><p class="small muted">${esc(t("fig_lead"))}</p>
      <div class="grid g2">${figures().map(figureCard).join("")}</div>` : ""}

      <h2>${esc(t("shield_tick"))}</h2>
      <div class="grid g2">
        <div class="panel"><h3>${esc(t("ours"))}</h3><ol class="steps">${(f.shield_steps || []).map((s) => { const x = L(s); return `<li><b>${esc(x.label)}</b><div class="small">${esc(x.detail)}</div></li>`; }).join("")}</ol></div>
        <div class="panel"><h3>${esc(t("grant_spec"))}</h3><ol class="steps">${(f.grant_shield_steps || []).map((s) => { const x = L(s); return `<li class="b-${esc(s.built || "")}"><b>${esc(x.label)}</b> <span class="chip">${esc(t("built_" + (s.built || "no")))}</span><div class="small">${esc(x.detail)}</div></li>`; }).join("")}</ol></div>
      </div>

      <h2>${esc(t("hud_guide"))}</h2>
      <div class="diagram-wrap">
        <div class="panel"><div class="hud"><img src="/media/img/policy_hud_nfz2.jpg" alt="policy HUD">
          ${hud.map((h, i) => `<div class="hot ${S.hudSel === h.key ? "on" : ""}" data-hud="${esc(h.key)}" style="left:${(100 * h.x) / 1280}%;top:${(100 * h.y) / 720}%">${i + 1}</div>`).join("")}</div></div>
        <div class="panel">${hsel ? `<h3>${esc(L(hsel).label)}</h3><p>${esc(L(hsel).detail)}</p><hr style="border:0;border-top:1px solid var(--line)">` : ""}
          <ol>${hud.map((h) => `<li class="hud-item" data-hud="${esc(h.key)}" style="cursor:pointer"><b>${esc(L(h).label)}</b></li>`).join("")}</ol>
          <h3 style="margin-top:12px">${esc(t("status_legend"))}</h3>
          ${(f.statuses || []).map((s) => `<div class="small" style="margin-bottom:6px"><span class="sw" style="background:${esc(s.color)}"></span><b>${esc(L(s).label)}</b> — ${esc(L(s).meaning)}</div>`).join("")}
        </div>
      </div>`;
  }

  // ------------------------------------------------------------ grant
  const TOPO_ICON = {
    desktop: '<svg viewBox="0 0 40 40"><rect x="4" y="6" width="32" height="20" rx="2" fill="none" stroke="var(--accent)" stroke-width="3"/><path d="M14 34h12M20 26v8" stroke="var(--accent)" stroke-width="3"/></svg>',
    orin: '<svg viewBox="0 0 40 40"><rect x="8" y="8" width="24" height="24" rx="3" fill="none" stroke="var(--accent)" stroke-width="3"/><rect x="14" y="14" width="12" height="12" fill="var(--accent)"/><path d="M4 14h4M4 20h4M4 26h4M32 14h4M32 20h4M32 26h4" stroke="var(--accent)" stroke-width="2.5"/></svg>',
    drone: '<svg viewBox="0 0 40 40"><circle cx="9" cy="10" r="5" fill="none" stroke="var(--accent)" stroke-width="2.5"/><circle cx="31" cy="10" r="5" fill="none" stroke="var(--accent)" stroke-width="2.5"/><circle cx="9" cy="30" r="5" fill="none" stroke="var(--accent)" stroke-width="2.5"/><circle cx="31" cy="30" r="5" fill="none" stroke="var(--accent)" stroke-width="2.5"/><rect x="15" y="15" width="10" height="10" rx="2" fill="var(--accent)"/></svg>',
  };
  const TOPO_BOXES = { dev: ["desktop"], hil: ["desktop", "orin"], flight: ["orin", "drone"] };

  function renderGrant() {
    const ov = S.content.overview || {};
    const kp = ov.kpis || [];
    const start = new Date("2026-02-01"), end = new Date("2026-12-01");
    const pos = (d) => Math.max(0, Math.min(100, (100 * (new Date(d) - start)) / (end - start)));
    const months = ["Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov"];
    const kstat = { measured: "DONE", partial: "PARTIAL", not_measured: "MISSING", wrong_definition: "DRIFT" };
    return `
      <h1>${esc(t("nav_grant"))}</h1>
      <div class="grid g2">${(ov.wps || []).map((w) => { const x = L(w); return `
        <div class="panel"><h3>${esc(x.name || w.key)}</h3><p>${esc(x.plain)}</p>
          ${w.grant_quote ? `<blockquote>“${esc(w.grant_quote)}” — ${esc(w.grant_page)}</blockquote>` : ""}
          <h4 style="margin-top:10px">${esc(t("grant_asks"))}</h4>${list(x.grant_asks)}
          <p><b>${esc(t("our_status"))}:</b> ${esc(x.status)}</p>
          <div class="row small">${(w.kpis || []).map((k) => `<span class="chip">${esc(k)}</span>`).join("")}</div>
          <div class="small muted" style="margin-top:6px">${(w.files || []).map((f) => `<code>${esc(f)}</code>`).join(" · ")}</div>
          <a href="#board" data-area="${esc(w.key)}" class="go-area small">${esc(t("open_board"))}</a></div>`; }).join("")}</div>

      <h2>${esc(t("kpi_table"))}</h2>
      <div class="panel table-wrap"><table><thead><tr><th>${esc(t("kpi_name"))}</th><th>${esc(t("kpi_type"))}</th><th>${esc(t("kpi_status"))}</th><th>${esc(t("kpi_value"))}</th><th>${esc(t("kpi_where"))}</th><th>${esc(t("kpi_gap"))}</th></tr></thead><tbody>
        ${kp.map((k) => { const x = L(k); return `<tr><td><b>${esc(x.name)}</b><div class="small muted">${esc(x.plain)}</div></td>
          <td><span class="chip">${esc(k.contract === "acceptance" ? t("contract_acceptance") : (k.contract || "").toUpperCase())}</span></td>
          <td><span class="pill v-${kstat[k.status] || "MISSING"}">${esc(t("kpi_" + k.status))}</span></td>
          <td class="small">${esc(k.value || t("empty"))}</td><td class="small">${esc(x.where)}</td><td class="small">${esc(x.gap)}</td></tr>`; }).join("")}
      </tbody></table></div>

      <h2>${esc(t("topologies"))}</h2>
      <div class="grid g3">${(ov.topologies || []).map((tp) => { const x = L(tp); return `
        <div class="topo panel ${tp.we_have ? "have" : "not"}"><h3>${esc(x.name || tp.key)} <span class="pill ${tp.we_have ? "v-DONE" : "v-MISSING"}">${esc(t(tp.we_have ? "we_have" : "we_dont"))}</span></h3>
          <div class="boxes">${(TOPO_BOXES[tp.key] || []).map((b) => `<div>${TOPO_ICON[b]}${esc(b === "desktop" ? "Desktop" : b === "orin" ? "Jetson Orin" : "Drone")}</div>`).join("")}</div>
          <p class="small"><b>${esc(x.runs_where)}</b></p><p class="small">${esc(x.purpose)}</p><p class="small muted">${esc(x.our_status)}</p></div>`; }).join("")}</div>

      <h2>${esc(t("timeline"))}</h2>
      <div class="panel gantt">
        <div class="axis">${months.map((m) => `<div>${m}</div>`).join("")}</div>
        ${(ov.timeline || []).map((g) => { const x = L(g); const a = pos(g.start), b = pos(g.end); return `
          <div class="grow"><div class="gname" title="${esc(x.note)}">${esc(x.name)}</div><div class="track">
            <div class="gbar" style="left:${a}%;width:${Math.max(1.2, b - a)}%;background:${VCOL[g.status] || "var(--todo)"}" title="${esc(x.name)} · ${esc(t("verdict_" + g.status))} · ${esc(x.note)}"></div>
            <div class="gatemark" style="left:${pos("2026-07-20")}%"></div><div class="gatemark" style="left:${pos("2026-11-30")}%"></div>
            <div class="today" style="left:${pos(new Date())}%"></div></div></div>`; }).join("")}
        <div class="legend">${VERDICTS.map((v) => `<span><i style="background:${VCOL[v]}"></i>${esc(t("verdict_" + v))}</span>`).join("")}<span><i style="background:var(--missing);width:2px"></i>${esc(t("today"))}</span><span>┆ ${esc(t("midterm"))} / ${esc(t("final"))}</span></div>
      </div>`;
  }

  // ------------------------------------------------------------ board
  function visibleCards() {
    const f = S.filters; const q = f.q.trim().toLowerCase();
    return cards().filter((c) => (!f.area || c.area === f.area) && (!f.verdict || c.status === f.verdict) && (!f.prio || c.priority === f.prio) && (!f.gate || c.priority === "gate")
      && (!q || JSON.stringify([c.id, c.i18n && c.i18n[S.lang], c.evidence, c.grant]).toLowerCase().includes(q)));
  }
  function kcard(c) {
    const x = L(c); const st = cstate(c);
    return `<div class="kcard v-${c.status}-l" draggable="true" data-id="${esc(c.id)}">
      <div class="meta"><span class="id">${esc(c.id)}</span>${pill(c.status)}<span class="icons">${(c.media || []).length ? "🖼" : ""}${st.note ? " 📝" : ""}</span></div>
      <div class="t">${esc(x.title || c.id)}</div>
      <div class="meta"><span class="chip">${esc(t("area_" + c.area))}</span><span class="chip ${c.priority}">${esc(t("prio_" + c.priority))}</span>${c.needs_pi ? `<span class="chip pi">PI</span>` : ""}<span class="chip">${esc(t("effort_" + (c.effort || "unknown")))}</span></div></div>`;
  }
  function renderBoard() {
    const vis = visibleCards();
    const order = (c) => PRIOS.indexOf(c.priority) * 1000 + AREAS.indexOf(c.area) * 100;
    const f = S.filters;
    const opt = (vals, cur, lab, pre) => `<option value="">${esc(lab)}</option>` + vals.map((v) => `<option value="${v}" ${cur === v ? "selected" : ""}>${esc(t(pre + v))}</option>`).join("");
    return `
      <div class="toolbar">
        <input type="search" id="f-q" placeholder="${esc(t("search"))}" value="${esc(f.q)}">
        <select id="f-area">${opt(AREAS, f.area, t("all_areas"), "area_")}</select>
        <select id="f-verdict">${opt(VERDICTS, f.verdict, t("all_verdicts"), "verdict_")}</select>
        <select id="f-prio">${opt(PRIOS, f.prio, t("all_priorities"), "prio_")}</select>
        <label class="row small"><input type="checkbox" id="f-gate" ${f.gate ? "checked" : ""}> ${esc(t("only_gate"))}</label>
        <span class="spacer"></span><span class="muted small">${vis.length} / ${cards().length} ${esc(t("cards"))}</span>
        <button class="btn" id="b-export">${esc(t("export"))}</button><button class="btn" id="b-reset">${esc(t("reset"))}</button>
      </div>
      <p class="small muted">${esc(t("drag_hint"))}</p>
      <div class="board">${STATES.map((s) => { const cs = vis.filter((c) => cstate(c).state === s).sort((a, b) => order(a) - order(b) || a.id.localeCompare(b.id));
        return `<div class="col" data-state="${s}"><div class="col-head"><span class="dot" style="background:${SCOL[s]}"></span>${esc(t("state_" + s))}<span class="count">${cs.length}</span></div>
          <div class="col-body">${cs.map(kcard).join("")}</div></div>`; }).join("")}</div>`;
  }
  function openCard(id) {
    const c = cards().find((x) => x.id === id); if (!c) return;
    const x = L(c); const st = cstate(c);
    const media = (c.media || []).map(mediaOf).filter(Boolean);
    $("#drawer-inner").innerHTML = `
      <div class="dhead"><div><div class="row"><span class="id mono">${esc(c.id)}</span>${pill(c.status)}<span class="chip">${esc(t("area_" + c.area))}</span><span class="chip ${c.priority}">${esc(t("prio_" + c.priority))}</span>${c.needs_pi ? `<span class="chip pi">${esc(t("needs_pi"))}</span>` : ""}<span class="chip">${esc(t("effort_" + (c.effort || "unknown")))}</span></div>
        <h2>${esc(x.title)}</h2></div><button class="x" id="d-close" aria-label="${esc(t("close"))}">×</button></div>
      <div class="small muted">${esc(t("move_to"))}</div>
      <div class="states">${STATES.map((s) => `<button data-state="${s}" class="${st.state === s ? "on" : ""}" style="${st.state === s ? `background:${SCOL[s]}` : ""}">${esc(t("state_" + s))}</button>`).join("")}</div>
      <h3>${esc(t("summary"))}</h3><p>${esc(x.summary)}</p>
      <div class="twocol"><div class="box ok"><h4>${esc(t("have"))}</h4>${x.have && x.have.length ? `<ul class="check">${x.have.map((h) => `<li>${esc(h)}</li>`).join("")}</ul>` : `<p class="muted">${t("empty")}</p>`}</div>
        <div class="box bad"><h4>${esc(t("missing"))}</h4>${x.missing && x.missing.length ? `<ul class="check">${x.missing.map((h) => `<li>${esc(h)}</li>`).join("")}</ul>` : `<p class="muted">${t("empty")}</p>`}</div></div>
      <h3 style="margin-top:14px">${esc(t("why"))}</h3><p>${esc(x.why)}</p>
      <h3>${esc(t("next"))}</h3>${list(x.next, "arrow")}
      ${c.grant && (c.grant.quote || c.grant.page) ? `<h3>${esc(t("grant_ref"))}</h3><blockquote>${c.grant.quote ? `“${esc(c.grant.quote)}”` : ""} — ${esc(c.grant.page || "")}</blockquote>` : ""}
      ${(c.evidence || []).length ? `<h3 style="margin-top:14px">${esc(t("evidence"))}</h3><ul class="ev">${c.evidence.map((e) => `<li>${esc(e)}</li>`).join("")}</ul>` : ""}
      ${media.length ? `<h3>${esc(t("media"))}</h3><div class="thumbs">${media.map((m) => `<div class="thumb" data-media="${m.id}">${m.kind === "img" ? `<img src="${m.src}" alt="">` : `<video src="${m.src}#t=4" preload="metadata" muted></video><div class="play">▶</div>`}</div>`).join("")}</div>` : ""}
      <h3 style="margin-top:14px">${esc(t("notes"))}</h3><textarea id="d-note" placeholder="${esc(t("notes_ph"))}">${esc(st.note)}</textarea>
      ${st.history.length ? `<h3 style="margin-top:14px">${esc(t("history"))}</h3><ul class="hist">${st.history.slice().reverse().map((h) => `<li>${esc(new Date(h.t).toLocaleString())}: ${esc(t("state_" + h.from))} → ${esc(t("state_" + h.to))}</li>`).join("")}</ul>` : ""}`;
    $("#drawer").classList.add("open"); $("#scrim").classList.add("open"); $("#drawer").setAttribute("aria-hidden", "false");
    $("#d-close").onclick = closeDrawer;
    $$("#drawer .states button").forEach((b) => (b.onclick = () => { setCard(c, { state: b.dataset.state }); openCard(id); if (route() === "board") refreshBoard(); }));
    $("#d-note").oninput = (e) => {
      const cur = cstate(c); const now = new Date().toISOString();
      const log = cur.note_log.map((x) => Object.assign({}, x)); const last = log[log.length - 1];
      if (last && Date.now() - Date.parse(last.t) < 600000) { last.to = e.target.value.length; last.t = now; }
      else log.push({ t: now, from: cur.note.length, to: e.target.value.length });
      while (log.length > 20) log.shift();
      S.board.cards[c.id] = Object.assign({}, cur, { note: e.target.value, note_t: now, note_log: log }); save();
    };
    $$("#drawer .thumb").forEach((el) => (el.onclick = () => openMedia(el.dataset.media, x.title)));
    location.hash = "board?card=" + encodeURIComponent(id);
  }
  function closeDrawer() {
    $("#drawer").classList.remove("open"); $("#scrim").classList.remove("open"); $("#drawer").setAttribute("aria-hidden", "true");
    if (location.hash.startsWith("#board?")) history.replaceState(null, "", "#board");
    if (route() === "board") refreshBoard();
  }
  function refreshBoard() {
    const y = window.scrollY, b = $(".board"), bx = b ? b.scrollLeft : 0, by = b ? b.scrollTop : 0;
    $("#main").innerHTML = renderBoard(); bind("board"); window.scrollTo(0, y);
    const nb = $(".board"); if (nb) { nb.scrollLeft = bx; nb.scrollTop = by; }
  }
  function openMedia(id, caption) {
    const m = mediaOf(id); if (!m) return;
    $("#lightbox-inner").innerHTML = `${m.kind === "img" ? `<img src="${m.src}" alt="">` : `<video src="${m.src}" controls autoplay></video>`}<div class="cap">${esc(caption || "")}</div>`;
    $("#lightbox").classList.add("open");
  }

  // ------------------------------------------------------------ evidence
  function renderEvidence() {
    const f = S.content.flow || {};
    const res = f.results || [];
    const bar = (v, max) => (v == null ? t("empty") : `<div class="row"><div class="bar" style="width:90px"><span style="width:${Math.min(100, (100 * v) / max)}%"></span></div><span class="small">${max === 1 ? pct(v) : v}</span></div>`);
    return `
      <h1>${esc(t("examples"))}</h1>
      <div class="grid g2">${(f.examples || []).map((e) => { const m = mediaOf(e.media); const x = L(e); return `
        <div class="panel ex">${m ? `<div class="media" data-media="${m.id}" data-cap="${esc(x.title)}">${m.kind === "img" ? `<img src="${m.src}" alt="${esc(x.title)}">` : `<video src="${m.src}#t=6" preload="metadata" muted></video><div class="play">▶</div>`}</div>` : ""}
          <div class="row"><h3 style="margin:0">${esc(x.title)}</h3><span class="spacer"></span><span class="chip">${esc(e.date || "")}</span><span class="chip">${esc(t("rail_" + (e.rail || "airsim")))}</span></div>
          <dl><dt>${esc(t("what_you_see"))}</dt><dd>${esc(x.what)}</dd><dt>${esc(t("proves"))}</dt><dd class="yes">${esc(x.proves)}</dd><dt>${esc(t("not_proves"))}</dt><dd class="no">${esc(x.not_proves)}</dd></dl></div>`; }).join("")}</div>

      ${figures().length ? `<h2>${esc(t("fig_title"))}</h2><p class="small muted">${esc(t("fig_lead"))}</p>
      <div class="grid g2">${figures().map(figureCard).join("")}</div>` : ""}

      <h2>${esc(t("results"))}</h2>
      <div class="panel callout warn small">${esc(t("not_contract"))}</div>
      <div class="panel table-wrap" style="margin-top:10px"><table><thead><tr><th>${esc(t("run"))}</th><th>${esc(t("date"))}</th><th>${esc(t("rail"))}</th><th>${esc(t("policy"))}</th>
        <th>${esc(t("within_30m"))}</th><th>${esc(t("estimate_on_car"))}</th><th>${esc(t("p0_escape"))}</th><th>${esc(t("nfz_s"))}</th><th>${esc(t("detector_hz"))}</th><th>${esc(t("shield_corrections"))}</th><th>${esc(t("notes_col"))}</th></tr></thead><tbody>
        ${res.map((r) => { const m = r.metrics || {}; return `<tr><td><code>${esc(r.run)}</code></td><td class="small">${esc(r.date)}</td><td><span class="chip">${esc(t("rail_" + (r.rail || "airsim")))}</span></td><td class="small">${esc(r.policy)}</td>
          <td>${bar(m.within_30m, 1)}</td><td>${bar(m.estimate_on_car, 1)}</td><td>${m.p0_escape == null ? t("empty") : esc(m.p0_escape)}</td><td>${m.nfz_s == null ? t("empty") : esc(m.nfz_s)}</td>
          <td>${bar(m.detector_hz, 4)}</td><td>${m.shield_corrections == null ? t("empty") : esc(m.shield_corrections)}</td>
          <td>${r.notes && Lstr(r.notes) ? `<details class="note"><summary title="${esc(Lstr(r.notes))}">ⓘ</summary><div class="small">${fmt(Lstr(r.notes))}</div></details>` : ""}</td></tr>`; }).join("")}
      </tbody></table></div>`;
  }

  // ------------------------------------------------------------ glossary
  function renderGlossary() {
    const g = (S.content.flow && S.content.flow.glossary) || [];
    return `<h1>${esc(t("glossary"))}</h1>
      <div class="toolbar"><input type="search" id="g-q" placeholder="${esc(t("search"))}"></div>
      <div class="panel"><dl class="gloss" id="gloss">${g.slice().sort((a, b) => a.term.localeCompare(b.term)).map((x) => `<div class="gi"><dt>${esc(x.term)}</dt><dd>${esc(Lstr(x))}</dd></div>`).join("")}</dl></div>`;
  }


  // ------------------------------------------------------------ updates
  function renderBadge() {
    const el = $("#upd-badge"); if (!el) return;
    const us = updates(); const last = us[us.length - 1];
    if (!last) { el.style.display = "none"; return; }
    el.style.display = "";
    const seen = +(safeGet("seenUpdate") || 0);
    el.innerHTML = `<span class="bdot ${last.seq > seen ? "on" : ""}"></span><span class="btxt">${esc(t("last_update"))} #${last.seq} · ${esc(fdate(last.date))}</span>`;
    const pv = S.content && S.content.project_version;
    el.title = `${t("project")} ${pv || "?"}${S.board.updated ? ` · ${t("board_saved")} ${ftime(S.board.updated)}` : ""}`;
  }
  function latestPanel() {
    const us = updates(); const e = us[us.length - 1]; if (!e) return "";
    const x = L(e);
    return `<a class="latest" href="#updates?v=${e.seq}"><div class="small muted">${esc(t("upd_latest"))} · #${e.seq} · ${esc(fdate(e.date))}</div><b>${esc(x.title)}</b></a>`;
  }
  function updMatches(e) {
    const f = S.updF; const q = f.q.trim().toLowerCase();
    if (f.kind && e.kind !== f.kind) return false;
    if (f.retracted && !(e.changes || []).some((c) => c.cat === "retracted")) return false;
    if (f.media && !(e.media || []).length) return false;
    if (q && !JSON.stringify([e.version, e.i18n && e.i18n[S.lang], (e.changes || []).map((c) => c.i18n && c.i18n[S.lang]), e.files, e.cards]).toLowerCase().includes(q)) return false;
    return true;
  }
  function timelineStrip(us) {
    if (!us.length) return "";
    const first = new Date((us[0].date_start || us[0].date) + "T00:00:00"), now = new Date();
    const span = Math.max(1, now - first);
    const pos = (d) => Math.max(0, Math.min(100, (100 * (new Date(d + "T00:00:00") - first)) / span));
    const months = []; const m0 = new Date(first.getFullYear(), first.getMonth() + 1, 1);
    for (let d = m0; d <= now; d = new Date(d.getFullYear(), d.getMonth() + 1, 1)) months.push(d);
    return `<div class="panel tl"><div class="small muted">${esc(t("upd_timeline"))}</div><div class="tl-axis">
      ${months.map((d) => `<div class="tl-month" style="left:${(100 * (d - first)) / span}%">${esc(d.toLocaleDateString(LOCALE[S.lang], { month: "short" }))}</div>`).join("")}
      ${us.map((e) => `<button class="tl-dot k-${e.kind}" data-seq="${e.seq}" style="left:${pos(e.date)}%" title="#${e.seq} · ${esc(fdate(e.date))} · ${esc(L(e).title)}"></button>`).join("")}
      ${us.filter((e) => e.kind === "release").map((e) => `<span class="tl-lab" style="left:${pos(e.date)}%">${esc(e.version)}</span>`).join("")}
      <div class="tl-today" style="left:100%"></div></div>
      <div class="legend"><span><i class="lg k-release"></i>${esc(t("upd_kind_release"))}</span><span><i class="lg k-work"></i>${esc(t("upd_kind_work"))}</span><span><i class="lg k-site"></i>${esc(t("upd_kind_site"))}</span></div></div>`;
  }
  // Releases a day apart would print on top of each other: stack each label on the lowest row it fits.
  function layoutTimeline() {
    $$(".tl-axis").forEach((ax) => {
      const rows = [];
      [...ax.querySelectorAll(".tl-lab")].sort((a, b) => parseFloat(a.style.left) - parseFloat(b.style.left)).forEach((l) => {
        l.hidden = false; l.style.bottom = "";
        const r = l.getBoundingClientRect();
        let i = rows.findIndex((right) => r.left >= right + 4);
        if (i < 0) { i = rows.length; rows.push(0); }
        if (i > 3) { l.hidden = true; return; }
        rows[i] = r.right; l.style.bottom = 12 + i * 13 + "px"; l.style.setProperty("--lead", i * 13 + 4 + "px");
      });
    });
  }
  window.addEventListener("resize", () => { clearTimeout(layoutTimeline._t); layoutTimeline._t = setTimeout(layoutTimeline, 100); });
  function entryHTML(e, open) {
    const x = L(e);
    const byCat = {}; (e.changes || []).forEach((c) => (byCat[c.cat] = byCat[c.cat] || []).push(c));
    const cardChip = (id) => { const c = cards().find((k) => k.id === id); return `<a class="cchip" href="#board?card=${encodeURIComponent(id)}">${esc(id)}${c ? " " + pill(c.status) : ""}</a>`; };
    const media = (e.media || []).map(mediaOf).filter(Boolean);
    const src = e.source || {};
    return `<article class="panel entry ${open ? "open" : ""}" id="upd-${e.seq}">
      <div class="erail"><span class="eseq k-${e.kind}">#${e.seq}</span></div>
      <div class="ebody">
        <div class="row"><span class="chip k-${e.kind}">${esc(t("upd_kind_" + e.kind))}</span><span class="chip" title="${esc(t("upd_version_help"))}">${esc(e.version)}</span>
          <span class="small muted">${esc(fdate(e.date))}${e.date_start ? " (" + esc(fdate(e.date_start)) + " →)" : ""}</span>
          ${e.backfilled ? `<span class="chip">${esc(t("upd_backfilled"))}</span>` : ""}<span class="spacer"></span><span class="small muted">${esc(verLabel(e))}</span></div>
        <h3>${esc(x.title)}</h3><p>${fmt(x.summary)}</p>
        ${(e.highlights || []).length ? `<div class="tiles">${e.highlights.map((h) => `<div class="tile"><div class="big-number">${esc(h.value)}</div><div class="small muted">${esc(Lstr(h))}</div></div>`).join("")}</div>` : ""}
        ${e.chart && e.chart.type === "verdict_stack" ? `<div style="margin:8px 0">${stack(e.chart.values, VERDICTS, VCOL)}<div class="legend">${VERDICTS.map((v) => `<span><i style="background:${VCOL[v]}"></i>${esc(t("verdict_" + v))} ${e.chart.values[v] || 0}</span>`).join("")}</div></div>` : ""}
        <button class="btn small etoggle" data-seq="${e.seq}">${esc(t(open ? "upd_less" : "upd_more"))}</button>
        <div class="edetail">
          ${CATS.filter((k) => byCat[k]).map((k) => `<div class="cat"><div class="cat-h" style="color:${CATCOL[k]}" ${k === "retracted" ? `title="${esc(t("cat_retracted_help"))}"` : ""}><span class="cdot" style="background:${CATCOL[k]}"></span>${esc(t("cat_" + k))} <span class="muted small">${byCat[k].length}</span></div>
            <ul>${byCat[k].map((c) => `<li>${fmt(Lstr(c))}${(c.cards || []).length ? " " + c.cards.map(cardChip).join(" ") : ""}</li>`).join("")}</ul></div>`).join("")}
          ${(e.cards || []).length ? `<div class="row small"><b>${esc(t("upd_cards"))}:</b> ${e.cards.map(cardChip).join(" ")}</div>` : ""}
          ${(e.pages || []).length ? `<div class="row small"><b>${esc(t("upd_see_on"))}:</b> ${e.pages.map((p) => `<a class="chip" href="#${p}">${esc(t("nav_" + p))}</a>`).join(" ")}</div>` : ""}
          ${media.length ? `<div class="thumbs" style="margin-top:8px">${media.map((m) => `<div class="thumb" data-media="${esc(m.id)}" data-cap="${esc(x.title)}">${m.kind === "img" ? `<img src="${m.src}" alt="">` : `<video src="${m.src}#t=4" preload="metadata" muted></video><div class="play">▶</div>`}</div>`).join("")}</div>` : ""}
          ${(e.proof || []).length ? `<div class="box ok" style="margin-top:10px"><h4>${esc(t("upd_proof"))}</h4><ul class="check">${e.proof.map((pf) => `<li><b>${esc(pf.value)}</b> — ${esc(Lstr(pf))}</li>`).join("")}</ul></div>` : ""}
          ${(e.files || []).length ? `<div class="small muted" style="margin-top:8px"><b>${esc(t("upd_files"))}:</b> ${e.files.map((f) => `<code>${esc(f)}</code>`).join(" ")}</div>` : ""}
          <div class="small muted" style="margin-top:6px"><b>${esc(t("upd_source"))}:</b> ${esc(src.file || "")} ${esc(src.heading || "")}${(src.commits || []).length ? " · git " + src.commits.map(esc).join(", ") : ""}</div>
        </div></div></article>`;
  }
  function renderActivity() {
    const ev = [];
    const byId = {}; cards().forEach((c) => (byId[c.id] = c));
    Object.entries(S.board.cards || {}).forEach(([id, st]) => {
      (st.history || []).forEach((h) => ev.push({ t: h.t, id, kind: "move", from: h.from, to: h.to }));
      (st.note_log || []).forEach((n) => ev.push({ t: n.t, id, kind: "note", from: n.from, to: n.to, text: st.note }));
      if (st.note && !(st.note_log || []).length) ev.push({ t: null, id, kind: "note", text: st.note });
    });
    (S.board.events || []).forEach((x) => ev.push({ t: x.t, kind: x.kind }));
    const timed = ev.filter((x) => x.t).sort((a, b) => b.t.localeCompare(a.t));
    const untimed = ev.filter((x) => !x.t);
    const week = Date.now() - 7 * 86400000;
    const done = cards().filter((c) => cstate(c).state === "done").length;
    const days = []; for (let i = 29; i >= 0; i--) { const d = new Date(Date.now() - i * 86400000); days.push(d.toISOString().slice(0, 10)); }
    const perDay = {}; timed.forEach((x) => { const d = x.t.slice(0, 10); (perDay[d] = perDay[d] || []).push(x); });
    const maxN = Math.max(1, ...days.map((d) => (perDay[d] || []).length));
    const item = (x) => { const c = byId[x.id]; const title = c ? L(c).title : ""; const when = x.t ? ftime(x.t) : "";
      if (x.kind === "reset") return `<li class="act"><span class="small muted">${esc(when)}</span> <b>${esc(t("act_reset"))}</b></li>`;
      const what = x.kind === "move" ? `<span class="sw" style="background:${SCOL[x.from]}"></span>${esc(t("state_" + x.from))} → <span class="sw" style="background:${SCOL[x.to]}"></span>${esc(t("state_" + x.to))}`
        : `${esc(t("act_note_edit"))}${x.from != null ? ` (${x.from} → ${x.to} ${esc(t("chars"))})` : ""}${x.text ? ` · <span class="muted">${esc(x.text.slice(0, 80))}${x.text.length > 80 ? "…" : ""}</span>` : ""}`;
      return `<li class="act"><span class="small muted">${esc(when)}</span> <a href="#board?card=${encodeURIComponent(x.id)}"><code>${esc(x.id)}</code> ${esc(title)}</a><div class="small">${what}</div></li>`; };
    if (!ev.length) return `<div class="panel muted">${esc(t("act_empty"))}</div>`;
    const groups = {}; timed.forEach((x) => { const d = x.t.slice(0, 10); (groups[d] = groups[d] || []).push(x); });
    return `<div class="grid g4">
        <div class="panel stat"><span class="big-number">${timed.filter((x) => x.kind === "move" && Date.parse(x.t) > week).length}</span><span class="muted">${esc(t("act_moves"))} · ${esc(t("act_week"))}</span></div>
        <div class="panel stat"><span class="big-number">${timed.filter((x) => x.kind === "note" && Date.parse(x.t) > week).length}</span><span class="muted">${esc(t("act_notes"))} · ${esc(t("act_week"))}</span></div>
        <div class="panel stat"><span class="big-number">${done}</span><span class="muted">${esc(t("act_done"))}</span></div>
        <div class="panel stat"><span style="font-weight:700">${esc(timed[0] ? ftime(timed[0].t) : t("empty"))}</span><span class="muted">${esc(t("act_last"))}</span></div></div>
      <div class="panel" style="margin-top:12px"><div class="small muted">${esc(t("act_30d"))}</div><div class="bars30">${days.map((d) => { const xs = perDay[d] || [];
        return `<div class="b30" title="${esc(fdate(d))}: ${xs.length}">${xs.map((x) => `<span style="height:${100 / maxN}%;background:${x.kind === "move" ? SCOL[x.to] : "var(--todo)"}"></span>`).join("")}</div>`; }).join("")}</div></div>
      <div class="panel" style="margin-top:12px">${Object.keys(groups).map((d) => `<h4>${esc(fdate(d))}</h4><ul class="acts">${groups[d].map(item).join("")}</ul>`).join("")}
        ${untimed.length ? `<h4>${esc(t("act_unknown"))}</h4><ul class="acts">${untimed.map(item).join("")}</ul>` : ""}</div>`;
  }
  function renderUpdates() {
    const all = updates();
    const m = location.hash.match(/[?&]v=(\d+)/); const want = m ? +m[1] : null;
    const last = all[all.length - 1];
    if (last) safeSet("seenUpdate", String(last.seq));
    const tm = location.hash.match(/[?&]tab=(log|activity)/); if (tm) S.updTab = tm[1];
    if (S.updTab === "activity") {
      return `<h1>${esc(t("upd_title"))}</h1>${updTabs()}${renderActivity()}`;
    }
    const vis = all.filter(updMatches).slice().reverse();
    const f = S.updF;
    return `<h1>${esc(t("upd_title"))}</h1>${updTabs()}
      <p class="small muted">${esc(t("upd_version_help"))}</p>
      ${timelineStrip(all)}
      <div class="toolbar" style="margin-top:12px">
        <input type="search" id="u-q" placeholder="${esc(t("search"))}" value="${esc(f.q)}">
        <div class="tabs" style="margin:0">${["", "release", "work", "site"].map((k) => `<button data-kind="${k}" class="${f.kind === k ? "on" : ""}">${esc(t(k ? "upd_kind_" + k : "upd_all"))}</button>`).join("")}</div>
        <label class="row small"><input type="checkbox" id="u-ret" ${f.retracted ? "checked" : ""}> ${esc(t("upd_only_retracted"))}</label>
        <label class="row small"><input type="checkbox" id="u-media" ${f.media ? "checked" : ""}> ${esc(t("upd_with_media"))}</label>
        <span class="spacer"></span><span class="small muted">${vis.length} / ${all.length}</span></div>
      <div class="entries">${vis.map((e, i) => entryHTML(e, S.updOpen[e.seq] != null ? S.updOpen[e.seq] : (want ? e.seq === want : (i === 0 && !f.q && !f.kind && !f.retracted && !f.media)))).join("") || `<p class="muted">${esc(t("no_results"))}</p>`}</div>`;
  }
  function updTabs() {
    return `<div class="tabs"><button data-utab="log" class="${S.updTab === "log" ? "on" : ""}">${esc(t("upd_tab_log"))}</button><button data-utab="activity" class="${S.updTab === "activity" ? "on" : ""}">${esc(t("upd_tab_activity"))}</button></div>`;
  }
  function bindUpdates() {
    const rerender = () => { const y = window.scrollY; $("#main").innerHTML = renderUpdates(); bindUpdates(); window.scrollTo(0, y); };
    $$("[data-utab]").forEach((b) => (b.onclick = () => { S.updTab = b.dataset.utab; rerender(); }));
    $$("[data-kind]").forEach((b) => (b.onclick = () => { S.updF.kind = b.dataset.kind; rerender(); }));
    const q = $("#u-q"); if (q) q.oninput = (e) => { S.updF.q = e.target.value; clearTimeout(bindUpdates._t); bindUpdates._t = setTimeout(() => { rerender(); const el = $("#u-q"); el.focus(); el.setSelectionRange(el.value.length, el.value.length); }, 250); };
    const r = $("#u-ret"); if (r) r.onchange = (e) => { S.updF.retracted = e.target.checked; rerender(); };
    const md = $("#u-media"); if (md) md.onchange = (e) => { S.updF.media = e.target.checked; rerender(); };
    $$(".etoggle").forEach((b) => (b.onclick = () => { const art = b.closest(".entry"); const open = !art.classList.contains("open");
      art.classList.toggle("open", open); S.updOpen[+b.dataset.seq] = open; b.textContent = t(open ? "upd_less" : "upd_more"); }));
    $$(".tl-dot").forEach((b) => (b.onclick = () => { const seq = +b.dataset.seq; S.updOpen[seq] = true; S.updF = { q: "", kind: "", retracted: false, media: false }; rerender();
      const el = $("#upd-" + seq); if (el) { el.scrollIntoView({ behavior: "smooth", block: "start" }); el.classList.add("flash"); setTimeout(() => el.classList.remove("flash"), 1500); } }));
    $$("#main .thumb").forEach((el) => (el.onclick = () => openMedia(el.dataset.media, el.dataset.cap)));
    const m = location.hash.match(/[?&]v=(\d+)/);
    if (m) { const el = $("#upd-" + m[1]); if (el) setTimeout(() => { el.scrollIntoView({ block: "start" }); el.classList.add("flash"); setTimeout(() => el.classList.remove("flash"), 1500); }, 50); }
    layoutTimeline();
    renderBadge();
  }

  // ------------------------------------------------------------ events
  function bind(page) {
    $$(".go-area").forEach((a) => (a.onclick = () => { S.filters = { q: "", area: a.dataset.area, verdict: "", prio: "", gate: false }; }));
    $$("[data-media]").forEach((el) => { if (!el.classList.contains("thumb")) el.onclick = () => openMedia(el.dataset.media, el.dataset.cap); });
    if (page === "flow") {
      $$(".tabs button").forEach((b) => (b.onclick = () => { S.flowTab = b.dataset.tab; S.flowSel = null; render(); }));
      $$("svg.diagram .node").forEach((g) => (g.onclick = () => { S.flowSel = g.dataset.key; const y = window.scrollY; render(); window.scrollTo(0, y); }));
      $$("[data-hud]").forEach((h) => (h.onclick = () => { S.hudSel = h.dataset.hud; const y = window.scrollY; render(); window.scrollTo(0, y); }));
    }
    if (page === "board") {
      const upd = (k, v) => { S.filters[k] = v; refreshBoard(); };
      $("#f-q").oninput = (e) => { S.filters.q = e.target.value; clearTimeout(bind._q); bind._q = setTimeout(() => { refreshBoard(); const el = $("#f-q"); el.focus(); el.setSelectionRange(el.value.length, el.value.length); }, 250); };
      $("#f-area").onchange = (e) => upd("area", e.target.value);
      $("#f-verdict").onchange = (e) => upd("verdict", e.target.value);
      $("#f-prio").onchange = (e) => upd("prio", e.target.value);
      $("#f-gate").onchange = (e) => upd("gate", e.target.checked);
      $("#b-export").onclick = () => {
        const blob = new Blob([JSON.stringify({ exported: new Date().toISOString(), board: S.board }, null, 1)], { type: "application/json" });
        const a = document.createElement("a"); a.href = URL.createObjectURL(blob); a.download = "grant-tracker-state.json"; a.click();
      };
      $("#b-reset").onclick = () => { if (confirm(t("reset_confirm"))) {
        S.board = { cards: {}, updated: null, events: (S.board.events || []).concat([{ t: new Date().toISOString(), kind: "reset" }]) };
        save(true); refreshBoard(); } };
      $$(".kcard").forEach((el) => {
        el.onclick = () => openCard(el.dataset.id);
        el.ondragstart = (e) => { e.dataTransfer.setData("text/plain", el.dataset.id); el.classList.add("dragging"); };
        el.ondragend = () => el.classList.remove("dragging");
      });
      $$(".col").forEach((col) => {
        col.ondragover = (e) => { e.preventDefault(); col.classList.add("drop"); };
        col.ondragleave = () => col.classList.remove("drop");
        col.ondrop = (e) => { e.preventDefault(); col.classList.remove("drop"); const id = e.dataTransfer.getData("text/plain");
          const c = cards().find((x) => x.id === id); if (c && cstate(c).state !== col.dataset.state) { setCard(c, { state: col.dataset.state }); refreshBoard(); } };
      });
      const m = location.hash.match(/card=([^&]+)/); if (m) openCard(decodeURIComponent(m[1]));
    }
    if (page === "updates") bindUpdates();
    if (page === "dashboard") bindDashboard();
    if (page === "glossary") {
      $("#g-q").oninput = (e) => { const q = e.target.value.toLowerCase(); $$("#gloss .gi").forEach((d) => (d.style.display = d.textContent.toLowerCase().includes(q) ? "" : "none")); };
    }
  }

  // ------------------------------------------------------------ boot
  $$("#lang button").forEach((b) => (b.onclick = () => { S.lang = b.dataset.lang; safeSet("lang", S.lang); render(); if ($("#drawer").classList.contains("open")) { const m = location.hash.match(/card=([^&]+)/); if (m) openCard(decodeURIComponent(m[1])); } }));
  $("#scrim").onclick = closeDrawer;
  $("#lightbox").onclick = (e) => { if (e.target.tagName !== "VIDEO") { $("#lightbox").classList.remove("open"); $("#lightbox-inner").innerHTML = ""; } };
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") { if ($("#lightbox").classList.contains("open")) { $("#lightbox").classList.remove("open"); $("#lightbox-inner").innerHTML = ""; } else closeDrawer(); } });
  window.addEventListener("hashchange", () => {
    if (location.hash.startsWith("#board?card=")) {
      // From another page (a card chip on Updates, a link): show the board
      // first; bind("board") then opens the card named in the hash.
      if (S.page !== "board") render();
      else { const m = location.hash.match(/card=([^&]+)/); if (m && !$("#drawer").classList.contains("open")) openCard(decodeURIComponent(m[1])); }
      return;
    }
    $("#drawer").classList.remove("open"); $("#scrim").classList.remove("open"); render();
  });

  // The server answers /api/content with an ETag and a 304 to a matching If-None-Match. Until
  // 2026-10-21 a reload compared only the JSON's LENGTH, so a same-length edit (0.838 -> 0.844)
  // never showed until a manual reload, and every return to the tab re-downloaded 1.2 MB.
  async function fetchContent() {
    const r = await fetch("/api/content", { headers: S.contentEtag ? { "If-None-Match": S.contentEtag } : {} });
    if (r.status === 304) return null;
    if (!r.ok) throw new Error(r.status);
    S.contentEtag = r.headers.get("ETag");
    return r.json();
  }
  async function reloadContent(manual) {
    try {
      if (manual) S.contentEtag = null;
      const c = await fetchContent();
      if (c && (manual || JSON.stringify(c) !== JSON.stringify(S.content || {}))) {
        S.content = c; const y = window.scrollY; render(); window.scrollTo(0, y);
      }
      if (manual) { await loadDash(true); toast(t("reloaded")); }
    } catch (e) { if (manual) toast(t("load_failed"), true); }
  }
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState !== "visible" || !S.content) return;
    reloadContent(false);
    if (route() === "dashboard") { loadDash(false); loadLive(); }
  });
  $("#reload").onclick = () => reloadContent(true);

  async function boot() {
    renderNav();
    try {
      const [c, s] = await Promise.all([fetchContent(), fetch("/api/state").then((r) => r.json()).catch(() => null), loadDash(false, true), loadLive(true)]);
      S.content = c;
      const local = safeGet("board");
      S.board = (s && s.cards) ? s : (local ? JSON.parse(local) : { cards: {}, updated: null });
    } catch (e) {
      $("#main").innerHTML = `<p class="pad">${esc(t("load_failed"))}</p>`; return;
    }
    render();
  }
  boot();
})();
