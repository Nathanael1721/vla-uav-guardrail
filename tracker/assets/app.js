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
    flow_system: ["img", "flowchart_system.svg"],
    v_nfz_hud: ["video", "citylife_redcar_nfz_policy_hud.mp4"], v_identity: ["video", "citylife_redcar_identity.mp4"],
    v_before_after: ["video", "progress_0930_before_after.mp4"], v_demo_nfz: ["video", "demo_nfz.mp4"],
    v_retarget: ["video", "retarget_smooth.mp4"], v_demo_follow: ["video", "demo_follow.mp4"],
    v_ped: ["video", "citylife_ped_final.mp4"],
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
  function mediaOf(id) {
    if (id && id.includes("/")) { const kind = id.startsWith("video/") ? "video" : "img"; return { id, kind, src: `/media/${id}` }; }
    const m = MEDIA[id]; return m ? { id, kind: m[0], src: `/media/${m[0]}/${m[1]}` } : null;
  }
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
  const PAGES = { overview: renderOverview, flow: renderFlow, grant: renderGrant, board: renderBoard, evidence: renderEvidence, glossary: renderGlossary, updates: renderUpdates };
  function route() { const h = (location.hash || "#overview").slice(1).split("?")[0]; return PAGES[h] ? h : "overview"; }
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

  async function reloadContent(manual) {
    try {
      const c = await fetch("/api/content").then((r) => r.json());
      if (JSON.stringify(c).length !== JSON.stringify(S.content || {}).length || manual) {
        S.content = c; const y = window.scrollY; render(); window.scrollTo(0, y);
        if (manual) toast(t("reloaded"));
      }
    } catch (e) { if (manual) toast(t("load_failed"), true); }
  }
  document.addEventListener("visibilitychange", () => { if (document.visibilityState === "visible" && S.content) reloadContent(false); });
  $("#reload").onclick = () => reloadContent(true);

  async function boot() {
    renderNav();
    try {
      const [c, s] = await Promise.all([fetch("/api/content").then((r) => r.json()), fetch("/api/state").then((r) => r.json()).catch(() => null)]);
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
