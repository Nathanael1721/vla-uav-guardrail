/*
 * nathan-deck design system, shared by the September 2026 decks.
 *
 * Tokens, badge(), heading(), card() and darkBase() are copied from the
 * nathan-deck skill's references/build_template.js with their geometry
 * unchanged - the skill is explicit that the token values and helper geometry
 * ARE the design system. table() follows the template's table archetype, and
 * img() rasterises an SVG or reads a JPEG/PNG so every visual is a real
 * artefact from the repository.
 */
const fs = require("fs");
const path = require("path");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");

const T = {
  TEAL: "249DB2", TEAL_DK: "1A7484", TEAL_DEEP: "12545F",
  TEAL_TINT: "EAF6F8", TEAL_TINT2: "D6EDF1",
  CHARCOAL: "444441", BLACK: "1A1A1A", INK: "2B2B2B",
  GREY: "6B7280", LGREY: "9AA3AD", WHITE: "FFFFFF", LINE: "E3E8EC",
  FF: "Poppins",
};

async function iconPng(Icon, color = "#" + T.TEAL, size = 256) {
  const svg = ReactDOMServer.renderToStaticMarkup(
    React.createElement(Icon, { color, size: String(size) }));
  const buf = await sharp(Buffer.from(svg)).png().toBuffer();
  return "image/png;base64," + buf.toString("base64");
}

/* An image from the repository, as a data URI plus its aspect ratio. SVGs are
 * rasterised at `density` so diagram text stays sharp when projected. */
async function img(file, { density = 220, maxW = 2400 } = {}) {
  let pipe = sharp(fs.readFileSync(file), file.endsWith(".svg") ? { density } : {});
  const meta = await pipe.metadata();
  if (meta.width > maxW) pipe = pipe.resize({ width: maxW });
  const buf = await pipe.png().toBuffer();
  const m2 = await sharp(buf).metadata();
  return { data: "image/png;base64," + buf.toString("base64"), aspect: m2.width / m2.height };
}

function helpers(pres) {
  const { TEAL, TEAL_DK, TEAL_TINT, BLACK, INK, GREY, LGREY, WHITE, LINE, FF } = T;

  function badge(s, n) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: 9.3, y: 5.18, w: 0.5, h: 0.32,
      fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.06,
    });
    s.addText(String(n).padStart(2, "0"), {
      x: 9.3, y: 5.18, w: 0.5, h: 0.32,
      fontSize: 9, color: WHITE, fontFace: FF, bold: true, align: "center", valign: "middle",
    });
  }

  function heading(s, eyebrow, title, x = 0.55, y = 0.45) {
    s.addText(eyebrow.toUpperCase(), {
      x, y, w: 8.5, h: 0.28, fontSize: 10, color: TEAL, fontFace: FF, bold: true, charSpacing: 2,
    });
    s.addText(title, { x, y: y + 0.3, w: 8.9, h: 0.6, fontSize: 26, color: BLACK, fontFace: FF, bold: true });
    s.addShape(pres.shapes.LINE, { x, y: y + 0.98, w: 0.9, h: 0, line: { color: TEAL, width: 2.5 } });
  }

  function card(s, x, y, w, h, iconData, titleTxt, bodyTxt, bodySize = 11) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x, y, w, h, fill: { color: WHITE }, line: { color: LINE, width: 1 }, rectRadius: 0.08,
      shadow: { type: "outer", blur: 6, offset: 2, angle: 90, color: "D9DEE3", opacity: 0.4 },
    });
    let ty;
    if (iconData) {
      s.addImage({ data: iconData, x: x + 0.22, y: y + 0.22, w: 0.36, h: 0.36 });
      s.addText(titleTxt, { x: x + 0.68, y: y + 0.2, w: w - 0.9, h: 0.4, fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle" });
      ty = y + 0.72;
    } else {
      s.addText(titleTxt, { x: x + 0.24, y: y + 0.2, w: w - 0.48, h: 0.4, fontSize: 13, color: INK, fontFace: FF, bold: true });
      ty = y + 0.66;
    }
    if (bodyTxt) {
      s.addText(bodyTxt, {
        x: x + 0.24, y: ty, w: w - 0.48, h: Math.max(0.3, h - (ty - y) - 0.2),
        fontSize: bodySize, color: GREY, fontFace: FF, lineSpacingMultiple: 1.25, valign: "top",
      });
    }
  }

  function darkBase(s, eyebrow) {
    s.background = { color: TEAL_DK };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 0.09, h: 5.625, fill: { color: TEAL }, line: { color: TEAL } });
    if (eyebrow) {
      s.addText(eyebrow.toUpperCase(), { x: 0.55, y: 0.5, w: 9, h: 0.3, fontSize: 10, color: TEAL_TINT, fontFace: FF, bold: true, charSpacing: 3 });
    }
  }

  /* Table archetype: teal header, TEAL_TINT zebra, LINE hairlines.
   * `emph` = [[row, col], ...] cells drawn bold in teal (the "winner" cell). */
  function table(s, rows, { x = 0.55, y = 1.85, w = 8.9, colFrac, rh = 0.42, fontSize = 11, emph = [], align } = {}) {
    const cols = rows[0].length;
    const fr = colFrac || Array(cols).fill(1 / cols);
    const colW = fr.map((f) => w * f);
    const isEmph = (r, c) => emph.some(([er, ec]) => er === r && ec === c);
    rows.forEach((r, ri) => {
      let cx = x;
      r.forEach((cell, ci) => {
        const header = ri === 0;
        const zebra = !header && ri % 2 === 0;
        s.addShape(pres.shapes.RECTANGLE, {
          x: cx, y: y + ri * rh, w: colW[ci], h: rh,
          fill: { color: header ? TEAL : zebra ? TEAL_TINT : WHITE }, line: { color: LINE, width: 1 },
        });
        const e = !header && isEmph(ri, ci);
        s.addText(String(cell), {
          x: cx + 0.1, y: y + ri * rh, w: colW[ci] - 0.2, h: rh,
          fontSize: header ? fontSize + 0.5 : fontSize, color: header ? WHITE : e ? TEAL : INK,
          bold: header || e, fontFace: FF,
          align: align ? align[ci] : ci === 0 ? "left" : "center", valign: "middle",
        });
        cx += colW[ci];
      });
    });
    return y + rows.length * rh;
  }

  /* A small caption under a figure or table: the artefact it came from. */
  function source(s, txt, y = 4.98) {
    s.addText(txt, { x: 0.55, y, w: 8.6, h: 0.26, fontSize: 8.5, color: LGREY, fontFace: FF, italic: true });
  }

  /* A big number with a label, for KPI rows. */
  function stat(s, x, y, w, value, label, color = TEAL, size = 30) {
    s.addText(value, { x, y, w, h: 0.62, fontSize: size, color, fontFace: FF, bold: true });
    s.addText(label, { x, y: y + 0.62, w, h: 0.5, fontSize: 10.5, color: GREY, fontFace: FF, lineSpacingMultiple: 1.15, valign: "top" });
  }

  /* An image fitted inside a box, centred, with a hairline frame. */
  function figure(s, im, bx, by, bw, bh) {
    let w = bw, h = bw / im.aspect;
    if (h > bh) { h = bh; w = bh * im.aspect; }
    const x = bx + (bw - w) / 2, y = by + (bh - h) / 2;
    s.addImage({ data: im.data, x, y, w, h });
    s.addShape(pres.shapes.RECTANGLE, { x, y, w, h, fill: { type: "none" }, line: { color: LINE, width: 1 } });
    return { x, y, w, h };
  }

  return { badge, heading, card, darkBase, table, source, stat, figure };
}

/* pptxgenjs 4.0.1 writes a package PowerPoint refuses to open. The repo's own
 * repair and audit scripts fix and then check it; a failed audit fails the build. */
function repairAndAudit(pptx, repo, python) {
  const { execFileSync } = require("child_process");
  execFileSync(python, [path.join(repo, "tools/fix_pptx_package.py"), pptx], { stdio: "inherit" });
  execFileSync(python, [path.join(repo, "tools/audit_pptx.py"), pptx], { stdio: "inherit" });
}

module.exports = { T, iconPng, img, helpers, repairAndAudit };
