#!/usr/bin/env node
/*
 * The Gauge Studio page (data/www/index.html) checked against the firmware it
 * configures. Run by test_host/run.py as the "studio_page" scenario, or alone:
 *
 *     node test_host/studio_tests.js
 *
 * The page offers what the layout can hold, and nothing the display ignores:
 *  - every setting of the layout's global, network, watchdog and warnings
 *    blocks the firmware reads is one row of the page's settings block, once,
 *    with the firmware's own default; no row names a key it does not read;
 *  - every key the screen editor can write is read by this board's
 *    UIBuilder.cpp, in the code that builds that screen type, and the
 *    editor's number defaults are the firmware's;
 *  - every page element the script looks up exists, and every settings
 *    section has its place on the page;
 *  - the keys the page drops from a layout are read nowhere any more;
 *  - the script between @shared-begin and @shared-end is the same in both
 *    screens' pages.
 * What it cannot see: a key the firmware reads but never draws (min and max
 * of a cell are parsed for every reading) - that is the editor's own tables.
 *
 * This file is identical in both display projects.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.dirname(__dirname);                       // this screen's project
const BOARD = /cluster/.test(path.basename(ROOT)) ? 'cluster' : 'round';
const OTHER = path.join(path.dirname(ROOT), BOARD === 'round' ? 'screen2-cluster' : 'screen1-round');
const read = (...p) => fs.readFileSync(path.join(...p), 'utf8');

let failures = 0, checks = 0;
function ok(cond, what) {
  checks++;
  if (!cond) { failures++; console.log('FAIL ' + what); }
}
function section(t) { console.log('-- ' + t); }

const page = read(ROOT, 'data', 'www', 'index.html');
const script = (page.match(/<script>([\s\S]*)<\/script>/) || [])[1] || '';
const between = (s, a, b) => {
  const i = s.indexOf(a), j = s.indexOf(b, i + 1);
  return i < 0 || j < 0 ? null : s.slice(i, j);
};

/* ── C++ helpers ───────────────────────────────────────────────────────── */
/** Source with comments blanked (string literals kept). */
function stripComments(src) {
  let out = '', i = 0;
  while (i < src.length) {
    const c = src[i], n = src[i + 1];
    if (c === '/' && n === '/') { while (i < src.length && src[i] !== '\n') i++; continue; }
    if (c === '/' && n === '*') { const e = src.indexOf('*/', i + 2); i = e < 0 ? src.length : e + 2; out += ' '; continue; }
    if (c === '"' || c === "'") {
      let j = i + 1;
      while (j < src.length && src[j] !== c) j += src[j] === '\\' ? 2 : 1;
      out += src.slice(i, j + 1); i = j + 1; continue;
    }
    out += c; i++;
  }
  return out;
}
/** The body of the first function whose header matches re (braces matched,
 *  skipping strings). */
function body(src, re) {
  const m = re.exec(src);
  if (!m) return null;
  let i = src.indexOf('{', m.index + m[0].length - 1), depth = 0;
  const start = i;
  for (; i < src.length; i++) {
    const c = src[i];
    if (c === '"' || c === "'") {
      let j = i + 1;
      while (j < src.length && src[j] !== c) j += src[j] === '\\' ? 2 : 1;
      i = j; continue;
    }
    if (c === '{') depth++;
    else if (c === '}' && --depth === 0) return src.slice(start, i + 1);
  }
  return null;
}
const fn = (src, name) => body(src, new RegExp('^[\\w:<>&\\* ]*\\b' + name.replace(/::/g, '::') + '\\s*\\(', 'm'));

/** #define NAME value from the board's include/*.h. */
const MACROS = {};
for (const f of fs.readdirSync(path.join(ROOT, 'include')).filter(f => f.endsWith('.h'))) {
  for (const m of stripComments(read(ROOT, 'include', f)).matchAll(/^\s*#define\s+(\w+)\s+(.+?)\s*$/gm))
    if (!(m[1] in MACROS)) MACROS[m[1]] = m[2];
}
/** A firmware default expression as a value, or undefined when it is not a
 *  plain one. */
function cValue(expr) {
  let e = String(expr).trim();
  for (let k = 0; k < 4; k++) {
    e = e.replace(/^\((?:int|bool|float|uint8_t|uint32_t|const char \*)\)\s*/, '').trim();
    if (/^\(.*\)$/.test(e) && !/\?/.test(e)) e = e.slice(1, -1).trim();
    if (/^\w+$/.test(e) && MACROS[e] !== undefined && !/^\d/.test(e)) { e = MACROS[e]; continue; }
    break;
  }
  // A default inside a call - strcmp(w["k"] | "bottom", "top") - ends at the comma.
  const str = e.match(/^"((?:[^"\\]|\\.)*)"\s*(?:,.*)?$/);
  if (str) return JSON.parse('"' + str[1] + '"');
  e = e.replace(/\s*,.*$/, '');
  if (e === 'true' || e === 'false') return e === 'true';
  const n = e.replace(/[uUlLfF]+$/, '');
  if (/^-?(\d+\.?\d*|\.\d+)([eE]-?\d+)?$/.test(n)) return Number(n);
  return undefined;
}
/** Does the default for key in this source equal v? A key read in several
 *  places must have the same default in each. */
function defaultsOf(src, obj, key) {
  const out = [];
  const re = new RegExp(obj + '\\["' + key + '"\\]\\s*\\|\\s*([^;]+?)\\s*(?:;|\\)\\s*;|,\\s*\\d|\\?)', 'g');
  for (const m of src.matchAll(re)) out.push(m[1]);
  return out;
}

/* ── the page's core, loaded as the browser would for this board ───────── */
section(`studio page for the ${BOARD} board`);
const core = between(script, '/* @core-begin', '/* @core-end */');
ok(core, 'the page has its @core-begin / @core-end block');
const ctx = {};
vm.createContext(ctx);
vm.runInContext(`'use strict';const BOARD=${JSON.stringify(BOARD)};\n${core}\n` +
  'this.out={SETTINGS,SET,TYPES,TYPES_ALL,F,DASH_PARTS,DASH_BASE,BG,BEZEL,GONE,cleanLayout,bezelDefault};', ctx);
const P = ctx.out;

const shared = s => between(s, '/* @shared-begin', '/* @shared-end');
const otherPage = fs.existsSync(path.join(OTHER, 'data', 'www', 'index.html')) ? read(OTHER, 'data', 'www', 'index.html') : null;
ok(shared(page), 'the page has its @shared-begin / @shared-end block');
if (otherPage) ok(shared(page) === shared(otherPage),
  'the shared part of the script is the same in both screens\' pages - change both');
ok(new RegExp(`const BOARD = '${BOARD}';`).test(script), `the page says BOARD = '${BOARD}'`);

/* ── settings ──────────────────────────────────────────────────────────── */
section('settings block against ConfigManager.h, parseWatchdog, Telltales::configure');
const cfgH = stripComments(read(ROOT, 'src', 'config', 'ConfigManager.h'));
const ui = stripComments(read(ROOT, 'src', 'ui', 'UIBuilder.cpp'));
const tt = stripComments(read(ROOT, 'src', 'ui', 'Telltales.cpp'));
const wdBody = fn(ui, 'UIBuilder::parseWatchdog');
const ttBody = fn(tt, 'Telltales::configure');
ok(wdBody && ttBody, 'found UIBuilder::parseWatchdog and Telltales::configure');

/* What the firmware reads, with its default expressions. */
const fw = new Map();                         // "block.key" -> [default exprs]
const add = (k, d) => { if (!fw.has(k)) fw.set(k, []); if (d !== undefined) fw.get(k).push(d); };
for (const m of cfgH.matchAll(/_layout\["(global|network)"\]\["(\w+)"\](\s*\|\s*([^;]+?)\s*;)?/g))
  add(m[1] + '.' + m[2], m[4]);
for (const m of wdBody.matchAll(/\bwd\["(\w+)"\](\s*\|\s*([^;)]+?)\s*[;)])?/g)) add('watchdog.' + m[1], m[3]);
for (const m of ttBody.matchAll(/\bw\["(\w+)"\](\s*\|\s*(\([^;]*?\)|[^;)]+?)\s*[;)])?/g)) add('warnings.' + m[1], m[3]);
for (const m of ttBody.matchAll(/msFrom\(\s*w\["(\w+)"\]\s*,\s*([\d.]+)/g)) add('warnings.' + m[1], m[2]);
/* Read by the firmware, written elsewhere: the device's touch calibration,
   the guard list (the Watchdog tab's cards), the lamps (the Lamps tab's). */
const NOT_ROWS = new Set(['global.touch_cal', 'watchdog.items', 'warnings.lamps']);
const strip = /\bkStripAlwaysDefault\b/;
const lcd = (n) => cValue(n);
function fwDefault(key) {
  const exprs = fw.get(key) || [];
  const vals = exprs.map(e => {
    if (strip.test(e)) {                      // (kStripAlwaysDefault ? "always" : "peek")
      const always = lcd('LCD_WIDTH') !== lcd('LCD_HEIGHT');
      const m = e.match(/\?\s*"(\w+)"\s*:\s*"(\w+)"/);
      return m ? (always ? m[1] : m[2]) : undefined;
    }
    return cValue(e);
  });
  return vals;
}

const rows = P.SET;
const seen = new Map();
for (const r of rows) seen.set(r.k, (seen.get(r.k) || 0) + 1);
for (const [k, n] of seen) ok(n === 1, `setting ${k} is one row, not ${n}`);
for (const k of fw.keys()) {
  if (NOT_ROWS.has(k)) continue;
  ok(seen.has(k), `the firmware reads ${k}: it needs a row in the page's SETTINGS`);
}
for (const r of rows) {
  ok(fw.has(r.k), `row ${r.k}: the ${BOARD} firmware does not read it`);
  if (!fw.has(r.k)) continue;
  const d = fwDefault(r.k).filter(v => v !== undefined);
  ok(d.length > 0, `row ${r.k}: no default found in the firmware to compare with`);
  for (const v of d)
    ok(typeof v === 'number' ? Math.abs(v - r.def) < 1e-9 : v === r.def,
       `row ${r.k}: the page's default ${JSON.stringify(r.def)} is not the firmware's ${JSON.stringify(v)}`);
  const kinds = { boolean: ['check'], number: ['num', 'gain'], string: ['text', 'select', 'color'] };
  ok((kinds[typeof r.def] || []).includes(r.kind), `row ${r.k}: a ${typeof r.def} shown as "${r.kind}"`);
  if (r.kind === 'select') ok(r.opts.some(o => o[0] === r.def), `row ${r.k}: its default is not one of its options`);
}
/* Each section is shown somewhere: a #set-<section> on the page, drawn by
   renderSet from the tab it is on. */
for (const sec of new Set(rows.map(r => r.sec))) {
  ok(new RegExp(`id="set-${sec}"`).test(page), `section "${sec}" has no id="set-${sec}" on the page`);
  ok(new RegExp(`renderSet\\('${sec}'\\)|\\[[^\\]]*'${sec}'[^\\]]*\\]\\.forEach\\(renderSet\\)`).test(script),
     `section "${sec}" is never drawn (renderSet)`);
}

/* The example layout's settings are all known. */
const example = JSON.parse(read(ROOT, 'data', 'layout.json'));
for (const blk of ['global', 'network', 'watchdog', 'warnings'])
  for (const k of Object.keys(example[blk] || {}))
    if (!k.startsWith('_'))
      ok(seen.has(blk + '.' + k) || NOT_ROWS.has(blk + '.' + k),
         `data/layout.json has ${blk}.${k}, which is neither a row nor a known exception`);

/* ── the screen editor ─────────────────────────────────────────────────── */
section('screen editor against UIBuilder.cpp');
const bodies = {};
const get = name => {
  if (!(name in bodies)) bodies[name] = /::/.test(name) ? fn(ui, name)
    : body(ui, new RegExp('^static [\\w<>&\\* ]*\\b' + name + '\\s*\\(', 'm'));
  return bodies[name] || '';
};
const literal = (src, k) => src.includes('"' + k + '"');
/* The code that builds each screen type. */
const BUILD = {
  arc_gauge: ['buildArcGauge'], meter_gauge: ['buildMeterGauge'], quad: ['buildQuad'],
  text_list: ['buildTextList'], bars: ['buildBars'], cluster: ['buildCluster'], chart: ['buildChart'],
  dash: ['buildDash', 'dashPart'], gmeter: ['buildGMeter'], perf: ['buildPerf'], diag: ['buildDiag'],
};
const COMMON = ['UIBuilder::buildScreen', 'UIBuilder::applyBackground', 'UIBuilder::parseCommon',
                'UIBuilder::parseThreshold', 'screenTitle'];
const dispatched = new Set([...get('UIBuilder::buildScreen').matchAll(/strcmp\(type,\s*"(\w+)"\)/g)].map(m => m[1]));
dispatched.delete('split');                   // the old name of quad, still accepted
ok([...dispatched].sort().join() === Object.keys(P.TYPES).sort().join(),
   `the editor's types (${Object.keys(P.TYPES).sort()}) are the ones buildScreen builds (${[...dispatched].sort()})`);

/** Every key a spec can write, as dotted paths ("/" = the screen's own). */
function specKeys(spec, pre) {
  const out = [];
  const fields = (typeof spec.f === 'string' ? spec.f.split(' ').filter(Boolean).map(n => P.F[n]) : (spec.f || []))
    .concat(spec.x || []);
  for (const f of fields) out.push({ key: f[0][0] === '/' ? f[0].slice(1) : pre + f[0], f });
  if (spec.thr) for (const t of ['warning', 'critical'])
    for (const k of ['value', 'hysteresis', 'direction', 'color'])
      out.push({ key: `${pre}thresholds.${t}.${k}` });
  return out;
}
for (const [type, sp] of Object.entries(P.TYPES)) {
  const build = (BUILD[type] || []).map(n => 'UIBuilder::' + n);
  ok(build.every(n => get(n)), `${type}: found ${build.join(', ')}`);
  const src = build.concat(COMMON).map(get).join('\n');
  const keys = [];
  if (sp.title) keys.push({ key: 'label' });
  const lists = [];
  if (sp.dash) for (const p of P.DASH_PARTS) {
    keys.push({ key: p.k });
    if (p.max) lists.push(p); else keys.push(...specKeys(p, p.flag ? '' : p.k + '.'));
  }
  for (const p of sp.parts || []) {
    keys.push({ key: p.k });
    if (p.max) lists.push(p); else keys.push(...specKeys(p, p.k + '.'));
  }
  if (sp.list) lists.push(sp.list);
  if (sp.series) { lists.push(sp.series); keys.push(...specKeys(sp.single, '')); keys.push(...specKeys({ x: sp.x }, '')); }
  else if (!sp.dash && !sp.parts && !sp.list) keys.push(...specKeys(sp, ''));
  for (const L of lists) { keys.push({ key: L.k }); keys.push(...specKeys(L, L.k + '.0.')); }
  for (const { key } of keys)
    for (const seg of key.split('.').filter(s => !/^\d+$/.test(s)))
      ok(literal(src, seg), `${type}: the editor writes "${key}", but "${seg}" is not read in ${build.join('/')} or the common code`);
  /* Number defaults: the first "key" | default in the type's own code. */
  if (!sp.dash) for (const { key, f } of keys) {
    if (!f || f[2] !== 'num' || typeof f[3] !== 'number') continue;
    const leaf = key.split('.').pop();
    const own = build.map(get).join('\n');
    const d = [own, src].map(s => defaultsOf(s, '\\w+', leaf)).find(a => a.length);
    const v = d && cValue(d[0]);
    if (v !== undefined) ok(Math.abs(v - f[3]) < 1e-9,
      `${type}: "${key}" defaults to ${f[3]} in the editor, ${v} in the firmware`);
  }
  /* A list never holds more than the firmware draws. */
  for (const L of lists) {
    const own = build.map(get).join('\n');
    const m = own.match(new RegExp(`\\(size_t\\)\\s*${L.k}\\.size\\(\\)\\s*,\\s*\\(?(?:size_t\\)\\s*)?(\\w+)`)) ||
              own.match(new RegExp(`${L.k}\\.size\\(\\)\\s*,\\s*\\(?(?:size_t\\)\\s*)?(\\w+)`));
    if (m) {
      let cap = Number(m[1]);
      if (isNaN(cap)) {
        const h = stripComments(read(ROOT, 'src', 'ui', 'UIBuilder.h'));
        const c = h.match(new RegExp(m[1] + '\\s*=\\s*(\\d+)'));
        cap = c ? Number(c[1]) : NaN;
      }
      if (!isNaN(cap)) ok(L.max === cap, `${type}: the editor allows ${L.max} ${L.k}, the firmware draws ${cap}`);
    }
  }
}
/* Every screen's background, and the round board's bezel. */
{
  const src = COMMON.map(get).join('\n');
  for (const f of P.BG) for (const seg of f[0].split('.'))
    ok(literal(src, seg), `background "${f[0]}": "${seg}" is not read by applyBackground`);
  ok(literal(get('UIBuilder::buildScreen'), 'show_popup') && literal(get('UIBuilder::buildScreen'), 'show_telltales'),
     'buildScreen reads show_popup and show_telltales');
  ok((P.BEZEL.length > 0) === literal(get('UIBuilder::buildScreen'), 'dial_face'),
     'the screen-wide bezel switch is offered exactly where buildScreen reads dial_face');
  for (const f of P.BEZEL) ok(literal(get('UIBuilder::buildScreen'), f[0]), `bezel "${f[0]}" is read by buildScreen`);
  const listLike = get('UIBuilder::buildScreen').match(/listLike\s*=([^;]+);/);
  if (listLike) for (const t of Object.keys(P.TYPES))
    ok(P.bezelDefault(t) === !listLike[1].includes('"' + t + '"'),
       `the bezel's default for ${t} is the firmware's`);
}

/* ── page elements ─────────────────────────────────────────────────────── */
section('page elements');
const idsOf = p => new Set([...p.matchAll(/\bid="([\w-]+)"/g)].map(m => m[1]));
const ids = idsOf(page), otherIds = otherPage ? idsOf(otherPage) : new Set();
/* The shared script carries the other board's few extras too (the cluster's
   panel colour note): a lookup there must find its element on one of the
   pages, and tolerate its absence; the board's own code, on this page. */
const sharedPart = shared(page) || '';
for (const m of script.matchAll(/\$\('#([\w-]+)'\)|getElementById\('([\w-]+)'\)/g)) {
  const id = m[1] || m[2], inShared = sharedPart.includes(m[0]);
  ok(ids.has(id) || (inShared && otherIds.has(id)), `the script looks up #${id}, which the page does not have`);
}
for (const t of [...page.matchAll(/data-tab="(\w+)"/g)].map(m => m[1]))
  ok(ids.has('tab-' + t), `tab ${t} has no section id="tab-${t}"`);

/* ── keys dropped from a layout ────────────────────────────────────────── */
section('keys the page drops');
const src = (dir) => fs.readdirSync(dir, { withFileTypes: true }).flatMap(d =>
  d.isDirectory() ? src(path.join(dir, d.name)) : /\.(cpp|h)$/.test(d.name) ? [read(dir, d.name)] : []);
const firmware = src(path.join(ROOT, 'src')).map(stripComments).join('\n');
for (const [blk, keys] of Object.entries(P.GONE))
  for (const k of keys)
    ok(k.startsWith('_') || !firmware.includes('"' + k + '"'), `${blk}.${k} is dropped by the page but still read by the firmware`);
{
  const l = JSON.parse(JSON.stringify(example));
  l.global = l.global || {};
  for (const k of P.GONE.global) l.global[k] = 1;
  const n = P.cleanLayout(l);
  ok(n === P.GONE.global.length && JSON.stringify(l) === JSON.stringify(example),
     'cleanLayout drops exactly the dropped keys');
}

console.log(`${checks - failures}/${checks} checks passed` + (failures ? `, ${failures} FAILED` : ''));
process.exit(failures ? 1 : 0);
