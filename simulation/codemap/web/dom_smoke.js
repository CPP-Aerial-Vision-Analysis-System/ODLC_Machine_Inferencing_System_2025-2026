/*
 * simulation/codemap/web/dom_smoke.js -- PURPOSE: a smoke test of the explorer page's screen code without a
 * browser. It builds a tiny fake DOM (just enough of document/element/event/timer for index.html), loads the
 * page's markup and BOTH of its scripts into node, and then drives the page like a person would:
 *   1. ARTIFACT mode (window.ASTRA_CODEMAP + window.ASTRA_TRACE = the sample files): the page must draw the
 *      top level, start the replay, make blocks glow green, count calls, mark running programs, fill the
 *      "just ran" log, react to a click (panel), a double-click (open a system), search + Enter (jump to a
 *      function), arrow keys, Backspace (up), Overview, "Show tests", the scrubber and the legend, and it must
 *      NEVER call fetch() or EventSource (an artifact has no network).
 *   2. LOCALHOST mode: /codemap.json answers 404, so the page must fall back to sample_codemap.json, show the
 *      SAMPLE DATA chip, connect to /events (a fake EventSource) and glow on a "calls" event.
 * It cannot show what the page LOOKS like (colours, overlaps, fonts): that still needs a real browser.
 *
 *   3. (optional) --codemap PATH: boot any code map, for example the real codemap.json, open every system, a
 *      package and a file inside it, search + Enter and the overview, with no errors and no network.
 *   4. (optional) --artifact PATH: take the data embedded in a built dist/astra_code_map.html (the real code map
 *      and the real recorded run), check every recorded id is a block of the map, and replay it at four places
 *      of the run: blocks must glow and count, with no errors and no network.
 *                               node simulation/codemap/web/dom_smoke.js --artifact simulation/codemap/dist/astra_code_map.html
 * Run (node 18+, no packages):  node simulation/codemap/web/dom_smoke.js [--dagre path/to/dagre.min.js]
 *                               node simulation/codemap/web/dom_smoke.js --codemap simulation/codemap/codemap.json
 * Exit code 0 = every check passed, 1 = a check failed or the page threw an error.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const here = __dirname;
const html = fs.readFileSync(path.join(here, 'index.html'), 'utf8');
const sample = JSON.parse(fs.readFileSync(path.join(here, 'sample_codemap.json'), 'utf8'));
const recordingText = fs.readFileSync(path.join(here, 'sample_recording.jsonl'), 'utf8');
const dagreArg = process.argv.indexOf('--dagre');
const dagre = dagreArg >= 0 ? require(path.resolve(process.argv[dagreArg + 1])) : null;

let failures = 0, passes = 0;
function check(cond, msg) { if (cond) passes++; else { failures++; console.log('FAIL ' + msg); } }

/* ---------------- a tiny fake DOM ---------------- */
const SVGNS = 'http://www.w3.org/2000/svg';
const VOID = new Set(['input', 'meta', 'link', 'br', 'img', 'hr', 'source']);

function makeDocument() {
  const doc = { listeners: {}, activeElement: null, kbd: false, nodeType: 9, parentNode: null };
  class N {
    constructor() { this.parentNode = null; this.childNodes = []; this.ownerDocument = doc; }
    get firstChild() { return this.childNodes[0] || null; }
    get lastChild() { return this.childNodes[this.childNodes.length - 1] || null; }
    get previousSibling() { const p = this.parentNode; if (!p || !p.childNodes) return null; const i = p.childNodes.indexOf(this); return i > 0 ? p.childNodes[i - 1] : null; }
    appendChild(c) { if (c.parentNode && c.parentNode.removeChild) c.parentNode.removeChild(c); c.parentNode = this; this.childNodes.push(c); return c; }
    removeChild(c) { const i = this.childNodes.indexOf(c); if (i >= 0) this.childNodes.splice(i, 1); c.parentNode = null; return c; }
    replaceWith(n) { const p = this.parentNode; if (!p) return; if (n.parentNode) n.parentNode.removeChild(n); const i = p.childNodes.indexOf(this); p.childNodes[i] = n; n.parentNode = p; this.parentNode = null; }
    get textContent() { return this.childNodes.map((c) => c.textContent).join(''); }
    set textContent(v) { for (const c of this.childNodes) c.parentNode = null; this.childNodes = []; if (v != null && v !== '') this.appendChild(new T(String(v))); }
  }
  class T extends N { constructor(t) { super(); this.nodeType = 3; this.data = t; } get textContent() { return this.data; } set textContent(v) { this.data = String(v); } }
  class E extends N {
    constructor(tag, ns) {
      super();
      this.nodeType = 1; this.localName = tag; this.tagName = tag.toUpperCase(); this.namespaceURI = ns || null;
      this.attrs = new Map(); this.listeners = {}; this.value = ''; this.checked = false; this.disabled = false; this.files = [];
      const st = {}; Object.defineProperty(st, 'setProperty', { value: (k, v) => { st[k] = String(v); } }); this.style = st;
      const el = this;
      this.classList = {
        _get() { return new Set((el.getAttribute('class') || '').split(/\s+/).filter(Boolean)); },
        _put(s) { el.setAttribute('class', Array.from(s).join(' ')); },
        add(...c) { const s = this._get(); c.forEach((x) => s.add(x)); this._put(s); },
        remove(...c) { const s = this._get(); c.forEach((x) => s.delete(x)); this._put(s); },
        toggle(c, force) { const s = this._get(); const on = force === undefined ? !s.has(c) : !!force; if (on) s.add(c); else s.delete(c); this._put(s); return on; },
        contains(c) { return this._get().has(c); }
      };
    }
    setAttribute(k, v) { this.attrs.set(k, String(v)); if (k === 'value') this.value = String(v); if (k === 'checked') this.checked = true; if (k === 'disabled') this.disabled = true; }
    getAttribute(k) { return this.attrs.has(k) ? this.attrs.get(k) : null; }
    removeAttribute(k) { this.attrs.delete(k); }
    hasAttribute(k) { return this.attrs.has(k); }
    get id() { return this.getAttribute('id') || ''; }
    get className() { return this.getAttribute('class') || ''; }
    set className(v) { this.setAttribute('class', v); }
    get hidden() { return this.attrs.has('hidden'); }
    set hidden(v) { if (v) this.attrs.set('hidden', ''); else this.attrs.delete('hidden'); }
    get isContentEditable() { return false; }
    get offsetWidth() { return 260; }
    get offsetHeight() { return 60; }
    addEventListener(t, fn) { (this.listeners[t] = this.listeners[t] || []).push(fn); }
    removeEventListener(t, fn) { const a = this.listeners[t] || []; const i = a.indexOf(fn); if (i >= 0) a.splice(i, 1); }
    dispatchEvent(ev) { return dispatch(this, ev); }
    matches(sel) { return matches(this, sel); }
    closest(sel) { let n = this; while (n && n.nodeType === 1) { if (matches(n, sel)) return n; n = n.parentNode; } return null; }
    querySelector(sel) { return this.querySelectorAll(sel)[0] || null; }
    querySelectorAll(sel) { const out = []; const walk = (n) => { for (const c of n.childNodes) if (c.nodeType === 1) { if (matches(c, sel)) out.push(c); walk(c); } }; walk(this); return out; }
    focus() { const prev = doc.activeElement; if (prev === this) return; doc.activeElement = this; if (prev && prev !== doc.body) dispatch(prev, mkEvent('focusout')); dispatch(this, mkEvent('focusin')); }
    blur() { if (doc.activeElement === this) { doc.activeElement = doc.body; dispatch(this, mkEvent('focusout')); } }
    getBoundingClientRect() { return { left: 0, top: 0, width: 1000, height: 640, right: 1000, bottom: 640 }; }
    scrollIntoView() {}
    setPointerCapture() {}
    get isConnected() { let n = this; while (n) { if (n === doc) return true; n = n.parentNode; } return false; }
  }
  function compound(el, part) {
    if (part === ':focus-visible') return doc.activeElement === el && doc.kbd;
    const m = /^([a-zA-Z0-9]*)((?:[#.][-\w]+|\[[-\w]+\])*)$/.exec(part);
    if (!m) throw new Error('fake DOM cannot match selector part ' + part);
    if (m[1] && el.localName !== m[1]) return false;
    const toks = m[2].match(/[#.][-\w]+|\[[-\w]+\]/g) || [];
    for (const t of toks) {
      if (t[0] === '#' && el.id !== t.slice(1)) return false;
      if (t[0] === '.' && !el.classList.contains(t.slice(1))) return false;
      if (t[0] === '[' && !el.hasAttribute(t.slice(1, -1))) return false;
    }
    return true;
  }
  function matches(el, sel) {
    return sel.split(',').some((s) => {
      const parts = s.trim().split(/\s+/);
      if (!compound(el, parts[parts.length - 1])) return false;
      let n = el.parentNode, i = parts.length - 2;
      while (i >= 0 && n && n.nodeType === 1) { if (compound(n, parts[i])) i--; n = n.parentNode; }
      return i < 0;
    });
  }
  function mkEvent(type, props) {
    return Object.assign({ type: type, defaultPrevented: false, preventDefault() { this.defaultPrevented = true; }, stopPropagation() { this._stop = true; } }, props || {});
  }
  function dispatch(target, ev) {
    if (!ev.target) ev.target = target;
    let n = target;
    while (n && !ev._stop) {
      ev.currentTarget = n;
      for (const fn of (n.listeners && n.listeners[ev.type]) || []) fn.call(n, ev);
      n = n.parentNode;
    }
    return !ev.defaultPrevented;
  }
  doc.createElement = (t) => new E(t);
  doc.createElementNS = (ns, t) => new E(t, ns);
  doc.createTextNode = (t) => new T(t);
  doc.createRange = () => ({ selectNodeContents() {} });
  doc.addEventListener = (t, fn) => { (doc.listeners[t] = doc.listeners[t] || []).push(fn); };
  doc.getElementById = (id) => { const walk = (n) => { for (const c of n.childNodes) if (c.nodeType === 1) { if (c.id === id) return c; const r = walk(c); if (r) return r; } return null; }; return walk(doc.documentElement); };
  doc.querySelector = (sel) => doc.documentElement.querySelector(sel);
  doc.documentElement = new E('html');
  doc.documentElement.parentNode = doc;
  doc.body = new E('body');
  doc.documentElement.appendChild(doc.body);
  doc.activeElement = doc.body;
  doc.mkEvent = mkEvent;
  return doc;
}

function decode(t) {
  return t.replace(/&#(\d+);/g, (m, n) => String.fromCharCode(Number(n))).replace(/&amp;/g, '&').replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"');
}
function loadMarkup(doc, markup) {
  const re = /<!--[\s\S]*?-->|<\/([a-zA-Z0-9]+)\s*>|<([a-zA-Z0-9]+)((?:\s+[-\w:]+(?:\s*=\s*(?:"[^"]*"|'[^']*'|[^\s>]+))?)*)\s*(\/?)>|([^<]+)/g;
  const stack = [doc.body];
  let m;
  while ((m = re.exec(markup))) {
    const top = stack[stack.length - 1];
    if (m[0].startsWith('<!--')) continue;
    if (m[1]) { while (stack.length > 1 && stack.pop().localName !== m[1].toLowerCase()) { /* pop */ } continue; }
    if (m[2]) {
      const tag = m[2].toLowerCase();
      const inSvg = tag === 'svg' || stack.some((s) => s.namespaceURI === SVGNS);
      const el = inSvg ? doc.createElementNS(SVGNS, tag) : doc.createElement(tag);
      const ar = /([-\w:]+)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s>]+)))?/g;
      let a;
      while ((a = ar.exec(m[3] || ''))) el.setAttribute(a[1], decode(a[2] != null ? a[2] : a[3] != null ? a[3] : a[4] != null ? a[4] : ''));
      top.appendChild(el);
      if (!m[4] && !VOID.has(tag)) stack.push(el);
      continue;
    }
    if (m[5] && m[5].trim()) top.appendChild(doc.createTextNode(decode(m[5])));
  }
}

/* ---------------- run the page in a fake window ---------------- */
function makePage(opts) {
  const doc = makeDocument();
  const body = /<body>([\s\S]*?)<script\b/.exec(html)[1];
  loadMarkup(doc, body);
  const core = /<script id="astra-core">([\s\S]*?)<\/script>/.exec(html)[1];
  const app = /<script id="astra-app">([\s\S]*?)<\/script>/.exec(html)[1];
  let now = 1000, rafId = 0;
  let raf = [];
  let timers = [];
  const net = [];
  const errors = [];
  const win = {
    document: doc, console: { log() {}, warn() {}, error: (...a) => errors.push(a.map(String).join(' ')), info() {} },
    performance: { now: () => now },
    requestAnimationFrame: (fn) => { raf.push({ id: ++rafId, fn: fn }); return rafId; },
    cancelAnimationFrame: (id) => { raf = raf.filter((r) => r.id !== id); },
    setTimeout: (fn, ms) => { timers.push({ at: now + (ms || 0), fn: fn, every: 0 }); return timers.length; },
    setInterval: (fn, ms) => { timers.push({ at: now + ms, fn: fn, every: ms }); return timers.length; },
    clearTimeout() {}, clearInterval() {},
    matchMedia: () => ({ matches: false, addEventListener() {} }),
    localStorage: { _m: new Map(), getItem(k) { return this._m.has(k) ? this._m.get(k) : null; }, setItem(k, v) { this._m.set(k, String(v)); } },
    navigator: {}, location: { hash: '' }, getSelection: () => ({ removeAllRanges() {}, addRange() {} }),
    addEventListener() {},
    fetch: (url) => { net.push('fetch ' + url); return opts.fetch ? opts.fetch(url) : Promise.reject(new Error('no network in this test')); },
    EventSource: function (url) { net.push('EventSource ' + url); const es = { url: url, readyState: 0, listeners: {}, addEventListener(t, fn) { (this.listeners[t] = this.listeners[t] || []).push(fn); }, close() { this.readyState = 2; } }; win.lastES = es; return es; },
    FileReader: function () {}
  };
  win.window = win;
  if (opts.dagre) win.dagre = opts.dagre;
  if (opts.codemap) { win.ASTRA_CODEMAP = opts.codemap; win.ASTRA_TRACE = opts.trace || []; }
  vm.createContext(win);
  try {
    new vm.Script(core, { filename: 'astra-core.js' }).runInContext(win);
    new vm.Script(app, { filename: 'astra-app.js' }).runInContext(win);
  } catch (err) { errors.push('script threw: ' + (err && err.stack || err)); }
  const page = {
    win: win, doc: doc, net: net, errors: errors,
    $: (id) => doc.getElementById(id),
    // advance the fake clock: run due timers and animation frames (16 ms per frame)
    pump(ms) {
      const end = now + ms;
      while (now < end) {
        now += 16;
        const due = timers.filter((t) => t.at <= now);
        timers = timers.filter((t) => t.at > now || t.every);
        for (const t of due) { try { t.fn(); } catch (err) { errors.push('timer threw: ' + (err && err.stack || err)); } if (t.every) t.at = now + t.every; }
        const q = raf; raf = [];
        for (const r of q) { try { r.fn(now); } catch (err) { errors.push('frame threw: ' + (err && err.stack || err)); } }
      }
    },
    nodes() { return page.$('viewport').querySelectorAll('.node'); },
    node(id) { return page.nodes().find((n) => n.getAttribute('data-el') === id) || null; },
    tap(el, props) {
      const shape = el.querySelector('.shape') || el;
      const base = Object.assign({ pointerId: 1, pointerType: 'mouse', button: 0, clientX: 100, clientY: 100 }, props || {});
      shape.dispatchEvent(doc.mkEvent('pointerdown', Object.assign({}, base)));
      shape.dispatchEvent(doc.mkEvent('pointerup', Object.assign({}, base)));
    },
    // like a browser: a key goes to the focused element, or to <body> when that element was removed
    key(target, key) { doc.kbd = true; const t = target && target.isConnected ? target : doc.body; t.dispatchEvent(doc.mkEvent('keydown', { key: key })); },
    text(id) { const e = page.$(id); return e ? e.textContent : ''; }
  };
  return page;
}
const flush = () => new Promise((r) => setImmediate(r));

async function artifactMode() {
  const trace = recordingText.split('\n').filter((l) => l.trim()).map((l) => JSON.parse(l));
  const p = makePage({ codemap: sample, trace: trace, dagre: dagre });
  await flush(); await flush();
  p.pump(50);
  const top = p.nodes();
  check(top.length === 8, 'artifact: top level draws 3 systems + 5 programs (got ' + top.length + ')');
  check(p.node('ext:gazebo') && p.node('sys:team_stack'), 'artifact: Gazebo and the team stack are on the top level');
  check(p.$('viewport').querySelectorAll('.edge').length >= 5, 'artifact: top level has labelled links');
  check(/Recorded run/.test(p.text('panel-body')), 'artifact: panel says how to see it live');
  check(/demo_traced\.sh/.test(p.text('panel-body')) && /127\.0\.0\.1:8090/.test(p.text('panel-body')), 'artifact: live-mode command and URL shown');
  check(p.$('tab-live').hidden === true, 'artifact: no Live tab');
  check(p.$('btn-play').classList.contains('is-playing'), 'artifact: the replay starts by itself');
  // play 12 s of the recording
  let maxGlow = 0;
  for (let i = 0; i < 48; i++) {
    p.pump(250);
    for (const n of p.nodes()) { const g = n.querySelector('.glow'); const v = Number(g && g.style.opacity || 0); if (v > maxGlow) maxGlow = v; }
  }
  check(maxGlow > 0.3, 'artifact: blocks glow green during the replay (max opacity ' + maxGlow.toFixed(2) + ')');
  check(p.node('sys:team_stack').classList.contains('has-count'), 'artifact: the team stack counts calls');
  check(p.node('ext:gazebo').classList.contains('has-run') && p.node('ext:gazebo').classList.contains('is-running'), 'artifact: Gazebo shows RUNNING');
  check(p.node('sys:team_stack').classList.contains('has-run'), 'artifact: the team stack shows "N running"');
  check(p.$('ticker-list').childNodes.length > 0 && p.$('ticker-empty').hidden, 'artifact: the "just ran" log fills');
  check(/\d+:\d\d \/ 1:1\d/.test(p.text('rep-time')), 'artifact: replay clock moves (' + p.text('rep-time') + ')');
  // click = details, double-click = open
  p.tap(p.node('sys:team_stack'));
  p.pump(50);
  check(p.text('panel-title') === 'Team flight code', 'click: panel shows the system (' + p.text('panel-title') + ')');
  check(p.node('sys:team_stack').classList.contains('is-sel'), 'click: the block is selected');
  p.pump(100);
  p.tap(p.node('sys:team_stack'));
  p.pump(50);
  check(p.node('pkg:ros2_ws/src/main') && p.node('pkg:ros2_ws/src/video_cam'), 'double-click: the system opens into its packages');
  check(/Team flight code/.test(p.text('crumbs')), 'double-click: breadcrumbs show where we are');
  // search + Enter jumps to a function
  const s = p.$('search');
  s.value = 'aim_at_box';
  s.dispatchEvent(p.doc.mkEvent('input'));
  check(!p.$('search-results').hidden && p.$('search-results').childNodes.length > 0, 'search: results list opens');
  s.dispatchEvent(p.doc.mkEvent('keydown', { key: 'Enter' }));
  p.pump(400);
  const aim = p.node('fn:ros2_ws/src/main/src/target_geolocation.py::aim_at_box');
  check(!!aim && aim.classList.contains('is-sel'), 'search: jumped to aim_at_box and selected it');
  check(p.text('panel-title') === 'aim_at_box', 'search: panel shows aim_at_box');
  check(/Called by/.test(p.text('panel-body')) && /Calls \(/.test(p.text('panel-body')), 'panel: who calls it and what it calls');
  check(/What it does/.test(p.text('panel-body')) && /Source code/.test(p.text('panel-body')), 'panel: explanation and source sections');
  check(/target_geolocation\.py/.test(p.text('crumbs')), 'search: breadcrumbs end at the file');
  // keyboard: arrows move focus, Enter twice opens, Backspace goes up
  p.doc.kbd = true;
  aim.focus();
  p.key(aim, 'ArrowLeft');
  check(p.doc.activeElement && p.doc.activeElement !== aim && p.doc.activeElement.classList.contains('node'), 'keys: arrow moves to another block');
  p.key(p.doc.activeElement, 'Backspace');
  p.pump(400);
  check(p.node('mod:ros2_ws/src/main/src/target_geolocation.py') !== null, 'keys: Backspace goes up to the package');
  const tg = p.node('mod:ros2_ws/src/main/src/target_geolocation.py');
  tg.focus();
  p.key(tg, 'Enter');
  p.key(tg, 'Enter');
  p.pump(400);
  check(p.node('fn:ros2_ws/src/main/src/target_geolocation.py::aim_at_box') !== null, 'keys: Enter twice opens the file');
  check(p.doc.activeElement && p.doc.activeElement.isConnected && p.doc.activeElement.classList.contains('node'), 'keys: focus moves into the opened view');
  // show tests, overview, legend, scrubber
  p.key(p.doc.activeElement || p.doc.body, 'Backspace');
  p.pump(300);
  check(!p.node('mod:ros2_ws/src/main/test/test_target_geolocation.py'), 'tests hidden by default');
  p.$('opt-tests').checked = true;
  p.$('opt-tests').dispatchEvent(p.doc.mkEvent('change'));
  p.pump(300);
  check(!!p.node('mod:ros2_ws/src/main/test/test_target_geolocation.py'), '"Show tests" draws the test file');
  p.$('btn-overview').dispatchEvent(p.doc.mkEvent('click'));
  p.pump(300);
  check(p.$('viewport').querySelectorAll('.cluster').length === 3, 'overview: one box per system (' + p.$('viewport').querySelectorAll('.cluster').length + ')');
  check(p.nodes().length >= 12, 'overview: packages and programs drawn (' + p.nodes().length + ')');
  p.$('btn-legend').dispatchEvent(p.doc.mkEvent('click'));
  check(!p.$('legend').hidden && /MAVLink/.test(p.text('legend')), 'legend opens and explains the lines');
  p.$('scrub').value = '900';
  p.$('scrub').dispatchEvent(p.doc.mkEvent('input'));
  p.pump(300);
  check(/1:0\d \/ 1:1\d/.test(p.text('rep-time')), 'scrubber jumps near the end (' + p.text('rep-time') + ')');
  p.$('btn-play').dispatchEvent(p.doc.mkEvent('click'));
  check(!p.$('btn-play').classList.contains('is-playing'), 'pause button pauses');
  // a group block when a view has too many blocks is covered by the core self-test (cap), here: follow mode
  p.$('btn-play').dispatchEvent(p.doc.mkEvent('click'));
  p.$('opt-follow').checked = true;
  p.$('opt-follow').dispatchEvent(p.doc.mkEvent('change'));
  p.$('scrub').value = '100';
  p.$('scrub').dispatchEvent(p.doc.mkEvent('input'));
  p.pump(9000);
  const crumbs = p.text('crumbs');
  check(/\.py/.test(crumbs), 'follow: opened the busiest file (' + crumbs + ')');
  check(p.net.length === 0, 'artifact: no network use at all (' + p.net.join(', ') + ')');
  check(p.errors.length === 0, 'artifact: no errors (' + p.errors.slice(0, 3).join(' | ') + ')');
}

async function localMode() {
  const fetch = (url) => {
    if (url === '/codemap.json') return Promise.resolve({ ok: false, status: 404, json: () => Promise.reject(new Error('404')), text: () => Promise.resolve('') });
    if (url === 'sample_codemap.json') return Promise.resolve({ ok: true, status: 200, json: () => Promise.resolve(JSON.parse(JSON.stringify(sample))), text: () => Promise.resolve('') });
    if (url === '/recordings') return Promise.resolve({ ok: true, status: 200, json: () => Promise.resolve(['20261031-150000.jsonl']) });
    if (url.indexOf('/recordings/') === 0 || url === 'sample_recording.jsonl') return Promise.resolve({ ok: true, status: 200, text: () => Promise.resolve(recordingText) });
    return Promise.reject(new Error('unexpected url ' + url));
  };
  const p = makePage({ fetch: fetch, dagre: dagre });
  for (let i = 0; i < 6; i++) await flush();
  p.pump(50);
  check(p.nodes().length === 8, 'localhost: the sample map is drawn after /codemap.json fails (' + p.nodes().length + ')');
  check(!p.$('sample-chip').hidden, 'localhost: SAMPLE DATA chip shown');
  check(p.net.indexOf('EventSource /events') >= 0, 'localhost: listens to /events');
  const es = p.win.lastES;
  check(/waiting/i.test(p.text('mode-text')) || /loading/i.test(p.text('mode-text')), 'localhost: pill says waiting before the server answers (' + p.text('mode-text') + ')');
  es.readyState = 1;
  es.onopen();
  check(p.text('mode-text') === 'Live connected', 'localhost: pill says LIVE connected (' + p.text('mode-text') + ')');
  const send = (type, data) => (es.listeners[type] || []).forEach((fn) => fn({ data: JSON.stringify(data) }));
  send('procs', { t: 1000, running: ['ext:gazebo', 'ext:mavros', 'mod:ros2_ws/src/main/src/main_controller.py'] });
  send('calls', { t: 1000.2, calls: { 'fn:ros2_ws/src/main/src/main_controller.py::MainController.image_result_cb': 3, 'fn:not/in/map.py::x': 1 } });
  p.pump(50);
  const team = p.node('sys:team_stack');
  check(Number(team.querySelector('.glow').style.opacity) > 0.3, 'localhost: a live call makes the system glow');
  p.pump(300);   // counters repaint at most 4 times a second
  check(team.classList.contains('has-count') && /3/.test(team.querySelector('.badge.count').textContent), 'localhost: the system counts the live call');
  check(p.node('ext:gazebo').classList.contains('is-running'), 'localhost: procs mark Gazebo running');
  p.pump(1600);
  check(Number(team.querySelector('.glow').style.opacity) === 0, 'localhost: the glow is gone after about 1.5 s');
  check(/calls seen/.test(p.text('live-stats')) && /not in the map/.test(p.text('live-stats')), 'localhost: live line counts calls and unknown ids (' + p.text('live-stats') + ')');
  p.$('tab-replay').dispatchEvent(p.doc.mkEvent('click'));
  for (let i = 0; i < 6; i++) await flush();
  p.pump(2000);
  check(p.net.indexOf('fetch /recordings') >= 0 && p.net.some((x) => x.indexOf('fetch /recordings/2026') === 0), 'localhost: Replay tab lists and loads a recording');
  check(p.$('btn-play').classList.contains('is-playing'), 'localhost: the chosen recording plays');
  check(p.errors.length === 0, 'localhost: no errors (' + p.errors.slice(0, 3).join(' | ') + ')');
}

// --codemap PATH: boot ANY code map (for example the real codemap.json) in artifact mode and walk it: open every
// system, then the first package and file inside, search, the overview; no errors and no network allowed.
async function anyCodemap(cmPath) {
  const cm = JSON.parse(fs.readFileSync(cmPath, 'utf8'));
  const t0 = Date.now();
  const p = makePage({ codemap: cm, trace: [], dagre: dagre });
  await flush(); await flush();
  p.pump(50);
  console.log('booted ' + cm.nodes.length + ' nodes in ' + (Date.now() - t0) + ' ms (fake DOM)');
  const top = p.nodes().map((n) => n.getAttribute('data-el'));
  check(top.length > 0, 'any map: top level drawn (' + top.length + ' blocks)');
  let opened = 0;
  for (const id of top) {
    const el = p.node(id);
    if (!el) continue;
    p.tap(el); p.pump(100); p.tap(el); p.pump(400);
    if (p.text('crumbs').indexOf('All systems') >= 0 && p.nodes().length && !p.node(id)) {
      opened++;
      const first = p.nodes().find((n) => n.getAttribute('data-el').indexOf('pkg:') === 0 || n.getAttribute('data-el').indexOf('mod:') === 0);
      if (first) {
        p.tap(first); p.pump(100); p.tap(first); p.pump(400);
        const fileEl = p.nodes().find((n) => n.getAttribute('data-el').indexOf('mod:') === 0);
        if (fileEl) { p.tap(fileEl); p.pump(100); p.tap(fileEl); p.pump(400); }
      }
    }
    p.$('crumbs').querySelector('.crumb').dispatchEvent(p.doc.mkEvent('click'));
    p.pump(300);
  }
  check(opened > 0, 'any map: systems open into their parts (' + opened + ' of ' + top.length + ')');
  const fn = cm.nodes.find((n) => n.kind === 'function' && n.label);
  if (fn) {
    p.$('search').value = fn.label;
    p.$('search').dispatchEvent(p.doc.mkEvent('input'));
    p.$('search').dispatchEvent(p.doc.mkEvent('keydown', { key: 'Enter' }));
    p.pump(400);
    check(p.text('panel-title').length > 0, 'any map: search + Enter shows a panel (' + p.text('panel-title') + ')');
  }
  p.$('btn-overview').dispatchEvent(p.doc.mkEvent('click'));
  p.pump(400);
  check(p.nodes().length > 0, 'any map: overview drawn (' + p.nodes().length + ' blocks, ' + p.$('viewport').querySelectorAll('.cluster').length + ' system boxes)');
  check(p.net.length === 0, 'any map: no network use');
  check(p.errors.length === 0, 'any map: no errors (' + p.errors.slice(0, 3).join(' | ') + ')');
}

// --artifact PATH: take the DATA out of a built dist/astra_code_map.html (its astra-embed script: the real code map
// and the real recorded run) and replay it with this folder's page scripts. Checks that the replay starts by itself,
// that blocks glow and count, that the team stack and Gazebo light up somewhere in the run, no network, no errors.
async function artifactData(htmlPath) {
  const text = fs.readFileSync(htmlPath, 'utf8');
  const m = /<script\b[^>]*id="astra-embed"[^>]*>([\s\S]*?)<\/script\s*>/i.exec(text);
  check(!!m, 'artifact file: has the astra-embed data block');
  if (!m) return;
  const box = { window: {} };
  vm.createContext(box);
  new vm.Script(m[1], { filename: 'astra-embed.js' }).runInContext(box);
  const cm = box.window.ASTRA_CODEMAP, trace = box.window.ASTRA_TRACE || [];
  check(cm && Array.isArray(cm.nodes) && cm.nodes.length > 0, 'artifact file: window.ASTRA_CODEMAP has nodes');
  check(Array.isArray(trace) && trace.length > 0, 'artifact file: window.ASTRA_TRACE has events (' + trace.length + ')');
  const ids = new Set(cm.nodes.map((n) => n.id));
  let unknown = 0;
  for (const ev of trace) {
    for (const id of Object.keys(ev.calls || {})) if (!ids.has(id)) unknown++;
    for (const id of ev.running || []) if (!ids.has(id)) unknown++;
  }
  check(unknown === 0, 'artifact file: every id in the recording is a block of the map (' + unknown + ' unknown)');
  const t0 = Date.now();
  const p = makePage({ codemap: cm, trace: trace, dagre: dagre });
  await flush(); await flush();
  p.pump(50);
  console.log('artifact data: ' + cm.nodes.length + ' nodes, ' + trace.length + ' events, booted in ' + (Date.now() - t0) + ' ms (fake DOM)');
  check(p.nodes().length > 0, 'artifact data: top level drawn (' + p.nodes().length + ' blocks)');
  check(p.$('btn-play').classList.contains('is-playing'), 'artifact data: the replay starts by itself');
  const seen = { glow: 0, team: false, gazebo: false };
  const look = () => {
    for (const n of p.nodes()) { const g = n.querySelector('.glow'); const v = Number(g && g.style.opacity || 0); if (v > seen.glow) seen.glow = v; }
    const team = p.node('sys:team_stack');
    if (team && team.classList.contains('has-count')) seen.team = true;
    const gz = p.node('ext:gazebo') || p.node('sys:sim_world');
    if (gz && gz.classList.contains('has-run')) seen.gazebo = true;
  };
  // jump through the run (start, 25 %, 50 %, 75 %) and play 10 s at each place
  for (const at of ['0', '250', '500', '750']) {
    p.$('scrub').value = at;
    p.$('scrub').dispatchEvent(p.doc.mkEvent('input'));
    for (let i = 0; i < 40; i++) { p.pump(250); look(); }
  }
  check(seen.glow > 0.3, 'artifact data: blocks glow green during the replay (max opacity ' + seen.glow.toFixed(2) + ')');
  check(seen.team, 'artifact data: the team flight software counts calls at some point of the run');
  check(seen.gazebo, 'artifact data: the simulated world shows running programs at some point of the run');
  check(/\d+:\d\d \/ \d+:\d\d/.test(p.text('rep-time')), 'artifact data: replay clock shown (' + p.text('rep-time') + ')');
  check(p.net.length === 0, 'artifact data: no network use at all (' + p.net.join(', ') + ')');
  check(p.errors.length === 0, 'artifact data: no errors (' + p.errors.slice(0, 3).join(' | ') + ')');
}

(async () => {
  const artArg = process.argv.indexOf('--artifact');
  if (artArg >= 0) {
    await artifactData(path.resolve(process.argv[artArg + 1]));
    console.log((failures ? failures + ' FAILED, ' : 'all ') + passes + (failures ? ' passed' : ' checks passed'));
    process.exit(failures ? 1 : 0);
  }
  const cmArg = process.argv.indexOf('--codemap');
  if (cmArg >= 0) {
    await anyCodemap(path.resolve(process.argv[cmArg + 1]));
    console.log((failures ? failures + ' FAILED, ' : 'all ') + passes + (failures ? ' passed' : ' checks passed'));
    process.exit(failures ? 1 : 0);
  }
  await artifactMode();
  await localMode();
  console.log((failures ? failures + ' FAILED, ' : 'all ') + passes + (failures ? ' passed' : ' checks passed') + ' (layout: ' + (dagre ? 'dagre' : 'built-in simple layout') + ')');
  process.exit(failures ? 1 : 0);
})().catch((err) => { console.log('FAIL smoke test crashed: ' + (err && err.stack || err)); process.exit(1); });
