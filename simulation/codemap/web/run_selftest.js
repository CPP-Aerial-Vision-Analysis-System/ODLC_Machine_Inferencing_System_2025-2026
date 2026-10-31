/*
 * simulation/codemap/web/run_selftest.js -- PURPOSE: test the explorer page (index.html) without a browser.
 * It cuts the two inline scripts out of index.html, checks that both parse as JavaScript (like `node --check`),
 * runs the pure-logic script (id="astra-core") inside node and calls its built-in self-test
 * (AstraCore.selfTest): id parsing, the code-map index, building every kind of diagram level, both layouts,
 * the green glow fade, replay stepping, event parsing, search and formatting. When sample_codemap.json and
 * sample_recording.jsonl sit next to index.html, every view of the sample map is built and laid out and every
 * recorded call is checked against the map.
 *
 * Run (Windows or Linux, node 18 or newer, no packages needed):
 *     node simulation/codemap/web/run_selftest.js
 *     node simulation/codemap/web/run_selftest.js --dagre path/to/dagre.min.js   (also test the dagre layout)
 * Exit code 0 = all checks passed, 1 = a check failed, 2 = the page could not be read or parsed.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const here = __dirname;
const args = process.argv.slice(2);
const pageArg = args.indexOf('--page');
const dagreArg = args.indexOf('--dagre');
const pagePath = pageArg >= 0 ? args[pageArg + 1] : path.join(here, 'index.html');

function inlineScript(html, id) {
  // the script element with this id, up to its closing tag (the page never writes that closing tag inside code)
  const re = new RegExp('<script id="' + id + '">([\\s\\S]*?)</script>');
  const m = re.exec(html);
  return m ? m[1] : null;
}

let html;
try { html = fs.readFileSync(pagePath, 'utf8'); } catch (err) { console.error('cannot read ' + pagePath + ': ' + err.message); process.exit(2); }
const core = inlineScript(html, 'astra-core');
const app = inlineScript(html, 'astra-app');
if (!core || !app) { console.error('index.html must contain the scripts with id astra-core and astra-app'); process.exit(2); }

// 1. Both scripts must parse.
for (const [name, code] of [['astra-core', core], ['astra-app', app]]) {
  try { new vm.Script(code, { filename: name + '.js' }); console.log('parse ok: ' + name + ' (' + code.split('\n').length + ' lines)'); }
  catch (err) { console.error('parse FAILED: ' + name + ': ' + err.message); process.exit(2); }
}

// 2. Run the pure logic and its self-test.
const sandbox = { console: console };
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
new vm.Script(core, { filename: 'astra-core.js' }).runInContext(sandbox);
const Core = sandbox.AstraCore;
if (!Core || typeof Core.selfTest !== 'function') { console.error('astra-core did not define AstraCore.selfTest'); process.exit(2); }

const extra = {};
const samplePath = path.join(here, 'sample_codemap.json');
const recPath = path.join(here, 'sample_recording.jsonl');
if (fs.existsSync(samplePath)) extra.sample = JSON.parse(fs.readFileSync(samplePath, 'utf8'));
if (fs.existsSync(recPath)) extra.recording = fs.readFileSync(recPath, 'utf8');
if (dagreArg >= 0) {
  // dagre's browser bundle also works as a node module (module.exports = dagre)
  extra.dagre = require(path.resolve(args[dagreArg + 1]));
  console.log('dagre ' + (extra.dagre.version || '?') + ' loaded for the layout checks');
}
const res = Core.selfTest(extra);
for (const line of res.lines) console.log(line);
console.log('sample map: ' + (extra.sample ? 'checked' : 'not found') + ', sample recording: ' + (extra.recording ? 'checked' : 'not found'));
process.exit(res.failed ? 1 : 0);
