// Browser viewer for the multi-arm simulation. The physics runs on the main
// thread inside requestAnimationFrame with a wall-clock budget per frame.
//
// Page layout (see index.html): top bar (mode, run, reset, speed, theme,
// help), Jobs rail on the left, the 3D view with overlay controls and the
// scope strip in the centre, and a tabbed panel on the right (Inspect,
// Explore, Control, Signals, Console), with a status strip at the bottom.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { Machine } from '../src/machine/machine.js';
import { MATERIALS } from '../src/process/workpiece.js';
import { applyDirectives, parseDirectives } from '../src/cam/scenario.js';
import { parseSTL, centreOnPlate, meshBounds } from '../src/cam/mesh.js';
import { sliceMesh, printGcode } from '../src/cam/slicer.js';
import { MachineView } from './view3d.js';
import { Scope } from './scope.js';
import { Inspector, fmtNum } from './inspector.js';
import { icon, hydrateIcons } from './ui/icons.js';
import { JOBS, renderJobs, estimateTime, fmtEstimate } from './ui/jobs.js';
import { Explorer } from './ui/explore.js';
import { ControlPanel } from './ui/control.js';
import { Tour, storageGet, storageSet } from './ui/tour.js';

const $ = (id) => document.getElementById(id);
const FRAME_BUDGET_MS = 12;   // wall time spent on physics per animation frame
const MAX_BUDGET_MS = 24;     // at "max" speed physics may take more of each frame
const CHUNK_S = 0.002;        // sim time per m.run() call

hydrateIcons();

// ------------------------------------------------------------------ preferences
const MODE_TEXT = {
  demo: '<b>Demo mode:</b> simplified physics (ideal motors, a kinematic arm). Jobs run many times faster.',
  full: '<b>Full physics:</b> every coil current, gearbox, wire and heat flow is simulated. Slower, closest to the real build.',
};
let simMode = storageGet('multiarm.mode') === 'full' ? 'full' : 'demo';
// Themes: follow the system (light or dark), or pick light, mid (Blender's
// default interface greys) or dark.
const THEMES = ['auto', 'light', 'mid', 'dark'];
const THEME_NAME = { auto: 'follows your system', light: 'light', mid: 'mid grey (Blender-style)', dark: 'dark' };
let theme = THEMES.includes(storageGet('multiarm.theme')) ? storageGet('multiarm.theme') : 'auto';

function applyThemeAttr() {
  if (theme === 'auto') document.documentElement.removeAttribute('data-theme');
  else document.documentElement.setAttribute('data-theme', theme);
  const b = $('btnTheme');
  b.innerHTML = icon({ auto: 'auto', light: 'sun', mid: 'mid', dark: 'moon' }[theme]);
  b.title = `Colour theme: ${THEME_NAME[theme]} (click to change)`;
  b.setAttribute('aria-label', b.title);
}
applyThemeAttr();

// ------------------------------------------------------------------ three.js
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const viewportEl = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
viewportEl.prepend(renderer.domElement);
renderer.domElement.setAttribute('aria-label', '3D view of the machine. Click a part to inspect it.');
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 20);
camera.position.set(0.64, -0.6, 0.46);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.1, 0, 0.15);
controls.enableDamping = true;
controls.dampingFactor = 0.12;
controls.update();

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x20242a, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(0.6, -0.8, 1.4);
scene.add(sun);
const fill = new THREE.DirectionalLight(0xbfd4ff, 0.5);
fill.position.set(-1, 0.6, 0.5);
scene.add(fill);

function isLight() {
  if (theme !== 'auto') return theme === 'light';
  return !window.matchMedia('(prefers-color-scheme: dark)').matches;
}
function applyTheme() {
  const css = getComputedStyle(document.documentElement);
  const bg = css.getPropertyValue('--scene-bg').trim() || '#11151b';
  scene.background = new THREE.Color(bg);
  const light = isLight();
  benchMat.color.set(light ? '#d8d5ca' : theme === 'mid' ? '#303030' : '#1f1e1d');
  grid.material.color.set(light ? '#c2c0b6' : theme === 'mid' ? '#545454' : '#3e3e3a');
}
// Bench.
const benchMat = new THREE.MeshStandardMaterial({ color: '#1b2027', roughness: 0.95 });
const bench = new THREE.Mesh(new THREE.PlaneGeometry(1.8, 1.3), benchMat);
bench.position.set(-0.18, 0.05, -0.0006);
scene.add(bench);
const grid = new THREE.GridHelper(1.8, 36, 0x2a323d, 0x2a323d);
grid.rotation.x = Math.PI / 2;
grid.position.set(-0.18, 0.05, -0.0003);
grid.material.transparent = true; grid.material.opacity = 0.6;
scene.add(grid);
applyTheme();
window.matchMedia('(prefers-color-scheme: dark)').addEventListener?.('change', applyTheme);

function resize() {
  const r = viewportEl.getBoundingClientRect();
  renderer.setSize(Math.max(1, r.width), Math.max(1, r.height), false);
  camera.aspect = Math.max(1, r.width) / Math.max(1, r.height);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewportEl);
resize();

// ------------------------------------------------------------------ state
let m = null, view = null;
let running = false;
let speed = 1;
let selection = null;
let job = null;              // progress of the loaded job
let loadedJob = null;        // { text, name, def } to reload on mode switch
const rtf = { sim: 0, value: NaN, t0: performance.now() };
const layers = { current: false, toolpath: true, couplings: true, enclosure: true };

const scope = new Scope({
  canvas: $('scope'),
  selects: [...document.querySelectorAll('#tab-signals .ch')],
  readouts: [...document.querySelectorAll('#chanChips .cc-val')],
  names: [...document.querySelectorAll('#chanChips .cc-name')],
  windowSelect: $('scopeWin'),
  onChange: (chs) => {
    document.querySelectorAll('#chanChips .chanchip').forEach((c, k) => {
      c.classList.toggle('off', !chs[k]);
      c.title = chs[k] ? `${chs[k]}: click to change` : 'Channel off: click to choose a signal';
    });
  },
});
function paintChannelColours() {
  const css = getComputedStyle(document.documentElement);
  document.querySelectorAll('#chanChips .chanchip').forEach((c, k) => { c.style.color = css.getPropertyValue(`--ch${k + 1}`); });
  document.querySelectorAll('#tab-signals .chanrow').forEach((c, k) => { c.querySelector('.cr-dot').style.color = css.getPropertyValue(`--ch${k + 1}`); });
}
paintChannelColours();

const SCOPE_PRESETS = [
  { name: 'Overview', text: 'Bus, a coil, hot end, tip error', ch: ['24 V bus at PSU', 'j2 coil A', 'hot end (true)', 'tip error'] },
  { name: 'Power', text: 'Supply voltage, current, ground offset', ch: ['24 V bus at PSU', 'PSU current', 'Driver j2 VM', 'Ground offset board-j2'] },
  { name: 'Arm motion', text: 'Coil currents and gearbox wind-up', ch: ['j2 coil A', 'j3 coil A', 'j2 gearbox wind-up', 'tip error'] },
  { name: 'Printing', text: 'Hot end, duty, extrusion force, flow', ch: ['hot end (true)', 'hot end duty', 'extrusion force', 'melt flow'] },
  { name: 'Milling', text: 'Spindle speed, current, cutting power', ch: ['spindle speed', 'spindle current', 'cutting power', 'part-frame error'] },
  { name: 'Enclosure and probe', text: 'Chamber air, fan, stylus, bed', ch: ['chamber air', 'exhaust fan', 'probe stylus', 'bed (true)'] },
];
$('scopePresets').innerHTML = SCOPE_PRESETS.map((p, i) => `<button type="button" class="btn preset" data-i="${i}"><b>${p.name}</b><span>${p.text}</span></button>`).join('');
$('scopePresets').addEventListener('click', (e) => {
  const b = e.target.closest('.preset');
  if (b) { scope.setChannels(SCOPE_PRESETS[+b.dataset.i].ch); setScopeOpen(true); }
});

const inspector = new Inspector($('inspector'), {
  onSelect: (ref) => select(ref),
  suggestions: (mm) => [
    ['Hot-end heater', 'hot_heater'], ['Shoulder motor (j2)', 'mot_j2'], ['Touch probe', 'probe'],
    ['Enclosure', 'enclosure'], ['Power supply', 'psu'], ['Controller board', 'board'],
  ].filter(([, id]) => mm.A.components.has(id)).map(([label, id]) => ({ label, ref: { type: 'component', id } })),
});
const explorer = new Explorer($('parts'), $('partsFilter'), { onSelect: (ref) => select(ref) });
const control = new ControlPanel($('control'), {
  send: (lines) => sendGcode(lines, 'control'),
  getMachine: () => m,
  setDoor: (open) => setDoor(open),
  onProbeClear: () => clearProbeMarkers(),
});

// ------------------------------------------------------------------ log
const logEl = $('log');
let unread = 0;
function addLog(cls, text, t = m?.t) {
  const atBottom = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 8;
  const line = document.createElement('div');
  line.className = `line ${cls}`;
  const ts = document.createElement('span');
  ts.className = 't';
  ts.textContent = t == null ? '' : `${t.toFixed(3)}s`;
  line.append(ts, document.createTextNode(text));
  logEl.append(line);
  while (logEl.childElementCount > 600) logEl.firstElementChild.remove();
  if (atBottom) logEl.scrollTop = logEl.scrollHeight;
  if (activeTab !== 'console' && cls !== 'info' && cls !== 'cmd') {
    unread++;
    $('logBadge').hidden = false;
    $('logBadge').textContent = unread > 99 ? '99+' : String(unread);
  }
}
let lastPlant = null;
function pollPlantLog() {
  const log = m.log;
  let i = lastPlant ? log.lastIndexOf(lastPlant) + 1 : 0;
  for (; i < log.length; i++) addLog('plant', `[plant] ${log[i][1]}`, log[i][0]);
  if (log.length) lastPlant = log[log.length - 1];
}
function onFirmwareMessage(msg) {
  const prb = /^PRB:(-?[\d.]+),(-?[\d.]+),(-?[\d.]+):1/.exec(msg);
  if (prb) {
    const [x, y, z] = prb.slice(1, 4).map(Number);
    addLog('prb', `${msg}   (probe touched at X ${x.toFixed(3)}  Y ${y.toFixed(3)}  Z ${z.toFixed(3)} mm)`);
    control.addProbe(x, y, z, m.t);
    addProbeMarker();
    return;
  }
  addLog(/^(error|!!|warning)/.test(msg) ? 'warn' : 'fw', msg);
}
$('btnClearLog').addEventListener('click', () => { logEl.replaceChildren(); });

// ------------------------------------------------------------------ probe markers
// A small dot where the stylus touched, riding on the plate so it turns with it.
const probeMarkers = [];
const markerGeom = new THREE.SphereGeometry(0.0012, 16, 12);
const markerMat = new THREE.MeshBasicMaterial({ color: '#1fd18a', depthTest: false, transparent: true, opacity: 0.95 });
function addProbeMarker() {
  if (!m || !view) return;
  try {
    const tip = m.toolTip();
    const mk = new THREE.Mesh(markerGeom, markerMat);
    mk.renderOrder = 10;
    const group = view.bodyGroups?.get?.(m.workpiece?.body);
    const world = new THREE.Vector3(tip[0], tip[1], tip[2]);
    if (group) {
      scene.updateMatrixWorld();
      group.worldToLocal(world);
      mk.position.copy(world);
      if (group.matrixAutoUpdate === false) mk.matrixAutoUpdate = true;
      group.add(mk);
    } else {
      mk.position.copy(world);
      scene.add(mk);
    }
    probeMarkers.push(mk);
  } catch (err) { console.warn('probe marker', err); }
}
function clearProbeMarkers() {
  for (const mk of probeMarkers) mk.parent?.remove(mk);
  probeMarkers.length = 0;
}

// ------------------------------------------------------------------ machine lifecycle
function createMachine({ stock: stockOverride } = {}) {
  clearProbeMarkers();
  if (view) { scene.remove(view.root); view.dispose(); view = null; }
  const quick = $('quick').checked;
  const preheated = $('preheated').checked || quick;
  m = new Machine({ mode: simMode, preheated, startHomed: quick });
  job = null;
  $('jobCard').hidden = true;
  const stock = stockOverride ?? $('stock').value;
  if (stock && MATERIALS[stock]) m.workpiece.addStock(MATERIALS[stock], 0.03, 0.03, 0.008, -0.03, 0);
  m.host.onMessage = onFirmwareMessage;
  lastPlant = null;
  view = new MachineView(m);
  applyLayers();
  scene.add(view.root);
  view.attach?.(renderer, scene);   // newer scenes set up their own lighting and bench
  scope.attach(m, SCOPE_PRESETS[0].ch);
  inspector.attach(m);
  explorer.attach(m);
  const pr = $('problems');
  if (m.problems.length) {
    pr.hidden = false;
    pr.innerHTML = `<h4>Assembly problems (${m.problems.length})</h4><ul>${m.problems.map((p) => `<li>${escapeHtml(p)}</li>`).join('')}</ul>`;
    for (const p of m.problems) addLog('warn', `[assembly] ${p}`, 0);
  } else pr.hidden = true;
  addLog('info', `Machine built (${simMode === 'demo' ? 'demo mode' : 'full physics'}): ${m.A.components.size} components, ${m.A.connections.size} connections (${m.wires.length} wires)${quick ? ', already homed' : ''}${preheated ? ', preheated' : ''}${stock ? `, ${stock} stock` : ''}.`, 0);
  if (selection && (selection.type === 'component' ? m.A.components.has(selection.id) : m.A.connections.has(selection.id))) select(selection, { keepTab: true });
  else select(null, { keepTab: true });
  rtf.sim = 0; rtf.t0 = performance.now(); rtf.value = NaN;
  markLoadedCard();
  updateStatus();
  control.refresh();
}

function escapeHtml(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]); }

function setRunning(on) {
  running = on;
  pace.lastWall = performance.now(); pace.debt = 0;
  rtf.sim = 0; rtf.t0 = performance.now(); rtf.value = NaN;
  if (on) schedulePump();
  const b = $('btnRun');
  b.innerHTML = `${icon(on ? 'pause' : 'play')}<span>${on ? 'Pause' : 'Run'}</span>`;
  b.classList.toggle('running', on);
  b.classList.toggle('primary', !on);
  b.setAttribute('aria-pressed', String(on));
  if (on) setHint(null);
}

function setHint(html) {
  const h = $('hint');
  if (!html) { h.hidden = true; return; }
  h.innerHTML = html;
  h.hidden = false;
}
function defaultHint() {
  const narrow = window.matchMedia('(max-width: 1199px)').matches;
  setHint(`Choose a job ${narrow ? 'below' : 'on the left'} and press <b>Load</b>, then <b>Run</b>. Or click any part to inspect it.`);
}

function select(ref, { keepTab = false } = {}) {
  selection = ref;
  view?.setSelection(ref);
  inspector.show(ref);
  explorer.setSelection(ref);
  if (ref && !keepTab && activeTab !== 'inspect' && activeTab !== 'explore') showTab('inspect');
}

// Commands from the Control tab and the console go through the host PC.
function sendGcode(lines, from) {
  if (!m) return;
  for (const line of lines) {
    addLog('cmd', `> ${line}`);
    m.host.send(line);
  }
  if (!running) {
    setRunning(true);
    addLog('info', 'Simulation started so the command can run. Press Pause (space) to stop it.');
  }
  if (from === 'control' && !m.firmware.homed && lines.some((l) => /^(G0|G1|G38|M6|M620)/i.test(l))) {
    addLog('info', 'Note: the machine is not homed yet, so the firmware will ignore this move. Home first (G28).');
  }
}

function setDoor(open) {
  if (!m?.enclosure) return;
  m.enclosure.setDoor(open);
  addLog('info', open ? 'You opened the enclosure door.' : 'You shut the enclosure door.');
  control.refresh();
  updateStatus();
}

// ------------------------------------------------------------------ top bar
$('btnRun').addEventListener('click', () => setRunning(!running));
$('speed').addEventListener('change', (e) => { speed = e.target.value === 'max' ? 'max' : Number(e.target.value); });
$('btnReset').addEventListener('click', () => resetMachine());
function resetMachine() {
  setRunning(false);
  loadedJob = null;
  createMachine();
  defaultHint();
}
$('btnTheme').addEventListener('click', () => {
  theme = THEMES[(THEMES.indexOf(theme) + 1) % THEMES.length];
  storageSet('multiarm.theme', theme);
  applyThemeAttr();
  applyTheme();
  paintChannelColours();
});

function setMode(mode, { rebuild = true } = {}) {
  simMode = mode;
  storageSet('multiarm.mode', mode);
  for (const b of $('modeSeg').querySelectorAll('button')) b.setAttribute('aria-checked', String(b.dataset.mode === mode));
  $('modeNote').innerHTML = MODE_TEXT[mode];
  if (!rebuild) return;
  setRunning(false);
  if (loadedJob) {
    addLog('info', `Switched to ${mode === 'demo' ? 'demo mode' : 'full physics'}; reloading ${loadedJob.name}.`, 0);
    loadJob(loadedJob.text, loadedJob.name, loadedJob.def);
  } else {
    createMachine();
    defaultHint();
  }
}
$('modeSeg').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-mode]');
  if (b && b.dataset.mode !== simMode) setMode(b.dataset.mode);
});
radioKeys($('modeSeg'));

// ------------------------------------------------------------------ setup options
$('quick').addEventListener('change', () => { if (m && m.t === 0 && !job) createMachine(); else addLog('info', 'Quick start takes effect on Reset or when a job is loaded.'); });
$('preheated').addEventListener('change', () => { if (m && m.t === 0 && !job) createMachine(); else addLog('info', 'Preheated takes effect on Reset or when a job is loaded.'); });
$('stock').addEventListener('change', () => {
  if (m && m.t === 0 && !job) createMachine();
  else addLog('info', 'Stock change takes effect on Reset or when a job is loaded.');
});

// ------------------------------------------------------------------ jobs
const jobTexts = new Map();
const jobCards = renderJobs($('jobList'), JOBS, (def) => loadCatalogJob(def));

async function fetchJob(def) {
  if (jobTexts.has(def.file)) return jobTexts.get(def.file);
  const res = await fetch(`../scenarios/${def.file}`);
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  const text = await res.text();
  jobTexts.set(def.file, text);
  return text;
}
// Fetch every job once: fills in the time estimates and reveals optional
// jobs (the feature tour) only when their file exists.
for (const def of JOBS) {
  fetchJob(def).then((text) => {
    const card = jobCards.get(def.file);
    card.hidden = false;
    const est = def.machineTime ?? estimateTime(text, { homing: false });
    card.querySelector('.job-time').textContent = `${fmtEstimate(est)} machine time`;
  }).catch(() => {
    const card = jobCards.get(def.file);
    if (def.optional) card.hidden = true;
    else card.querySelector('.job-time').textContent = 'not available';
  });
}

async function loadCatalogJob(def) {
  try {
    const text = await fetchJob(def);
    loadJob(text, def.title, def);
  } catch (err) {
    addLog('warn', `Could not load ${def.file}: ${err.message}`);
    showTab('console');
  }
}

// Load a job: reset the machine, put any stock the job asks for on the plate,
// queue the G-code in the host PC and draw the planned path on the plate.
function loadJob(text, name, def = null) {
  setRunning(false);
  // A job that needs stock but declares none gets its default block, unless
  // the setup already puts one on the plate.
  let stock;
  if (def?.stock && !$('stock').value && !parseDirectives(text).stock.length) stock = def.stock;
  createMachine({ stock });
  if (stock) addLog('info', `Placed a 30 × 30 × 8 mm ${stock} block at X −30 mm: this job expects one.`);
  try {
    const d = applyDirectives(m, text);
    for (const st of d.stock) addLog('info', `Stock placed: ${st.material} ${st.size.join(' × ')} mm at (${st.centre.join(', ')}) mm.`);
  } catch (err) { addLog('warn', err.message); }
  view.updateWorkpiece?.();
  m.host.load(text);
  job = jobInfo(text, name);
  loadedJob = { text, name, def };
  $('jobCard').hidden = false;
  $('speed').value = 'max'; speed = 'max';
  setCamMode('work');
  const tp = toolpathOf(text);
  view.setToolpath(tp.segs, tp.kinds);
  view.toolpathVisible = layers.toolpath;
  addLog('info', `Loaded ${name}: ${m.host.lines.length} lines queued in the host (${tp.kinds.length} path segments).`);
  setHint(`<b>${escapeHtml(name)}</b> is loaded. Press <b>Run</b> (or Space).` +
    (m.firmware.cfg.startHomed ? '' : ' The machine first homes each joint (about 20 s).'));
  markLoadedCard();
  updateJob();
}

function markLoadedCard() {
  for (const [file, card] of jobCards) {
    const on = !!(job && loadedJob?.def?.file === file);
    card.classList.toggle('loaded', on);
    card.querySelector('.job-mini').hidden = !on;
    const btn = card.querySelector('.job-load');
    btn.textContent = on ? 'Reload' : 'Load';
  }
}

// Planned path in plate coordinates (the job's X/Y/Z), from the G-code text.
function toolpathOf(text) {
  const segs = [], kinds = [];
  let x = 0, y = 0, z = 50, e = 0, absXYZ = true, relE = false, milling = false, have = false;
  for (const raw of text.split('\n')) {
    const line = raw.replace(/;.*$/, '').trim().toUpperCase();
    if (!line) continue;
    if (line.startsWith('G90')) absXYZ = true; else if (line.startsWith('G91')) absXYZ = false;
    else if (line.startsWith('M83')) relE = true; else if (line.startsWith('M82')) relE = false;
    else if (/^M6\b/.test(line)) milling = /T1/.test(line);
    const mm = line.match(/^G([01])\b/);
    if (!mm) continue;
    const val = (k) => { const r = line.match(new RegExp(`${k}(-?[\\d.]+)`)); return r ? +r[1] : null; };
    const X = val('X'), Y = val('Y'), Z = val('Z'), E = val('E');
    const nx = X === null ? x : absXYZ ? X : x + X, ny = Y === null ? y : absXYZ ? Y : y + Y, nz = Z === null ? z : absXYZ ? Z : z + Z;
    if (have && (nx !== x || ny !== y || nz !== z)) {
      const extrudes = mm[1] === '1' && E !== null && (relE ? E > 0 : E > e);
      const kind = extrudes ? 1 : milling && mm[1] === '1' ? 2 : 0;
      segs.push(x / 1000, y / 1000, z / 1000, nx / 1000, ny / 1000, nz / 1000);
      kinds.push(kind);
    }
    if (E !== null) e = relE ? e + E : E;
    x = nx; y = ny; z = nz; have = true;
  }
  return { segs: Float32Array.from(segs), kinds: Uint8Array.from(kinds) };
}

// Slice an STL in the page and load it as a print job (polar mode, placed
// off the plate centre so the plate never has to flip half a turn).
$('stlFile').addEventListener('change', async (e) => {
  const f = e.target.files?.[0];
  e.target.value = '';
  if (!f) return;
  try {
    const tris = centreOnPlate(parseSTL(await f.arrayBuffer()));
    const { lo, hi } = meshBounds(tris);
    const size = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    if (Math.max(size[0], size[1]) > 40 || size[2] > 40) addLog('warn', `${f.name} is ${size.map((v) => v.toFixed(1)).join(' × ')} mm; parts over 40 mm may leave the arm's reach.`);
    const t0 = performance.now();
    const place = [Math.max(size[0], size[1]) / 2 + 10, 0];
    const sliced = sliceMesh(tris, { place });
    const out = printGcode(sliced, {
      preamble: [`; ${f.name}, sliced in the browser`, 'G28', 'M104 S210', 'M6 T0', 'M109 S210', 'G90', 'M83',
        `G0 X${place[0] + 5} Y0 Z5`, 'M620'],
      postamble: ['G0 Z15', 'M621', 'M104 S0', 'G0 X0 Y0 Z40', 'M114'],
    });
    addLog('info', `Sliced ${f.name} in ${Math.round(performance.now() - t0)} ms: ${out.stats.layers} layers, ${out.stats.filament_mm.toFixed(0)} mm filament, about ${Math.round(out.stats.time_s / 60)} min of printing.`);
    loadJob(out.lines.join('\n'), f.name);
  } catch (err) {
    addLog('warn', `Could not slice ${f.name}: ${err.message}`);
    showTab('console');
  }
});

// ------------------------------------------------------------------ layers
const LAYER_BTNS = { current: 'layCurrent', toolpath: 'layToolpath', couplings: 'layCouplings', enclosure: 'layEnclosure' };
function applyLayers() {
  if (!view) return;
  view.colorByCurrent = layers.current;
  view.showCouplings = layers.couplings;
  view.toolpathVisible = layers.toolpath;
  // The enclosure layer: the scene's own switch when it has one, otherwise
  // (older scene) hide or show the enclosure's box, hidden by default so it
  // does not hide the machine.
  const hasEnc = 'showEnclosure' in view;
  const encMeshes = hasEnc ? null : view.componentMeshes?.get?.('enclosure');
  if (!hasEnc && encMeshes && !layers.encFallbackSet) { layers.enclosure = false; layers.encFallbackSet = true; }
  $('layEnclosure').hidden = !hasEnc && !encMeshes;
  if (hasEnc) view.showEnclosure = layers.enclosure;
  else if (encMeshes) for (const o of encMeshes) o.visible = layers.enclosure;
  for (const [k, id] of Object.entries(LAYER_BTNS)) $(id).setAttribute('aria-pressed', String(layers[k]));
  updateLegend();
}
function toggleLayer(k) {
  if (k === 'enclosure' && $('layEnclosure').hidden) return;
  layers[k] = !layers[k];
  applyLayers();
}
for (const [k, id] of Object.entries(LAYER_BTNS)) $(id).addEventListener('click', () => toggleLayer(k));
function updateLegend() {
  const lg = $('legend');
  lg.hidden = !layers.current;
  lg.innerHTML = 'wire current <span>0</span><span class="ramp"></span><span>≥ 2 A</span>';
}

// ------------------------------------------------------------------ console input
const history = [];
let histIdx = -1;
$('gform').addEventListener('submit', (e) => {
  e.preventDefault();
  const input = $('gline');
  const line = input.value.trim();
  if (!line) return;
  history.push(line); histIdx = history.length;
  sendGcode([line], 'console');
  input.value = '';
});
$('gline').addEventListener('keydown', (e) => {
  if (e.key === 'ArrowUp' && history.length) { histIdx = Math.max(0, histIdx - 1); e.target.value = history[histIdx]; e.preventDefault(); }
  else if (e.key === 'ArrowDown' && history.length) { histIdx = Math.min(history.length, histIdx + 1); e.target.value = history[histIdx] ?? ''; e.preventDefault(); }
  else if (e.key === 'Escape') e.target.blur();
});

// ------------------------------------------------------------------ tabs
const TABS = ['inspect', 'explore', 'control', 'signals', 'console'];
let activeTab = 'inspect';
function showTab(name) {
  activeTab = name;
  for (const b of document.querySelectorAll('.tab')) {
    const on = b.dataset.tab === name;
    b.setAttribute('aria-selected', String(on));
    b.tabIndex = on ? 0 : -1;
  }
  for (const p of document.querySelectorAll('.tabpane')) {
    const on = p.id === `tab-${name}`;
    p.classList.toggle('active', on);
    p.hidden = !on;
  }
  if (name === 'console') { unread = 0; $('logBadge').hidden = true; logEl.scrollTop = logEl.scrollHeight; }
  if (name === 'explore') explorer.reveal();
  if (name === 'control') control.refresh();
}
document.querySelectorAll('.tab').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
document.querySelector('.tabs').addEventListener('keydown', (e) => {
  const i = TABS.indexOf(activeTab);
  let n = null;
  if (e.key === 'ArrowRight') n = TABS[(i + 1) % TABS.length];
  else if (e.key === 'ArrowLeft') n = TABS[(i + TABS.length - 1) % TABS.length];
  else if (e.key === 'Home') n = TABS[0];
  else if (e.key === 'End') n = TABS[TABS.length - 1];
  if (n) { e.preventDefault(); showTab(n); $(`tabbtn-${n}`).focus(); }
});
$('groupSeg').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-group]');
  if (!b) return;
  for (const x of $('groupSeg').querySelectorAll('button')) x.setAttribute('aria-checked', String(x === b));
  explorer.setMode(b.dataset.group);
});
radioKeys($('groupSeg'));

// Arrow keys move between the options of a segmented radio group.
function radioKeys(group) {
  group.addEventListener('keydown', (e) => {
    if (e.key !== 'ArrowRight' && e.key !== 'ArrowLeft') return;
    const bs = [...group.querySelectorAll('button')];
    const i = bs.indexOf(document.activeElement);
    if (i < 0) return;
    e.preventDefault();
    const n = bs[(i + (e.key === 'ArrowRight' ? 1 : bs.length - 1)) % bs.length];
    n.focus(); n.click();
  });
}

// ------------------------------------------------------------------ scope strip
function setScopeOpen(open) {
  $('scopeStrip').classList.toggle('collapsed', !open);
  $('btnScope').setAttribute('aria-expanded', String(open));
  storageSet('multiarm.scopeOpen', open ? '1' : '0');
}
setScopeOpen(storageGet('multiarm.scopeOpen') !== '0');
$('btnScope').addEventListener('click', () => setScopeOpen($('scopeStrip').classList.contains('collapsed')));
$('btnScopeCfg').addEventListener('click', () => showTab('signals'));
$('chanChips').addEventListener('click', (e) => {
  const c = e.target.closest('.chanchip');
  if (!c) return;
  showTab('signals');
  document.querySelectorAll('#tab-signals .ch')[+c.dataset.k]?.focus();
});

// ------------------------------------------------------------------ picking
const raycaster = new THREE.Raycaster();
raycaster.params.Line.threshold = 0.0035;
const ndc = new THREE.Vector2();
let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4 || e.button !== 0) return;
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  // Line pick threshold scales with distance so thin wires stay clickable.
  raycaster.params.Line.threshold = Math.max(0.002, camera.position.distanceTo(controls.target) * 0.004);
  scene.updateMatrixWorld();
  select(view.pick(raycaster));
});

// ------------------------------------------------------------------ job progress
// Executable lines of a job (as the host sends them) with the section each
// belongs to, taken from the job's comments ("; layer 5/22", "; finishing ...").
function jobInfo(text, name) {
  const sections = [];
  let label = '';
  for (const raw of text.split(/\r?\n/)) {
    const c = raw.match(/^\s*;\s*(layer .*|roughing .*|finishing.*|pass .*|step .*)$/i);
    if (c) { label = c[1].replace(/, z=.*$/, ''); continue; }
    if (raw.replace(/;.*$/, '').trim()) sections.push(label);
  }
  return { name, sections, total: sections.length, rate: [], done: false };
}
function updateJob() {
  const act = m?.firmware?.activity ?? '';
  $('tbActivity').innerHTML = m ? [running ? '' : '<b>Paused</b>', escapeHtml(act)].filter(Boolean).join(' · ') : '';
  if (!job || !m) return;
  const fw = m.firmware;
  const started = Math.min(job.total, Math.max(0, fw.stats.lines - fw.queue.length));
  const finished = m.host.done && !fw.current && !fw.queue.length;
  const pct = finished ? 100 : (100 * started) / Math.max(1, job.total);
  $('jobName').textContent = job.name;
  $('jobPct').textContent = `${pct.toFixed(pct < 10 ? 1 : 0)} %`;
  $('jobBar').style.width = `${pct}%`;
  const mini = loadedJob?.def ? jobCards.get(loadedJob.def.file)?.querySelector('.job-mini span') : null;
  if (mini) mini.style.width = `${pct}%`;
  $('jobAct').textContent = finished ? 'Job finished' : running ? act : `Paused: ${act}`;
  // A job's own narration (M118 lines), e.g. the feature tour.
  const cap = fw.caption ?? '';
  const capEl = $('jobCaption');
  if (capEl.textContent !== cap) capEl.textContent = cap;
  capEl.hidden = !cap;
  const sec = job.sections[Math.max(0, started - 1)];
  $('jobSec').textContent = sec ? `${sec.charAt(0).toUpperCase()}${sec.slice(1)}` : '';
  // Remaining time from the recent rate of lines per simulated second.
  job.rate.push([m.t, started]);
  while (job.rate.length > 2 && m.t - job.rate[0][0] > 20) job.rate.shift();
  const [t0, n0] = job.rate[0];
  const lps = m.t - t0 > 2 ? (started - n0) / (m.t - t0) : 0;
  if (finished) {
    $('jobEta').textContent = `Done in ${fmtDur(m.t)} of machine time`;
    if (!job.done) { job.done = true; addLog('info', `${job.name} finished after ${fmtDur(m.t)} of machine time.`); }
  } else if (lps > 0.5) {
    const simLeft = (job.total - started) / lps;
    const wallLeft = Number.isFinite(rtf.value) && rtf.value > 0.01 ? simLeft / rtf.value : NaN;
    $('jobEta').textContent = `Machine time ${fmtDur(m.t)}, about ${fmtDur(simLeft)} to go` + (Number.isFinite(wallLeft) ? ` (≈ ${fmtDur(wallLeft)} here at ${rtf.value.toFixed(2)}×)` : '');
  } else $('jobEta').textContent = `Machine time ${fmtDur(m.t)}`;
}
function fmtDur(s) {
  if (!Number.isFinite(s)) return '—';
  const m_ = Math.floor(s / 60), ss = Math.round(s % 60);
  return m_ ? `${m_} min ${String(ss).padStart(2, '0')} s` : `${ss} s`;
}

// ------------------------------------------------------------------ camera presets
let camMode = 'overview';
const camGoal = { pos: null, target: null };
function setCamMode(mode) {
  camMode = mode;
  for (const b of $('camSeg').querySelectorAll('button')) b.setAttribute('aria-checked', String(b.dataset.cam === mode));
  if (mode === 'overview') { camGoal.pos = new THREE.Vector3(0.64, -0.6, 0.46); camGoal.target = new THREE.Vector3(0.1, 0, 0.15); }
  else if (mode === 'work') {
    const c = m.spec.table.center;
    camGoal.target = new THREE.Vector3(c[0] - 0.02, c[1], c[2] + 0.03);
    camGoal.pos = new THREE.Vector3(c[0] + 0.2, c[1] - 0.26, c[2] + 0.2);
  } else camGoal.pos = null;
}
function updateCamera() {
  if (!m) return;
  if (camMode === 'follow') {
    const tip = m.toolTip();
    const tgt = new THREE.Vector3(tip[0], tip[1], tip[2]);
    const off = camera.position.clone().sub(controls.target);
    if (off.length() > 0.35) off.setLength(0.35);
    controls.target.lerp(tgt, 0.15);
    camera.position.copy(controls.target).add(off);
    return;
  }
  if (!camGoal.pos) return;
  controls.target.lerp(camGoal.target, 0.12);
  camera.position.lerp(camGoal.pos, 0.12);
  if (camera.position.distanceTo(camGoal.pos) < 1e-3) camGoal.pos = null;
}
$('camSeg').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-cam]');
  if (b) setCamMode(b.dataset.cam);
});
radioKeys($('camSeg'));
// Grabbing the view with the mouse ends an automatic camera move.
renderer.domElement.addEventListener('pointerdown', () => { if (camMode !== 'follow') camGoal.pos = null; });

// ------------------------------------------------------------------ status
function setStat(id, text, cls = '') {
  const e = $(id);
  if (e.textContent !== text) e.textContent = text;
  if (e.className !== cls) e.className = cls;
}
function updateStatus() {
  updateJob();
  const s = m.status();
  setStat('st-t', `${s.t.toFixed(3)} s`);
  setStat('st-rtf', Number.isFinite(rtf.value) ? `${rtf.value.toFixed(2)}×` : (running ? '…' : 'paused'));
  setStat('st-mode', simMode === 'demo' ? 'demo' : 'full');
  setStat('st-hot', `${s.hotend_C.toFixed(1)} °C`);
  setStat('st-bed', `${s.bed_C.toFixed(1)} °C`);
  setStat('st-tool', { hotend: 'hot end', spindle: 'spindle', probe: 'probe', none: 'none' }[s.tool] ?? s.tool);
  setStat('st-tip', Number.isFinite(s.tipError_mm) ? `${s.tipError_mm.toFixed(3)} mm` : '—');
  setStat('st-rpm', `${s.spindle_rpm.toFixed(0)} rpm`);
  setStat('st-psu', `${s.V_bus.toFixed(2)} V  ${fmtNum(s.I_psu, 'A')}`);
  setStat('st-homed', s.homed ? 'yes' : 'no', s.homed ? 'good' : 'warnc');
  setStat('st-queue', String(s.queue));
  const enc = m.enclosure;
  if (enc) {
    setStat('st-chamber', `${(enc.T - 273.15).toFixed(1)} °C`);
    setStat('st-door', enc.doorOpen ? 'open' : 'closed', enc.doorOpen ? 'alert' : '');
  }
}

// ------------------------------------------------------------------ help, tour, shortcuts
const tour = new Tour({
  root: $('tour'), spot: $('tourSpot'), card: $('tourCard'), stepEl: $('tourStep'), titleEl: $('tourTitle'), textEl: $('tourText'),
  dots: $('tourDots'), back: $('tourBack'), next: $('tourNext'), skip: $('tourSkip'),
});
function openHelp() {
  $('help').hidden = false;
  $('helpClose').focus();
}
function closeHelp() { $('help').hidden = true; $('btnHelp').focus(); }
$('btnHelp').addEventListener('click', openHelp);
$('helpClose').addEventListener('click', closeHelp);
$('help').addEventListener('click', (e) => { if (e.target === $('help')) closeHelp(); });
$('helpTour').addEventListener('click', () => { $('help').hidden = true; tour.open(0); });

window.addEventListener('keydown', (e) => {
  if (tour.isOpen) return;
  if (!$('help').hidden) { if (e.key === 'Escape') closeHelp(); return; }
  const tag = document.activeElement?.tagName;
  if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key;
  if (e.code === 'Space') {
    if (tag === 'BUTTON' && document.activeElement !== $('btnRun')) return; // let buttons click
    e.preventDefault(); setRunning(!running); return;
  }
  if (k === 'Escape') { select(null, { keepTab: true }); return; }
  if (k === '?') { e.preventDefault(); openHelp(); return; }
  if (k === 'R' && e.shiftKey) { resetMachine(); return; }
  if (k === '/') { e.preventDefault(); showTab('console'); $('gline').focus(); return; }
  const lower = k.toLowerCase();
  const tabKey = { i: 'inspect', e: 'explore', c: 'control', s: 'signals', l: 'console' }[lower];
  if (tabKey && !e.shiftKey) { showTab(tabKey); return; }
  const layerKey = { w: 'current', t: 'toolpath', k: 'couplings', n: 'enclosure' }[lower];
  if (layerKey && !e.shiftKey) { toggleLayer(layerKey); return; }
  if (lower === 'b') { setScopeOpen($('scopeStrip').classList.contains('collapsed')); return; }
  const cam = { 1: 'overview', 2: 'work', 3: 'follow' }[k];
  if (cam) setCamMode(cam);
});

// ------------------------------------------------------------------ main loop
// Physics runs on the main thread. Each animation frame spends up to
// FRAME_BUDGET_MS advancing the sim, paced by the wall clock and the chosen
// speed, then draws. If animation frames arrive late (software GL, a slow
// compositor), a small message-channel pump keeps physics going between them
// in 10 ms slices so the achieved speed does not collapse with the frame rate.
const perf = { physMs: 0, frames: 0 };
const pace = { lastWall: performance.now(), debt: 0 };
const MAX_BACKLOG_S = 0.05;   // at fixed speeds, drop sim time we cannot catch up on

function advance(budgetMs) {
  if (!running || !m) return;
  const t0 = performance.now();
  let target = Infinity;
  if (speed !== 'max') {
    pace.debt = Math.min(pace.debt + (speed * (t0 - pace.lastWall)) / 1000, MAX_BACKLOG_S * Math.max(1, speed));
    target = pace.debt;
  }
  pace.lastWall = t0;
  const simStart = m.t;
  try {
    while (m.t - simStart < target - 1e-9 && performance.now() - t0 < budgetMs) {
      m.run(Math.min(CHUNK_S, target - (m.t - simStart)));
    }
  } catch (err) {
    setRunning(false);
    addLog('warn', `Simulation error: ${err.message}`);
    console.warn(err);
  }
  const adv = m.t - simStart;
  if (speed !== 'max') pace.debt -= adv;
  rtf.sim += adv;
  perf.physMs += performance.now() - t0;
}

let lastRaf = performance.now(), pumpScheduled = false;
const pumpChannel = new MessageChannel();
pumpChannel.port1.onmessage = pump;
function schedulePump() {
  if (pumpScheduled || !running) return;
  pumpScheduled = true;
  if (performance.now() - lastRaf > 24) pumpChannel.port2.postMessage(0);
  else setTimeout(pump, 8);
}
function pump() {
  pumpScheduled = false;
  if (!running) return;
  if (performance.now() - lastRaf > 24) advance(10);
  schedulePump();
}

let lastSlow = 0, lastScope = 0;
function frame(now) {
  requestAnimationFrame(frame);
  lastRaf = performance.now();
  perf.frames++;
  if (running) {
    advance(speed === 'max' ? MAX_BUDGET_MS : FRAME_BUDGET_MS);
    pollPlantLog();
    const wall = (performance.now() - rtf.t0) / 1000;
    if (wall >= 0.5) { rtf.value = rtf.sim / wall; rtf.sim = 0; rtf.t0 = performance.now(); }
    schedulePump();
  }
  if (now - lastScope > 50 && !$('scopeStrip').classList.contains('collapsed')) { lastScope = now; scope.draw(); }
  if (now - lastSlow > 200) {
    lastSlow = now;
    updateStatus();
    inspector.refresh();
    if (activeTab === 'control') control.refresh();
  }
  if (view) view.update();
  updateCamera();
  controls.update();
  renderer.render(scene, camera);
}

setMode(simMode, { rebuild: false });
createMachine();
defaultHint();
showTab('inspect');
requestAnimationFrame(frame);
if (Tour.shouldAutoStart()) setTimeout(() => tour.open(0), 400);

// Handy for debugging from the devtools console.
window.sim = { perf, get m() { return m; }, get view() { return view; }, select, setRunning, scene, camera, controls, tour, loadJob, showTab };
