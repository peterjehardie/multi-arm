// Browser viewer for the multi-arm simulation. The physics runs on the main
// thread inside requestAnimationFrame with a wall-clock budget per frame.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { Machine } from '../src/machine/machine.js';
import { MATERIALS } from '../src/process/workpiece.js';
import { applyDirectives } from '../src/cam/scenario.js';
import { parseSTL, centreOnPlate, meshBounds } from '../src/cam/mesh.js';
import { sliceMesh, printGcode } from '../src/cam/slicer.js';
import { MachineView } from './view3d.js';
import { Scope } from './scope.js';
import { Inspector, fmtNum } from './inspector.js';
import { docFor } from './docs.js';

const $ = (id) => document.getElementById(id);
const FRAME_BUDGET_MS = 12;   // wall time spent on physics per animation frame
const CHUNK_S = 0.002;        // sim time per m.run() call

// ------------------------------------------------------------------ three.js
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const viewportEl = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
viewportEl.prepend(renderer.domElement);
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

function applyTheme() {
  const css = getComputedStyle(document.documentElement);
  const bg = css.getPropertyValue('--scene-bg').trim() || '#11151b';
  scene.background = new THREE.Color(bg);
  const light = window.matchMedia('(prefers-color-scheme: light)').matches;
  benchMat.color.set(light ? '#c5ccd4' : '#1b2027');
  grid.material.color.set(light ? '#aab3bd' : '#2a323d');
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
window.matchMedia('(prefers-color-scheme: light)').addEventListener?.('change', applyTheme);

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
const rtf = { sim: 0, value: NaN, t0: performance.now() };

const scope = new Scope({
  canvas: $('scope'),
  selects: [...document.querySelectorAll('.scope .ch')],
  readouts: [...document.querySelectorAll('.scope .rd')],
  windowSelect: $('scopeWin'),
});
const inspector = new Inspector($('inspector'), { onSelect: (ref) => select(ref) });

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
  if (!$('tab-console').classList.contains('active') && cls !== 'info' && cls !== 'cmd') {
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

// ------------------------------------------------------------------ machine lifecycle
function createMachine() {
  if (view) { view.dispose(); view = null; }
  const preheated = $('preheated').checked;
  m = new Machine({ preheated });
  const stock = $('stock').value;
  if (stock && MATERIALS[stock]) m.workpiece.addStock(MATERIALS[stock], 0.03, 0.03, 0.008, -0.03, 0);
  m.host.onMessage = (msg) => addLog('fw', msg);
  lastPlant = null;
  view = new MachineView(m);
  view.colorByCurrent = $('byCurrent').checked;
  view.showCouplings = $('couplings').checked;
  view.toolpathVisible = $('toolpath').checked;
  scene.add(view.root);
  scope.attach(m, ['24 V bus at PSU', 'j2 coil A', 'hot end (true)', 'tip error']);
  inspector.attach(m);
  buildPartsList();
  const pr = $('problems');
  if (m.problems.length) {
    pr.hidden = false;
    pr.innerHTML = `<h4>Assembly problems (${m.problems.length})</h4><ul>${m.problems.map((p) => `<li>${escapeHtml(p)}</li>`).join('')}</ul>`;
    for (const p of m.problems) addLog('warn', `[assembly] ${p}`, 0);
  } else pr.hidden = true;
  addLog('info', `Machine built: ${m.A.components.size} components, ${m.A.connections.size} connections (${m.wires.length} wires)${preheated ? ', preheated' : ''}${stock ? `, ${stock} stock` : ''}.`, 0);
  if (selection && (selection.type === 'component' ? m.A.components.has(selection.id) : m.A.connections.has(selection.id))) select(selection);
  else select(null);
  rtf.sim = 0; rtf.t0 = performance.now(); rtf.value = NaN;
  updateStatus();
}

function escapeHtml(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]); }

function setRunning(on) {
  running = on;
  pace.lastWall = performance.now(); pace.debt = 0;
  rtf.sim = 0; rtf.t0 = performance.now(); rtf.value = NaN;
  if (on) schedulePump();
  $('btnRun').textContent = on ? 'Pause' : 'Run';
  $('btnRun').classList.toggle('running', on);
  if (on) $('hint').hidden = true;
}

function select(ref) {
  selection = ref;
  view?.setSelection(ref);
  inspector.show(ref);
  if (ref) showTab('inspector');
}

// ------------------------------------------------------------------ controls
$('btnRun').addEventListener('click', () => setRunning(!running));
$('speed').addEventListener('change', (e) => { speed = e.target.value === 'max' ? 'max' : Number(e.target.value); });
$('btnReset').addEventListener('click', () => { setRunning(false); createMachine(); $('hint').hidden = false; });
$('stock').addEventListener('change', () => {
  if (m && m.t === 0) createMachine();
  else addLog('info', 'Stock change takes effect on Reset.');
});
// Load a job: reset the machine, put any stock the job asks for on the plate,
// queue the G-code in the host PC and draw the planned path on the plate.
function loadJob(text, name) {
  setRunning(false);
  createMachine();
  try {
    const d = applyDirectives(m, text);
    for (const st of d.stock) addLog('info', `Stock placed: ${st.material} ${st.size.join(' x ')} mm at (${st.centre.join(', ')}) mm.`);
  } catch (err) { addLog('warn', err.message); }
  view.updateWorkpiece?.();
  m.host.load(text);
  const tp = toolpathOf(text);
  view.setToolpath(tp.segs, tp.kinds);
  addLog('info', `Loaded ${name}: ${m.host.lines.length} lines queued in the host (${tp.kinds.length} path segments).`);
  $('hint').innerHTML = `${escapeHtml(name)} loaded. Press <b>Run</b> (try speed <b>max</b>)`;
  $('hint').hidden = false;
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

$('btnDemo').addEventListener('click', async () => {
  const file = $('job').value;
  try {
    const res = await fetch(`../scenarios/${file}`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    loadJob(await res.text(), file);
  } catch (err) {
    addLog('warn', `Could not load ${file}: ${err.message}`);
  }
});

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
    if (Math.max(size[0], size[1]) > 40 || size[2] > 40) addLog('warn', `${f.name} is ${size.map((v) => v.toFixed(1)).join(' x ')} mm; parts over 40 mm may leave the arm's reach.`);
    const t0 = performance.now();
    const place = [Math.max(size[0], size[1]) / 2 + 10, 0];
    const sliced = sliceMesh(tris, { place });
    const job = printGcode(sliced, {
      preamble: [`; ${f.name}, sliced in the browser`, 'G28', 'M104 S210', 'M6 T0', 'M109 S210', 'G90', 'M83',
        `G0 X${place[0] + 5} Y0 Z5`, 'M620'],
      postamble: ['G0 Z15', 'M621', 'M104 S0', 'G0 X0 Y0 Z40', 'M114'],
    });
    addLog('info', `Sliced ${f.name} in ${Math.round(performance.now() - t0)} ms: ${job.stats.layers} layers, ${job.stats.filament_mm.toFixed(0)} mm filament, about ${Math.round(job.stats.time_s / 60)} min of printing.`);
    loadJob(job.lines.join('\n'), f.name);
  } catch (err) {
    addLog('warn', `Could not slice ${f.name}: ${err.message}`);
  }
});
$('toolpath').addEventListener('change', (e) => { view.toolpathVisible = e.target.checked; });
$('byCurrent').addEventListener('change', (e) => {
  view.colorByCurrent = e.target.checked;
  updateLegend();
});
$('couplings').addEventListener('change', (e) => { view.showCouplings = e.target.checked; });
function updateLegend() {
  const lg = $('legend');
  lg.hidden = !$('byCurrent').checked;
  lg.innerHTML = 'wire current <span>0</span><span class="ramp"></span><span>≥ 2 A</span>';
}

$('gform').addEventListener('submit', (e) => {
  e.preventDefault();
  const input = $('gline');
  const line = input.value.trim();
  if (!line) return;
  addLog('cmd', `> ${line}`);
  m.host.send(line);
  input.value = '';
});

document.querySelectorAll('.tab').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
function showTab(name) {
  document.querySelectorAll('.tab').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tabpane').forEach((p) => p.classList.toggle('active', p.id === `tab-${name}`));
  if (name === 'console') { unread = 0; $('logBadge').hidden = true; logEl.scrollTop = logEl.scrollHeight; }
}

window.addEventListener('keydown', (e) => {
  const tag = document.activeElement?.tagName;
  if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
  if (e.code === 'Space') { e.preventDefault(); setRunning(!running); }
  if (e.code === 'Escape') select(null);
});

// ------------------------------------------------------------------ parts list
function buildPartsList() {
  const root = $('parts');
  root.replaceChildren();
  const groups = new Map();
  const add = (group, kind, obj, type) => {
    const key = `${group}: ${docFor(kind).title}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push({ obj, type });
  };
  for (const c of m.A.components.values()) add('Part', c.kind, c, 'component');
  for (const c of m.A.connections.values()) add('Link', c.kind, c, 'connection');
  for (const [key, items] of [...groups].sort()) {
    const det = document.createElement('details');
    const sum = document.createElement('summary');
    sum.innerHTML = `<b>${escapeHtml(key.split(': ')[1])}</b> <span class="muted">(${items.length})</span>`;
    det.append(sum);
    for (const { obj, type } of items) {
      const a = document.createElement('a');
      a.href = '#'; a.className = 'item link';
      a.dataset.search = `${obj.label} ${obj.id} ${obj.kind}`.toLowerCase();
      a.innerHTML = `${obj.color ? `<span class="swatch" style="background:${obj.color}"></span>` : ''}${escapeHtml(obj.label)}<code>${escapeHtml(obj.id)}</code>`;
      a.addEventListener('click', (e) => { e.preventDefault(); select({ type, id: obj.id }); });
      det.append(a);
    }
    root.append(det);
  }
  filterParts();
}
function filterParts() {
  const q = $('partsFilter').value.trim().toLowerCase();
  for (const det of $('parts').children) {
    let any = false;
    for (const a of det.querySelectorAll('.item')) {
      const hit = !q || a.dataset.search.includes(q);
      a.hidden = !hit; any ||= hit;
    }
    det.hidden = !any;
    if (q) det.open = any;
  }
}
$('partsFilter').addEventListener('input', filterParts);

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

// ------------------------------------------------------------------ status
function updateStatus() {
  const s = m.status();
  $('st-t').textContent = `${s.t.toFixed(3)} s`;
  $('st-rtf').textContent = Number.isFinite(rtf.value) ? `${rtf.value.toFixed(2)}×` : (running ? '…' : 'paused');
  $('st-hot').textContent = `${s.hotend_C.toFixed(1)} °C`;
  $('st-bed').textContent = `${s.bed_C.toFixed(1)} °C`;
  $('st-tool').textContent = s.tool;
  $('st-tip').textContent = Number.isFinite(s.tipError_mm) ? `${s.tipError_mm.toFixed(3)} mm` : '—';
  $('st-rpm').textContent = `${s.spindle_rpm.toFixed(0)} rpm`;
  $('st-psu').textContent = `${s.V_bus.toFixed(2)} V  ${fmtNum(s.I_psu, 'A')}`;
  $('st-homed').textContent = s.homed ? 'yes' : 'no';
  $('st-queue').textContent = String(s.queue);
}

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
    advance(FRAME_BUDGET_MS);
    pollPlantLog();
    const wall = (performance.now() - rtf.t0) / 1000;
    if (wall >= 0.5) { rtf.value = rtf.sim / wall; rtf.sim = 0; rtf.t0 = performance.now(); }
    schedulePump();
  }
  if (now - lastScope > 50) { lastScope = now; scope.draw(); }
  if (now - lastSlow > 200) { lastSlow = now; updateStatus(); inspector.refresh(); }
  if (view) view.update();
  controls.update();
  renderer.render(scene, camera);
}

createMachine();
updateLegend();
requestAnimationFrame(frame);

// Handy for debugging from the devtools console.
window.sim = { perf, get m() { return m; }, get view() { return view; }, select, setRunning, scene, camera, controls };
