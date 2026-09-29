#!/usr/bin/env node
// Headless runner: stream a G-code file into the simulated machine and report.
//
//   node src/headless/run.js scenarios/demo.gcode [--preheated] [--quick] [--demo] [--max 600] [--every 5] [--stock wax]
//
//   --quick  joints start referenced at the rest pose (G28 skipped)
//   --demo   demo mode: ideal drives, rigid gearboxes, 1 ms steps (about 20x faster)
//
// Writes out/heightmap.pgm (the part as a grey-scale height image) and
// out/summary.json.

import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { Machine } from '../machine/machine.js';
import { MATERIALS } from '../process/workpiece.js';
import { applyDirectives } from '../cam/scenario.js';

const args = process.argv.slice(2);
const opt = (name, def) => {
  const i = args.indexOf(`--${name}`);
  if (i < 0) return def;
  const v = args[i + 1];
  return v === undefined || v.startsWith('--') ? true : v;
};
const file = args.find((a) => !a.startsWith('--') && a.endsWith('.gcode')) ?? 'scenarios/demo.gcode';
const maxT = Number(opt('max', 900));
const every = Number(opt('every', 5));

const m = new Machine({ preheated: !!opt('preheated', false), startHomed: !!opt('quick', false), mode: opt('demo', false) ? 'demo' : 'full' });
if (m.problems.length) console.log('Assembly problems:\n  ' + m.problems.join('\n  '));
const stock = opt('stock', null);
if (stock && MATERIALS[stock]) m.workpiece.addStock(MATERIALS[stock], 0.03, 0.03, 0.008, -0.03, 0);

m.host.onMessage = (msg) => console.log(`  [fw ${m.t.toFixed(3)}s] ${msg}`);
const text = readFileSync(file, 'utf8');
const dir = applyDirectives(m, text); // e.g. "; @stock wax 30 30 8 -30 0"
for (const s of dir.stock) console.log(`Stock: ${s.material} ${s.size.join(' x ')} mm at (${s.centre.join(', ')}) mm`);
m.host.load(text);
console.log(`Running ${file} (${m.host.lines.length} lines)`);

// --snapshot: save the part's height map each time a tool locks on, so the
// state before each process step can be compared with the result.
let lastTool = null;
const snapshot = !!opt('snapshot', false);
const tagOf = (f) => f.replace(/^.*\//, '').replace(/\.gcode$/, '');
const wall0 = Date.now();
let lastLog = 0, tipErrMax = 0, tipErrSum = 0, tipErrN = 0, idle = 0, partErrMax = 0;
while (m.t < maxT) {
  m.run(0.25);
  const e = m.tipError();
  if (Number.isFinite(e) && m.master.tool) { tipErrMax = Math.max(tipErrMax, e); tipErrSum += e; tipErrN++; }
  const pe = m.partError();
  if (Number.isFinite(pe) && m.master.tool) partErrMax = Math.max(partErrMax, pe);
  const tool = m.master.tool?.toolName ?? null;
  if (snapshot && tool !== lastTool && tool) {
    mkdirSync('out', { recursive: true });
    writeFileSync(`out/heights-${tagOf(file)}-before-${tool}.bin`, Buffer.from(m.workpiece.h.slice().buffer));
  }
  lastTool = tool;
  for (; lastLog < m.log.length; lastLog++) console.log(`  [plant ${m.log[lastLog][0].toFixed(3)}s] ${m.log[lastLog][1]}`);
  if (Math.abs(m.t / every - Math.round(m.t / every)) < 1e-6) {
    const s = m.status();
    console.log(`t=${s.t.toFixed(1)}s  q=[${s.q_deg.map((x) => x.toFixed(1)).join(', ')}] table=${s.table_deg.toFixed(1)}  tool=${s.tool}` +
      `  hotend=${s.hotend_C.toFixed(1)}C  bus=${s.V_bus.toFixed(2)}V ${s.I_psu.toFixed(2)}A  tipErr=${Number.isFinite(s.tipError_mm) ? s.tipError_mm.toFixed(2) : '-'}mm partErr=${Number.isFinite(s.partError_mm) ? s.partError_mm.toFixed(2) : '-'}mm` +
      `  spindle=${s.spindle_rpm.toFixed(0)}rpm  queue=${s.queue}  (x${(m.t / ((Date.now() - wall0) / 1000)).toFixed(2)} real time)`);
  }
  const fw = m.firmware;
  if (m.host.done && !fw.current && fw.queue.length === 0) { if (++idle > 8) break; } else idle = 0;
}
const wall = (Date.now() - wall0) / 1000;

mkdirSync('out', { recursive: true });
const wp = m.workpiece;
let maxH = 0;
for (const h of wp.h) maxH = Math.max(maxH, h);
const img = Buffer.alloc(wp.n * wp.n);
for (let j = 0; j < wp.n; j++)
  for (let i = 0; i < wp.n; i++) img[(wp.n - 1 - j) * wp.n + i] = maxH > 0 ? Math.round((255 * wp.h[j * wp.n + i]) / maxH) : 0;
writeFileSync('out/heightmap.pgm', Buffer.concat([Buffer.from(`P5\n${wp.n} ${wp.n}\n255\n`), img]));

const summary = {
  simTime_s: m.t, wall_s: wall, speed_x: m.t / wall,
  firmware: { homed: m.firmware.homed, moves: m.firmware.stats.moves, maxStepsPerTick: m.firmware.stats.maxStepsPerTick },
  tipError_mm: { max: tipErrMax * 1e3, mean: tipErrN ? (tipErrSum / tipErrN) * 1e3 : null },
  partFrameErrorMax_mm: partErrMax * 1e3,
  extruded_mm3: m.extruder.totalOut * 1e9, deposited_mm3: wp.volumeAdded * 1e9, removed_mm3: wp.volumeRemoved * 1e9,
  partMaxHeight_mm: maxH * 1e3,
  driverViolations: Object.fromEntries(m.driverList.map((d) => [d.id, d.violations])),
  events: m.log.map(([t, s]) => `${t.toFixed(3)} ${s}`),
  firmwareLog: m.firmware.log.map(([t, s]) => `${t.toFixed(3)} ${s}`),
  wires: m.wires.filter((w) => Math.abs(w.i) > 0.01).map((w) => ({ id: w.id, I: +w.i.toFixed(3), T: +(w.T - 273.15).toFixed(1) })),
};
const tag = tagOf(file);
writeFileSync(`out/heightmap-${tag}.pgm`, Buffer.concat([Buffer.from(`P5\n${wp.n} ${wp.n}\n255\n`), img]));
writeFileSync(`out/heights-${tag}.bin`, Buffer.from(wp.h.buffer));
writeFileSync(`out/summary-${tag}.json`, JSON.stringify(summary, null, 2));
writeFileSync('out/summary.json', JSON.stringify(summary, null, 2));
console.log('\nSummary:', JSON.stringify({ ...summary, events: summary.events.length, firmwareLog: summary.firmwareLog.length, wires: summary.wires.length }, null, 2));
