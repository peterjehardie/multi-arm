// Demo mode, enclosure and touch probe: features exercised on the fast,
// simplified physics (ideal drives, rigid gearboxes, 1 ms steps). The same
// parts, wires and firmware run, so these also check the wiring of the
// door switch, the fan and the probe.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Machine } from '../src/machine/machine.js';
import { applyDirectives } from '../src/cam/scenario.js';

const quick = (opts = {}) => new Machine({ mode: 'demo', preheated: true, startHomed: true, ...opts });
const runUntil = (m, cond, tMax) => { while (!cond() && m.t < tMax) m.run(0.1); return cond(); };

test('demo mode: the arm follows its step pulses and runs much faster than full physics', { timeout: 60000 }, () => {
  const m = quick();
  assert.equal(m.sim.dt, 1e-3);
  m.host.load('G90\nG0 X20 Y10 Z30 F3000\nG0 X-10 Y-20 Z15\nM114');
  const w0 = performance.now();
  assert.ok(runUntil(m, () => m.firmware.log.some((l) => l[1].startsWith('X:')), 20), 'moves finished');
  m.run(0.2);
  const wall = (performance.now() - w0) / 1000;
  // Moved by step pulses through the drivers, coils and gearboxes.
  assert.ok(Math.abs(m.drivers.j1.steps) > 1000, `j1 steps ${m.drivers.j1.steps}`);
  // Rigid, backlash-free drives: only the seeded build tolerances remain.
  const tip = m.firmwareTip(), real = m.toolTip();
  const err = Math.hypot(tip[0] - real[0], tip[1] - real[1], tip[2] - real[2]);
  assert.ok(err < 0.5e-3, `tip error ${err * 1e3} mm`);
  assert.ok(m.t / wall > 4, `speed x${(m.t / wall).toFixed(1)}`);
});

test('door interlock: the spindle waits while the door is open', { timeout: 60000 }, () => {
  const m = quick();
  m.host.load('M6 T1\nM118 @door open 3\nG4 P200\nM3 S8000\nG4 P3000\nM5');
  assert.ok(runUntil(m, () => m.enclosure.doorOpen, 80), 'operator opened the door');
  const tOpen = m.t;
  assert.ok(m.firmware.doorOpen, 'door switch read through its wires');
  m.run(2);
  assert.ok(Math.abs(m.spindle.omega) < 1, `spindle held at ${m.spindle.omega} rad/s`);
  assert.ok(runUntil(m, () => !m.enclosure.doorOpen, tOpen + 5), 'door shut again');
  m.run(2.5);
  assert.ok(m.spindle.omega * 60 / (2 * Math.PI) > 5000, `spindle ${m.spindle.omega * 60 / (2 * Math.PI)} rpm`);
});

test('touch probe: measures a wax block and the plate within a few tenths of a millimetre', { timeout: 60000 }, () => {
  const m = quick();
  const job = '; @stock wax 20 20 8 -30 0\nM6 T2\nG90\nG0 X-30 Y0 Z14\nG38.2 Z4 F120\nG0 Z14\nG0 X20 Y0\nG38.2 Z-4 F120\nG0 Z14';
  applyDirectives(m, job);
  m.host.load(job);
  const prb = () => m.firmware.log.filter((l) => l[1].startsWith('PRB:')).map((l) => l[1].slice(4).split(':')[0].split(',').map(Number));
  assert.ok(runUntil(m, () => prb().length >= 2, 120), 'two probe points');
  const [top, plate] = prb();
  assert.ok(Math.abs(top[2] - 8) < 0.3, `block top ${top[2]} mm`);
  assert.ok(Math.abs(plate[2]) < 0.3, `plate ${plate[2]} mm`);
  assert.ok(Math.abs(top[2] - plate[2] - 8) < 0.15, `block height ${top[2] - plate[2]} mm`);
});

test('enclosure: a hot bed warms the chamber, the exhaust fan and an open door cool it', { timeout: 60000 }, () => {
  const run = (setup) => {
    const m = quick();
    setup(m);
    m.host.load('M140 S60');
    m.run(600);
    return m.enclosure.T - 273.15;
  };
  const shut = run(() => {});
  const fan = run((m) => m.host.send('M106 S255'));
  const open = run((m) => m.enclosure.setDoor(true));
  const room = 22;
  // About 11 W from the bed against 7 W/K of panels: slow and modest.
  assert.ok(shut > room + 1.2, `closed chamber ${shut.toFixed(1)} C`);
  assert.ok(fan < shut - 0.3, `fan ${fan.toFixed(1)} C vs ${shut.toFixed(1)} C`);
  assert.ok(open < shut - 0.4, `door open ${open.toFixed(1)} C vs ${shut.toFixed(1)} C`);
});

test('M118 narration reaches the host and the firmware caption', { timeout: 30000 }, () => {
  const m = quick();
  const seen = [];
  m.host.listeners.push((msg) => seen.push(msg));
  m.host.load('M118 Hello from the tour');
  m.run(0.5);
  assert.equal(m.firmware.caption, 'Hello from the tour');
  assert.ok(seen.includes('Hello from the tour'));
});
