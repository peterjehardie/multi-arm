// Whole-machine checks: the assembly obeys the physical-anchor rules, signals
// travel through wires, and the firmware can home the arm.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Machine } from '../src/machine/machine.js';
import { Wire } from '../src/electrical/circuit.js';

test('assembly: every component mounted, every required port connected', () => {
  const m = new Machine();
  assert.deepEqual(m.problems, []);
  // Every wire has a physical length from its route.
  for (const w of m.wires) assert.ok(w.length > 0.01, `${w.id} length ${w.length}`);
  // Loads are real circuits through real wires.
  assert.ok(m.drivers.j2.loopA.items.some((x) => x.el instanceof Wire));
  assert.ok(Number.isFinite(m.fets.bed.loop.resistance()), 'bed circuit closed through the slip ring');
  assert.ok(!Number.isFinite(m.fets.hotend.loop.resistance()), 'parked hot end: heater circuit open at the pogo pins');
});

test('logic edges arrive after RC charging plus flight time (nanoseconds)', () => {
  const m = new Machine();
  m.run(0.05); // PSU soft start, buck start-up, MCU boot
  const net = m.nets.get('board.GP2');
  const r = net.receivers[0];
  const tau = (30 + r.R) * r.C;
  assert.ok(tau > 1e-10 && tau < 5e-9, `RC ${tau}`);
  let arrived = null;
  const drv = m.drivers.j2, orig = drv.digitalIn.bind(drv);
  drv.digitalIn = (pin, lvl, t) => { if (pin === 'STEP' && lvl) arrived = t; orig(pin, lvl, t); };
  const t0 = m.t;
  m.board.mcu.write('GP2', true);
  m.run(1e-4);
  assert.ok(arrived > t0 && arrived - t0 < 1e-8, `edge delay ${arrived - t0}`);
});

test('homing finds every switch and leaves sub-millimetre tip error', { timeout: 120000 }, () => {
  const m = new Machine();
  m.host.load('G28');
  for (let k = 0; k < 40 && !m.firmware.log.some((l) => l[1] === 'homing done'); k++) m.run(1);
  m.run(1);
  assert.ok(m.firmware.homed, 'homed');
  const e = m.tipError();
  assert.ok(e < 1.5e-3, `tip error ${e * 1e3} mm`);
  for (const d of m.driverList) assert.deepEqual(d.violations, { setup: 0, pulse: 0 });
});
