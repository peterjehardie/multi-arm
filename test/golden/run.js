// The golden run: fixed scenario, fixed signals. Shared by tools/golden.js
// (records) and test/architecture.test.js (compares).
import { Machine } from '../../src/machine/machine.js';
import { hashState } from '../../src/core/state.js';

export const SIGNALS = [
  ['bus_V', (m) => m.net.across(m.psu.port('V+'), m.psu.port('V-'))],
  ['j2_coilA_A', (m) => m.drivers.j2.loopA.i],
  ['q1', (m) => m.arm.q[0]], ['q2', (m) => m.arm.q[1]], ['q3', (m) => m.arm.q[2]], ['q4', (m) => m.arm.q[3]],
  ['table', (m) => m.table.theta],
  ['hotend_K', (m) => m.hotTherm.T],
  ['j2_windup', (m) => m.gearboxes.j2.deflection],
  ['servo', (m) => m.servo.angle],
];

export function goldenRun(gcode, seconds = 3, every = 0.05) {
  const m = new Machine({ preheated: true, startHomed: true });
  m.host.load(gcode);
  const samples = [];
  for (let k = 0; k < Math.round(seconds / every); k++) {
    m.run(every);
    samples.push(SIGNALS.map(([, f]) => f(m)));
  }
  return { scenario: 'demo.gcode, quick start, preheated', every, signals: SIGNALS.map(([n]) => n), samples, hash: hashState(m) };
}
