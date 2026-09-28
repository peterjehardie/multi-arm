// Electrical physics checks against hand calculations and datasheet numbers.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Assembly, Component, Body } from '../src/core/graph.js';
import { Wire, traceLoop, Resistor } from '../src/electrical/circuit.js';
import { DCNetwork } from '../src/electrical/dcnetwork.js';
import { PowerSupply } from '../src/electrical/power.js';
import { StepperMotor, StepperDriver } from '../src/electrical/stepper.js';
import { MOTORS } from '../src/machine/spec.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg}: ${a} vs ${b}`);
const world = new Body('world');
const mount = (x = 0) => ({ body: world, p: [x, 0, 0] });

function twoTerminal(id, R) {
  const c = new Component(id, { mount: mount() });
  c.addPort('a', 'elec', 'load'); c.addPort('b', 'elec', 'load');
  c.addThrough('a', 'b', new Resistor(R));
  return c;
}

test('wire resistance follows copper geometry (22 AWG ~ 52.9 mOhm/m)', () => {
  const A = new Assembly();
  const x = A.add(twoTerminal('x', 1)), y = A.add(twoTerminal('y', 1));
  const w = A.connect(new Wire('w', x.port('a'), y.port('a'), { awg: 22, length: 1, contactR: 0 }));
  w.measure();
  near(w.resistance(), 0.0529, 0.02, '22 AWG per metre');
  w.T += 50; // 50 K warmer: copper tempco 0.393 %/K -> +19.7 %
  near(w.resistance(), 0.0529 * 1.1965, 0.02, 'warm wire');
});

test('series loop sums wires and elements; RL current is an exact exponential', () => {
  const A = new Assembly();
  const src = A.add(twoTerminal('src', 0));
  const coil = new Component('coil', { mount: mount(1) });
  coil.addPort('a', 'elec', 'load'); coil.addPort('b', 'elec', 'load');
  coil.addThrough('a', 'b', { i: 0, resistance: () => 2, inductance: () => 4e-3, record(i) { this.i = i; } });
  A.add(coil);
  const w1 = A.connect(new Wire('w1', src.port('a'), coil.port('a'), { length: 1, awg: 22, contactR: 0 }));
  const w2 = A.connect(new Wire('w2', coil.port('b'), src.port('b'), { length: 1, awg: 22, contactR: 0 }));
  w1.measure(); w2.measure();
  const loop = traceLoop(src.port('a'), src.port('b'));
  assert.equal(loop.items.length, 3);
  const R = loop.resistance(), L = loop.inductance();
  near(R, 2 + 2 * 0.0529, 0.01, 'loop R');
  // 10 V step, compare with i(t) = V/R (1 - exp(-t R / L)) at t = 3 ms using 25 us steps.
  for (let k = 0; k < 120; k++) loop.step(10, 25e-6);
  near(loop.i, (10 / R) * (1 - Math.exp((-3e-3 * R) / L)), 1e-9, 'RL step response');
  assert.equal(w1.i, loop.i);
  assert.equal(w2.i, loop.i);
});

test('DC network: PSU sags through wire resistance and refuses to sink current', () => {
  const A = new Assembly();
  const psu = A.add(new PowerSupply('psu', { mount: mount(), V: 24, Rout: 0.01, Cout: 1e-3 }));
  const load = new Component('load', { mount: mount(1) });
  load.addPort('+', 'elec', 'supply'); load.addPort('-', 'elec', 'supply', { node: 'gnd' });
  A.add(load);
  const wp = A.connect(new Wire('wp', psu.port('V+'), load.port('+'), { awg: 18, length: 2, contactR: 0 }));
  const wn = A.connect(new Wire('wn', psu.port('V-'), load.port('-'), { awg: 18, length: 2, contactR: 0 }));
  wp.measure(); wn.measure();
  const dt = 25e-6, net = new DCNetwork(dt);
  const sink = { current: 5 };
  psu.dcStamp(net);
  net.addLoad(load.port('+'), load.port('-'), sink);
  net.addCapacitor(load.port('+'), load.port('-'), 100e-6);
  net.addWire(wp); net.addWire(wn);
  net.finalize();
  for (let k = 0; k < 4000; k++) net.step();
  const Rw = wp.resistance() + wn.resistance();
  near(net.across(load.port('+'), load.port('-')), 24 - 5 * (0.01 + Rw), 1e-4, 'load voltage');
  near(wp.i, 5, 1e-4, 'wire current');
  // Regenerating load pushes current back: the PSU turns its output off and the caps charge up.
  sink.current = -1;
  for (let k = 0; k < 4000; k++) net.step();
  assert.equal(psu.mode, 'off');
  assert.ok(net.across(load.port('+'), load.port('-')) > 24, 'bus rises when a load regenerates');
});

test('stepper holding torque matches datasheet (both phases at rated current)', () => {
  const spec = MOTORS['17HS4401'];
  const m = new StepperMotor('m', spec, { mount: mount() });
  // Full-step position with both coils at +1.7 A: equilibrium at theta_e = 45 deg.
  m.coilA.i = 1.7; m.coilB.i = 1.7;
  let peak = 0;
  for (let k = 0; k <= 360; k++) {
    m.theta = ((k / 360) * 2 * Math.PI) / m.Nr; // one electrical cycle
    peak = Math.max(peak, Math.abs(m.computeTorque()));
  }
  near(peak, 0.40, 0.06, 'holding torque N m'); // datasheet 40 N cm (detent adds a little)
});

test('driver + motor: current reaches target at low speed, falls at high speed (back-EMF)', () => {
  // Ramp the step rate (as firmware does) to a final speed, then measure.
  const run = (revPerS) => {
    const A = new Assembly();
    const drv = A.add(new StepperDriver('d', { mount: mount(), Irun: 1.2 }));
    const mot = A.add(new StepperMotor('m', MOTORS['17HS4401'], { mount: mount(1), extraInertia: 2e-5 }));
    for (const ph of ['A1', 'A2', 'B1', 'B2']) {
      const w = A.connect(new Wire(ph, drv.port(ph), mot.port(ph), { awg: 22, length: 0.5 }));
      w.measure();
    }
    drv.resolveLoops(traceLoop);
    drv.net = { across: (p) => (p.name === 'VIO' ? 3.3 : 24) }; // ideal 24 V bus, logic powered
    drv.enLevel = false;
    drv.digitalIn('DIR', true, -1);
    const dt = 25e-6, rate = revPerS * 3200, Tr = 0.5;
    let t = 0, emitted = 0, peakI = 0, th0 = null;
    for (let k = 0; k < 0.8 / dt; k++) {
      const tt = t + dt;
      const target = tt < Tr ? (0.5 * rate * tt * tt) / Tr : 0.5 * rate * Tr + rate * (tt - Tr);
      while (emitted < Math.floor(target)) { drv.digitalIn('STEP', true, t); drv.digitalIn('STEP', false, t + 2e-6); emitted++; }
      drv.update(dt, t);
      mot.integrate(dt);
      t = tt;
      if (t > 0.65) { peakI = Math.max(peakI, Math.abs(drv.loopA.i)); th0 ??= mot.theta; }
    }
    // Average speed over the window: at low speed the rotor rings around each microstep.
    return { peakI, revs: (mot.theta - th0) / (t - 0.65) / (2 * Math.PI) };
  };
  const slow = run(2), fast = run(25);
  near(slow.peakI, 1.2, 0.02, 'low-speed peak current');
  near(slow.revs, 2, 0.1, 'rotor follows step rate (slow)');
  near(fast.revs, 25, 0.02, 'rotor follows step rate (fast)');
  assert.ok(fast.peakI < 0.7, `back-EMF limits current at 25 rev/s: ${fast.peakI} A`);
});
