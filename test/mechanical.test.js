// Mechanics checks: Newton-Euler arm dynamics, gearbox, kinematics.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Assembly, Component } from '../src/core/graph.js';
import { ArmModel } from '../src/mechanical/arm.js';
import { Gearbox } from '../src/mechanical/transmission.js';
import { ArmKinematics } from '../src/firmware/kinematics.js';
import { makeRng } from '../src/core/rng.js';
import { G0 } from '../src/core/units.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
const Y = [0, -1, 0];

function testArm(q0 = [0, 0, 0, 0]) {
  const A = new Assembly();
  const base = A.body('base');
  const spec = {
    joints: [
      { name: 'j1', body: 'l1', offset: [0, 0, 0.1], axis: [0, 0, 1], structure: [{ m: 1, p: [0, 0, 0.03], size: [0.08, 0.08, 0.06] }] },
      { name: 'j2', body: 'l2', offset: [0, 0, 0.08], axis: Y, structure: [{ m: 0.6, p: [0.1, 0, 0], size: [0.2, 0.04, 0.03] }] },
      { name: 'j3', body: 'l3', offset: [0.2, 0, 0], axis: Y, structure: [{ m: 0.4, p: [0.09, 0, 0], size: [0.18, 0.04, 0.03] }] },
      { name: 'j4', body: 'l4', offset: [0.18, 0, 0], axis: Y, structure: [{ m: 0.3, p: [0.04, 0.01, 0], size: [0.06, 0.04, 0.04] }] },
    ],
  };
  const arm = A.add(new ArmModel('arm', spec, { assembly: A, baseBody: base, q0 }));
  arm.rebuildInertia();
  return arm;
}

test('gravity torque at the shoulder matches a hand calculation (arm horizontal)', () => {
  const arm = testArm([0, 0, 0, 0]);
  arm.fk(); arm.dynamics();
  // Moments of link masses about the shoulder axis, arm straight out along +x.
  const expected = G0 * (0.6 * 0.1 + 0.4 * (0.2 + 0.09) + 0.3 * (0.38 + 0.04));
  near(arm.h[1], expected, 1e-9, 'shoulder gravity torque');
  near(arm.h[0], 0, 1e-12, 'no gravity torque about the vertical axis');
});

test('mass matrix is symmetric and positive definite', () => {
  const arm = testArm([0.3, 0.7, -1.1, 0.4]);
  arm.fk(); arm.dynamics();
  const n = arm.n;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) near(arm.M[i * n + j], arm.M[j * n + i], 1e-12, `M[${i}${j}]`);
  // Cholesky succeeded inside dynamics(); a vector test for good measure.
  let xMx = 0;
  const x = [1, -2, 0.5, 3];
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) xMx += x[i] * arm.M[i * n + j] * x[j];
  assert.ok(xMx > 0);
});

test('a free-swinging arm conserves energy (no friction, no drive)', () => {
  const arm = testArm([0.2, 0.9, -0.4, 0.3]);
  arm.qd.set([0.8, 0, 0.5, -1]);
  arm.fk(); arm.dynamics();
  const E0 = arm.energy().E;
  const dt = 2e-5;
  for (let k = 0; k < 50000; k++) { arm.integrate(dt); arm.fk(); arm.dynamics(); }
  const E1 = arm.energy().E;
  near(E1, E0, 2e-3 * Math.abs(E0) + 1e-4, 'total energy after 1 s');
});

test('gearbox: no torque inside the backlash gap, spring torque beyond it', () => {
  const mk = () => { const c = new Component('x'); return c; };
  const m = mk(), l = mk();
  const pin = m.addPort('s', 'rot', 'shaft'), pout = l.addPort('j', 'rot', 'joint');
  Object.assign(pin, { theta: 0, omega: 0, tau: 0 });
  Object.assign(pout, { theta: 0, omega: 0, tau: 0 });
  const gb = new Gearbox('gb', pin, pout, { ratio: 50, stiffness: 3000, backlash: 0.002, efficiency: 1, coulomb: 0 });
  pin.theta = 50 * 0.0009; gb.exchange();
  assert.equal(pout.tau, 0, 'inside gap');
  pout.tau = 0; pin.tau = 0;
  pin.theta = 50 * 0.003; gb.exchange();
  near(pout.tau, 3000 * (0.003 - 0.001), 1e-9, 'output torque');
  near(pin.tau, -pout.tau / 50, 1e-12, 'reaction on motor = output / ratio');
});

test('firmware inverse kinematics inverts forward kinematics', () => {
  const kin = new ArmKinematics({ baseHeight: 0.075, shoulderHeight: 0.085, L2: 0.22, L3: 0.2, L4: 0.045 });
  kin.tip = 0.058;
  const rng = makeRng(3);
  for (let k = 0; k < 200; k++) {
    const q = [rng.uniform(-2, 2), rng.uniform(0.2, 1.4), rng.uniform(-2.4, -0.3), rng.uniform(-1.5, 0.5)];
    const { p, phi } = kin.forward(q);
    const q2 = kin.inverse(p, phi);
    assert.ok(q2, 'reachable');
    const p2 = kin.forward(q2).p;
    for (let i = 0; i < 3; i++) near(p2[i], p[i], 1e-9, 'tip position');
  }
});
