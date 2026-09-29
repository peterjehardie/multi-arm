// Serial arm: rigid links joined by revolute joints, driven through
// gearboxes. Equations of motion M(q) q'' + h(q, q') = tau, where
//   M  is the joint-space mass matrix (how hard each joint is to accelerate,
//      including the coupling between joints),
//   h  collects gravity and velocity-dependent (Coriolis, centrifugal) torques,
//   tau are the torques arriving at the joints through the gearbox outputs
//      plus the effect of external forces at the tool (J^T F).
// Both M and h come from the recursive Newton-Euler algorithm: push
// velocities and accelerations out from the base, then push forces back in.
//
// The link inertias are not typed in as numbers. They are summed from the
// structure of each link plus every component mounted on that link's body
// (motors, gearboxes, the tool-changer plate, an attached tool).

import { Component } from '../core/graph.js';
import {
  add, sub, scale, cross, dot, m3mulv, m3mul, m3T, m3add, rotAxis, tf, tfMul, I3,
  parallelAxis, cholesky, cholSolve,
} from '../core/linalg.js';
import { G0 } from '../core/units.js';

// Inertia tensor of a solid box about its centre (link-frame axes).
export function boxInertia(m, [a, b, c]) {
  return [m * (b * b + c * c) / 12, 0, 0, 0, m * (a * a + c * c) / 12, 0, 0, 0, m * (a * a + b * b) / 12];
}

// Composite rigid body of point-sized boxes: returns { m, com, I (about com) }.
export function composite(parts) {
  let m = 0, s = [0, 0, 0];
  for (const p of parts) { m += p.m; s = add(s, scale(p.p, p.m)); }
  const com = m > 0 ? scale(s, 1 / m) : [0, 0, 0];
  let I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const p of parts) {
    const Ib = p.I ?? boxInertia(p.m, p.size ?? [0.01, 0.01, 0.01]);
    const Ir = p.R ? m3mul(m3mul(p.R, Ib), m3T(p.R)) : Ib;
    I = m3add(I, m3add(Ir, parallelAxis(p.m, sub(p.p, com))));
  }
  return { m, com, I };
}

export class ArmModel extends Component {
  static PARAMS = ['n', 'g', 'spec'];
  static STATE = ['q[]', 'qd[]', 'qdd[]'];
  // spec.joints: [{ name, offset:[x,y,z] in parent frame, axis:[..], limits:[lo,hi], structure:[parts] }]
  constructor(id, spec, { assembly, baseBody, q0, ...opts }) {
    super(id, { ...opts, kind: 'arm', mount: { body: baseBody, p: [0, 0, 0] } });
    this.assembly = assembly;
    this.spec = spec;
    this.n = spec.joints.length;
    this.baseBody = baseBody;
    this.bodies = spec.joints.map((j) => assembly.body(j.body ?? j.name));
    this.q = Float64Array.from(q0 ?? spec.joints.map(() => 0));
    this.qd = new Float64Array(this.n);
    this.qdd = new Float64Array(this.n);
    this.jointPorts = spec.joints.map((j, i) => {
      const p = this.addPort(j.name, 'rot', 'joint');
      p.theta = this.q[i]; p.omega = 0; p.tau = 0;
      // A joint port sits on the joint axis, at the origin of the link it moves.
      p.worldPos = () => (this.frames ? this.frames[i].p : null);
      return p;
    });
    this.extTau = new Float64Array(this.n);   // from forces at the tool
    this.payload = [];                        // extra bodies attached to the last link
    this.g = [0, 0, -G0];
    this.M = new Float64Array(this.n * this.n);
    this.h = new Float64Array(this.n);
    this.fk();
  }

  // Sum structure + mounted components into one inertia per link.
  rebuildInertia() {
    this.inertia = this.spec.joints.map((j, i) => {
      const body = this.bodies[i];
      const parts = [...(j.structure ?? [])];
      for (const c of this.assembly.components.values()) {
        if (c.mount?.body === body && c.mass > 0 && c !== this)
          parts.push({ m: c.mass, p: c.mount.p, size: c.size, R: c.mount.R });
      }
      if (i === this.n - 1) for (const p of this.payload) parts.push(p);
      return composite(parts);
    });
    this.dynamics();
  }

  // Forward kinematics with the current joint angles; updates link bodies.
  fk(q = this.q) {
    const frames = [];
    let T = this.baseBody.T;
    for (let i = 0; i < this.n; i++) {
      const j = this.spec.joints[i];
      T = tfMul(T, tf(I3(), j.offset));
      T = tfMul(T, tf(rotAxis(j.axis, q[i]), [0, 0, 0]));
      frames.push(T);
    }
    if (q === this.q) {
      this.frames = frames;
      for (let i = 0; i < this.n; i++) this.bodies[i].T = frames[i];
    }
    return frames;
  }

  // Recursive Newton-Euler: joint torques needed for (q, qd, qdd) in gravity g.
  rnea(qd, qdd, g, frames = this.frames) {
    const n = this.n, inert = this.inertia;
    const w = [], al = [], ac = [], p = [], z = [], c = [], Iw = [];
    let wPrev = [0, 0, 0], aPrev = [0, 0, 0], accPrev = scale(g, -1), pPrev = this.baseBody.T.p;
    for (let i = 0; i < n; i++) {
      const T = frames[i];
      p[i] = T.p;
      z[i] = m3mulv(T.R, this.spec.joints[i].axis);
      c[i] = add(T.p, m3mulv(T.R, inert[i].com));
      Iw[i] = m3mul(m3mul(T.R, inert[i].I), m3T(T.R));
      const d = sub(p[i], pPrev);
      const acc = add(accPrev, add(cross(aPrev, d), cross(wPrev, cross(wPrev, d))));
      w[i] = add(wPrev, scale(z[i], qd[i]));
      al[i] = add(add(aPrev, scale(z[i], qdd[i])), scale(cross(wPrev, z[i]), qd[i]));
      const r = sub(c[i], p[i]);
      ac[i] = add(acc, add(cross(al[i], r), cross(w[i], cross(w[i], r))));
      wPrev = w[i]; aPrev = al[i]; accPrev = acc; pPrev = p[i];
    }
    const tau = new Float64Array(n);
    let f = [0, 0, 0], nn = [0, 0, 0], pNext = null;
    for (let i = n - 1; i >= 0; i--) {
      const F = scale(ac[i], inert[i].m);
      const N = add(m3mulv(Iw[i], al[i]), cross(w[i], m3mulv(Iw[i], w[i])));
      let ni = add(N, add(nn, cross(sub(c[i], p[i]), F)));
      if (pNext) ni = add(ni, cross(sub(pNext, p[i]), f));
      f = add(F, f);
      nn = ni;
      pNext = p[i];
      tau[i] = dot(ni, z[i]);
    }
    return tau;
  }

  // Mass matrix and bias torques at the current state.
  dynamics() {
    const n = this.n, zero = new Float64Array(n);
    for (let j = 0; j < n; j++) {
      const e = new Float64Array(n); e[j] = 1;
      const col = this.rnea(zero, e, [0, 0, 0]);
      for (let i = 0; i < n; i++) this.M[i * n + j] = col[i];
    }
    // Rotor inertia of the drive, reflected through the gearbox, is NOT added
    // here: each motor rotor is its own degree of freedom behind a compliant
    // gearbox, which is what lets backlash and wind-up happen.
    for (let i = 0; i < n; i++) this.M[i * n + i] += this.spec.joints[i].armature ?? 1e-5;
    this.L = cholesky(this.M, n);
    this.h = this.rnea(this.qd, zero, this.g);
  }

  // Joint-space effect of a world force F applied at world point x: J^T F.
  addForceAt(x, F) {
    for (let i = 0; i < this.n; i++) {
      const T = this.frames[i];
      const z = m3mulv(T.R, this.spec.joints[i].axis);
      this.extTau[i] += dot(cross(z, sub(x, T.p)), F);
    }
  }

  // Linear Jacobian at world point x (3 x n, row-major).
  jacobian(x) {
    const J = new Float64Array(3 * this.n);
    for (let i = 0; i < this.n; i++) {
      const T = this.frames[i];
      const z = m3mulv(T.R, this.spec.joints[i].axis);
      const col = cross(z, sub(x, T.p));
      J[i] = col[0]; J[this.n + i] = col[1]; J[2 * this.n + i] = col[2];
    }
    return J;
  }

  integrate(dt) {
    const n = this.n, rhs = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const p = this.jointPorts[i];
      rhs[i] = p.tau + this.extTau[i] - this.h[i];
      // Hard joint stops (printed end stops): very stiff, well damped.
      const [lo, hi] = this.spec.joints[i].limits ?? [-Infinity, Infinity];
      if (this.q[i] < lo) rhs[i] += 2000 * (lo - this.q[i]) - 20 * Math.min(0, this.qd[i]);
      if (this.q[i] > hi) rhs[i] += 2000 * (hi - this.q[i]) - 20 * Math.max(0, this.qd[i]);
      p.tau = 0;
    }
    this.extTau.fill(0);
    cholSolve(this.L, n, rhs, this.qdd);
    for (let i = 0; i < n; i++) {
      this.qd[i] += dt * this.qdd[i];
      this.q[i] += dt * this.qd[i];
      this.jointPorts[i].theta = this.q[i];
      this.jointPorts[i].omega = this.qd[i];
    }
  }

  // Scheduled every kinematics stage: link poses, mass matrix, bias torques.
  updateKinematics() { this.fk(); this.dynamics(); }

  // Demo mode: no dynamics. Each joint sits where its gearbox output puts it
  // (rigid, no gravity sag, no inertia); only the link poses are updated.
  followPorts() {
    for (let i = 0; i < this.n; i++) {
      const p = this.jointPorts[i];
      this.q[i] = p.theta; this.qd[i] = p.omega; this.qdd[i] = 0;
      p.tau = 0;
    }
    this.extTau.fill(0);
  }
  updatePose() { this.fk(); }

  // Kinetic + potential energy (for verification).
  energy() {
    let KE = 0;
    const n = this.n;
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) KE += 0.5 * this.qd[i] * this.M[i * n + j] * this.qd[j];
    let PE = 0;
    for (let i = 0; i < n; i++) {
      const c = add(this.frames[i].p, m3mulv(this.frames[i].R, this.inertia[i].com));
      PE += this.inertia[i].m * G0 * c[2];
    }
    return { KE, PE, E: KE + PE };
  }

  inspect() {
    const o = {};
    for (let i = 0; i < this.n; i++) o[`q${i + 1}_deg`] = (this.q[i] * 180) / Math.PI;
    return o;
  }
}
