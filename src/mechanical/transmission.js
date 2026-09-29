// Mechanical connections: gearboxes and read-only sensor couplings, plus the
// rotating build plate.

import { Component, Connection } from '../core/graph.js';

// Gearbox (planetary, cycloidal or belt reduction) joining a motor shaft to a
// joint. It is a physical connection with:
//   ratio N           output turns 1/N for each input turn
//   stiffness k       torsional spring at the output [N m/rad]: teeth, shafts, belts bend
//   damping c         material damping in that spring [N m s/rad]
//   backlash          free play between teeth [rad at the output]
//   efficiency        mesh losses, modelled as friction that grows with the
//                     transmitted torque
//   coulomb, viscous  no-load friction at the output (seals, bearings)
//
// deflection d = theta_in / N - theta_out. Inside the backlash gap no torque
// passes. Once the teeth touch, torque = k (d - gap/2) + c d'.
export class Gearbox extends Connection {
  static PARAMS = ['N', 'k', 'c', 'backlash', 'eff', 'tauC', 'bv'];
  static STATE = ['offset', 'tau', 'tauF', 'deflection'];
  constructor(id, input, output, opts) {
    super(id, input, output, { ...opts, kind: 'gearbox' });
    this.N = opts.ratio;
    this.k = opts.stiffness;
    this.c = opts.damping ?? 0;
    this.backlash = opts.backlash ?? 0;
    this.eff = opts.efficiency ?? 0.85;
    this.tauC = opts.coulomb ?? 0;
    this.bv = opts.viscous ?? 0;
    this.offset = 0;
    this.tau = 0; this.deflection = 0; this.tauF = 0;
  }
  // Choose the zero so the teeth start in the middle of the gap.
  align() { this.offset = this.a.theta / this.N - this.b.theta; }
  exchange() {
    const d = this.a.theta / this.N - this.b.theta - this.offset;
    const dd = this.a.omega / this.N - this.b.omega;
    const g = this.backlash / 2;
    let ts = 0;
    if (d > g) { ts = this.k * (d - g) + this.c * dd; if (ts < 0) ts = 0; }
    else if (d < -g) { ts = this.k * (d + g) + this.c * dd; if (ts > 0) ts = 0; }
    const w = this.b.omega;
    const tf = (this.tauC + (1 - this.eff) * Math.abs(ts)) * Math.tanh(w / 0.002) + this.bv * w;
    this.tau = ts; this.tauF = tf; this.deflection = d;
    this.b.tau += ts - tf;
    this.a.tau -= ts / this.N;
  }
  inspect() {
    return {
      ratio: this.N, torque_out_Nm: this.tau, friction_Nm: this.tauF,
      windup_arcmin: (this.deflection * 180 * 60) / Math.PI, backlash_arcmin: (this.backlash * 180 * 60) / Math.PI,
    };
  }
}

// A cam, flag or magnet on a shaft that a sensor reads without loading it.
export class SensorCoupling extends Connection {
  static PARAMS = ['offset', 'scale'];
  static STATE = [];
  constructor(id, shaft, sensor, opts = {}) {
    super(id, shaft, sensor, { ...opts, kind: opts.kind ?? 'sensor-coupling' });
    this.offset = opts.offset ?? 0;
    this.scale = opts.scale ?? 1;
  }
  exchange() {
    this.b.theta = this.a.theta * this.scale + this.offset;
    this.b.omega = this.a.omega * this.scale;
  }
}

// Rotating build plate on a bearing. Its inertia includes the plate, the
// heater pad and whatever part is standing on it.
export class Turntable extends Component {
  static PARAMS = ['center', 'J0', 'tauC', 'bv'];
  static STATE = ['theta', 'omega', 'Jpart', 'extTau', 'extTauHeld'];
  constructor(id, { body, center, J, coulomb = 0.02, viscous = 0.002, ...opts }) {
    super(id, { ...opts, kind: 'turntable' });
    this.body = body; this.center = center;
    this.J0 = J; this.Jpart = 0;
    this.tauC = coulomb; this.bv = viscous;
    this.theta = 0; this.omega = 0; this.extTau = 0;
    this.axis = this.addPort('axis', 'rot', 'joint');
    this.axis.theta = 0; this.axis.omega = 0; this.axis.tau = 0;
    this.axis.worldPos = () => [this.center[0], this.center[1], this.center[2] - 0.02];
    this.updateBody();
  }
  updateBody() {
    const c = Math.cos(this.theta), s = Math.sin(this.theta);
    this.body.T = { R: [c, -s, 0, s, c, 0, 0, 0, 1], p: this.center };
  }
  integrate(dt) {
    const w = this.omega;
    const tau = this.axis.tau + this.extTau - this.bv * w - this.tauC * Math.tanh(w / 0.002);
    this.omega = w + (dt * tau) / (this.J0 + this.Jpart);
    this.theta += dt * this.omega;
    this.axis.theta = this.theta; this.axis.omega = this.omega; this.axis.tau = 0;
    this.extTau = 0;
  }
  inspect() { return { angle_deg: (this.theta * 180) / Math.PI, J_kgm2: this.J0 + this.Jpart }; }
}
