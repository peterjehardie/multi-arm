// Electrical devices: heaters, thermistors, MOSFET switch modules, a brushed
// DC spindle motor, a hobby servo and mechanical limit switches.

import { Component } from '../core/graph.js';
import { AMBIENT } from '../core/units.js';
import { thermalPort } from '../thermal/thermal.js';
import { Contact } from './circuit.js';

// ---------------------------------------------------------------------------
// Resistive heater (cartridge or silicone pad). Electrical element between
// terminals a and b; the power it dissipates heats its own core mass, which
// touches the part it heats through the 'surface' thermal port.
export class Heater extends Component {
  static PARAMS = ['R0', 'alpha', 'C'];
  static STATE = ['T', 'Eacc', 'Pavg'];
  constructor(id, { R0, alpha = 0.0002, C = 0.6, ...opts }) {
    super(id, { ...opts, kind: 'heater' });
    this.R0 = R0; this.alpha = alpha; this.C = C;
    this.T = AMBIENT;
    const self = this;
    this.element = {
      label: `${id} element`, i: 0, Pacc: 0,
      resistance: () => self.R0 * (1 + self.alpha * (self.T - 293.15)),
      record(i) { this.i = i; },
      accumulate: (dt, iSq) => self.accumulate(dt, iSq),
    };
    this.addPort('a', 'elec', 'load', { at: [0, 0.003, 0] });
    this.addPort('b', 'elec', 'load', { at: [0, -0.003, 0] });
    this.addThrough('a', 'b', this.element);
    this.surface = thermalPort(this, 'surface');
    this.Pavg = 0; this.Eacc = 0;
  }
  // Called every electrical step so short PWM pulses are counted exactly.
  accumulate(dt, iAvgSq) { this.Eacc += iAvgSq * this.element.resistance() * dt; }
  thermalStep(dt) {
    const P = this.Eacc / dt;
    this.Pavg = P; this.Eacc = 0;
    this.T += (dt * (P + this.surface.q)) / this.C;
    this.surface.q = 0;
    this.surface.T = this.T;
  }
  inspect() { return { T_C: this.T - 273.15, R_ohm: this.element.resistance(), P_W: this.Pavg }; }
}

// ---------------------------------------------------------------------------
// NTC thermistor (100k, beta 3950). A glass bead with a small thermal mass,
// glued into the part it measures. R = R25 exp(B (1/T - 1/298.15)).
export class Thermistor extends Component {
  static PARAMS = ['R25', 'B', 'C'];
  static STATE = ['T'];
  constructor(id, { R25 = 100e3, B = 3950, C = 0.01, ...opts } = {}) {
    super(id, { ...opts, kind: 'thermistor', size: [0.004, 0.004, 0.004] });
    this.R25 = R25; this.B = B; this.C = C;
    this.T = AMBIENT;
    const self = this;
    this.element = {
      label: `${id} NTC`, i: 0,
      resistance: () => self.R25 * Math.exp(self.B * (1 / self.T - 1 / 298.15)),
      record(i) { this.i = i; },
    };
    this.addPort('a', 'elec', 'sensor');
    this.addPort('b', 'elec', 'sensor');
    this.addThrough('a', 'b', this.element);
    this.bead = thermalPort(this, 'bead');
  }
  thermalStep(dt) {
    const Pself = this.element.i ** 2 * this.element.resistance();
    this.T += (dt * (this.bead.q + Pself)) / this.C;
    this.bead.q = 0;
    this.bead.T = this.T;
  }
  inspect() { return { T_C: this.T - 273.15, R_ohm: this.element.resistance() }; }
}

// ---------------------------------------------------------------------------
// Low-side MOSFET switch module (logic-level N-channel, e.g. AOD4184).
// Load connects between OUT+ (tied to VIN+) and OUT- (the drain). The gate
// is driven through the SIG pin by a logic signal. An optional flyback diode
// across the load carries inductive current when the switch opens.
//
// The gate can switch several times inside one physics step (PWM), so the
// module records each transition time and integrates the load loop piecewise
// between them: the average current is exact, not rounded to the step grid.
export class MosfetModule extends Component {
  static PARAMS = ['Rds', 'flyback', 'Vth'];
  static STATE = ['gate', 'iAvg', 'iAvgSq', 'load.current', 'loop.i'];
  constructor(id, opts = {}) {
    super(id, { ...opts, kind: 'mosfet', size: [0.034, 0.018, 0.012] });
    this.Rds = opts.Rds ?? 0.012;
    this.flyback = opts.flyback ?? false;
    this.Vth = opts.Vth ?? 1.9;          // gate threshold [V]
    this.gate = false;
    this.transitions = [];               // [t, level] inside current step
    this.load = { current: 0 };
    this.iAvg = 0; this.iAvgSq = 0;
    this.addPort('VIN+', 'elec', 'supply', { node: 'vin', at: [-0.015, 0.006, 0.006] });
    this.addPort('VIN-', 'elec', 'supply', { node: 'gnd', at: [-0.015, -0.006, 0.006] });
    this.addPort('OUT+', 'elec', 'load', { node: 'vin', at: [0.015, 0.006, 0.006] });
    this.addPort('OUT-', 'elec', 'load', { node: 'drain', at: [0.015, -0.006, 0.006] });
    this.addPort('SIG', 'elec', 'din', { at: [0, -0.009, 0.006] });
    this.addPort('SGND', 'elec', 'supply', { node: 'gnd', at: [0.004, -0.009, 0.006] });
  }
  dcStamp(net) {
    this.net = net;
    net.addLoad(this.port('VIN+'), this.port('VIN-'), this.load);
  }
  resolveLoops(traceLoop) { this.loop = traceLoop(this.port('OUT+'), this.port('OUT-')); }
  logicGround() { return this.port('SGND'); }
  inputCapacitance() { return 2.5e-9; } // gate charge seen as a capacitance
  inputThresholds() { return { rise: this.Vth, fall: this.Vth - 0.1 }; }
  digitalIn(pin, level, t) { this.transitions.push(t, level); }

  update(dt, t0) {
    const loop = this.loop;
    // Nothing switching, nothing flowing: skip the work.
    if (!this.gate && !this.transitions.length && (!loop || loop.i === 0)) {
      this.iAvg = 0; this.iAvgSq = 0; this.load.current = 0;
      return;
    }
    const vbus = this.net.across(this.port('VIN+'), this.port('VIN-'));
    let tPrev = t0, on = this.gate, q = 0, q2 = 0, qSupply = 0;
    const tr = this.transitions;
    const seg = (tEnd) => {
      const h = tEnd - tPrev;
      if (h <= 0 || !loop) { tPrev = tEnd; return; }
      const R = loop.Rc + (on ? this.Rds : 0), L = loop.Lc, e = loop.emf();
      const i0 = loop.i;
      if (on) loop.step(vbus, h, 0, R, L, e);
      else if (this.flyback && i0 > 1e-6) {
        loop.step(-0.7, h, 0, loop.Rc, L, e);
        if (loop.i < 0) loop.setCurrent(0);
      } else loop.setCurrent(0);
      const im = 0.5 * (i0 + loop.i);
      q += im * h; q2 += im * im * h;
      if (on) qSupply += im * h;
      tPrev = tEnd;
    };
    for (let k = 0; k < tr.length; k += 2) {
      seg(Math.min(t0 + dt, Math.max(t0, tr[k])));
      on = tr[k + 1];
    }
    seg(t0 + dt);
    tr.length = 0;
    this.gate = on;
    this.iAvg = q / dt;
    this.iAvgSq = q2 / dt;
    this.load.current = qSupply / dt;
    if (loop) {
      // Loop elements carry the average current for heating / display.
      loop.setCurrent(loop.i);
      for (const { el } of loop.items) el.accumulate?.(dt, this.iAvgSq);
    }
  }
  inspect() { return { gate: this.gate, I_load_A: this.iAvg, V_bus: this.net?.across(this.port('VIN+'), this.port('VIN-')) }; }
}

// ---------------------------------------------------------------------------
// Brushed DC motor (775 class) used as the milling spindle, with the collet
// and cutter on its shaft. Electrical: armature R, L and back-EMF Ke w.
// Mechanical: rotor + collet + cutter inertia, bearing and brush friction,
// and the cutting load that arrives through the 'bit' material port.
export class BrushedDCMotor extends Component {
  static PARAMS = ['R', 'L', 'Ke', 'J', 'tauF', 'b'];
  static STATE = ['omega', 'theta', 'tauEM'];
  constructor(id, spec, opts = {}) {
    super(id, { ...opts, kind: 'dc-motor', size: [0.045, 0.045, 0.1] });
    this.R = spec.R; this.L = spec.L; this.Ke = spec.Ke;
    this.J = spec.J; this.tauF = spec.tauF; this.b = spec.b;
    this.omega = 0; this.theta = 0; this.tauLoad = 0; this.tauEM = 0;
    const self = this;
    this.armature = {
      label: `${id} armature`, i: 0,
      resistance: () => self.R, inductance: () => self.L,
      emf: () => self.Ke * self.omega,
      record(i) { this.i = i; },
    };
    this.addPort('M+', 'elec', 'load');
    this.addPort('M-', 'elec', 'load');
    this.addThrough('M+', 'M-', this.armature);
    this.bit = this.addPort('bit', 'mat', 'cutter', { required: false, at: opts.bitAt ?? [0, 0, -0.09] });
    this.bit.omega = 0; this.bit.tauLoad = 0;
  }
  integrate(dt) {
    const w = this.omega;
    this.tauEM = this.Ke * this.armature.i;
    let tau = this.tauEM - this.b * w - this.tauF * Math.tanh(w / 1) - this.bit.tauLoad;
    this.omega = Math.max(-5, w + (dt * tau) / this.J);
    this.theta += dt * this.omega;
    this.bit.omega = this.omega;
    this.bit.tauLoad = 0;
  }
  inspect() {
    return { speed_rpm: (this.omega * 60) / (2 * Math.PI), I_A: this.armature.i, torque_Nm: this.tauEM };
  }
}

// ---------------------------------------------------------------------------
// Hobby servo (MG90S class). Reads the width of 50 Hz control pulses on SIG
// (1.0 ms -> 0 deg, 2.0 ms -> 180 deg), and its internal controller turns the
// output horn toward that angle at a limited speed. The horn is a rotational
// port, so whatever it drives (the tool-changer latch) sees the real angle.
export class Servo extends Component {
  static PARAMS = ['speed'];
  static STATE = ['angle', 'target', 'tRise', 'load.current'];
  constructor(id, opts = {}) {
    super(id, { ...opts, kind: 'servo', size: [0.023, 0.012, 0.029] });
    this.angle = opts.angle0 ?? 0;          // rad
    this.target = this.angle;
    this.speed = opts.speed ?? (Math.PI / 3) / 0.1; // 60 deg per 0.1 s
    this.tRise = -1; // time of the last rising edge on SIG (-1: none yet)
    this.load = { current: 0.008 };
    this.addPort('V+', 'elec', 'supply');
    this.addPort('GND', 'elec', 'supply', { node: 'gnd' });
    this.addPort('SIG', 'elec', 'din');
    this.horn = this.addPort('horn', 'rot', 'shaft', { required: false });
    this.horn.theta = this.angle; this.horn.omega = 0; this.horn.tau = 0;
  }
  dcStamp(net) { this.net = net; net.addLoad(this.port('V+'), this.port('GND'), this.load); }
  logicGround() { return this.port('GND'); }
  inputCapacitance() { return 20e-12; }
  inputThresholds() { return { rise: 1.6, fall: 1.2 }; }
  digitalIn(pin, level, t) {
    if (level) this.tRise = t;
    else if (this.tRise >= 0) {
      const w = t - this.tRise;
      if (w > 0.5e-3 && w < 2.6e-3) this.target = ((Math.min(2e-3, Math.max(1e-3, w)) - 1e-3) / 1e-3) * Math.PI;
    }
  }
  update(dt) {
    const vs = this.net.across(this.port('V+'), this.port('GND'));
    const powered = vs > 4.0;
    const err = this.target - this.angle;
    const moving = powered && Math.abs(err) > 0.005;
    const step = Math.sign(err) * Math.min(Math.abs(err), this.speed * (vs / 5) * dt);
    if (powered) this.angle += step;
    this.horn.theta = this.angle; this.horn.omega = moving ? step / dt : 0;
    this.load.current = powered ? (moving ? 0.35 : 0.008) : 0;
  }
  inspect() { return { angle_deg: (this.angle * 180) / Math.PI, target_deg: (this.target * 180) / Math.PI, I_A: this.load.current }; }
}

// ---------------------------------------------------------------------------
// Mechanical microswitch pressed by a cam on a joint. The cam port reads the
// joint angle through a SensorCoupling. When the angle passes the trip point
// the contact closes, bouncing a few times over about a millisecond as real
// contacts do. Differential travel gives hysteresis.
export class LimitSwitch extends Component {
  static PARAMS = ['trip', 'dirn', 'hyst'];
  static STATE = ['pressed', 'contact.closed'];
  constructor(id, { tripAngle, direction = +1, hysteresis = 0.004, rng, sim, ...opts }) {
    super(id, { ...opts, kind: 'limit-switch', size: [0.02, 0.01, 0.006] });
    this.trip = tripAngle; this.dirn = direction; this.hyst = hysteresis;
    this.rng = rng; this.sim = sim;
    this.contact = new Contact(0.03, `${id} contact`);
    this.pressed = false;
    this.addPort('COM', 'elec', 'sensor');
    this.addPort('NO', 'elec', 'sensor');
    this.addThrough('COM', 'NO', this.contact);
    this.cam = this.addPort('cam', 'rot', 'sensor');
    sim.handlers.contact ??= (sw, closed) => { if (sw.pressed) sw.contact.closed = closed; };
  }
  update() {
    const a = this.cam.theta;
    if (a === undefined) return;
    const x = this.dirn * (a - this.trip);
    const pressed = this.pressed ? x > -this.hyst : x > 0;
    if (pressed === this.pressed) return;
    this.pressed = pressed;
    if (!pressed) { this.contact.closed = false; return; }
    // Contact bounce on make: 2-5 short opens over ~1 ms.
    const n = 2 + Math.floor(this.rng.next() * 4);
    let t = this.sim.t;
    this.contact.closed = true;
    for (let k = 0; k < n; k++) {
      t += this.rng.uniform(40e-6, 250e-6);
      this.sim.post(t, 'contact', this, false);
      t += this.rng.uniform(40e-6, 250e-6);
      this.sim.post(t, 'contact', this, true);
    }
  }
  inspect() { return { pressed: this.pressed, closed: this.contact.closed, trip_deg: (this.trip * 180) / Math.PI }; }
}
