// Two-phase hybrid stepper motor and a TMC2209-class step/dir driver.

import { Component } from '../core/graph.js';
import { AMBIENT, COPPER } from '../core/units.js';

// ---------------------------------------------------------------------------
// Hybrid stepper motor (1.8 deg/step, 50 rotor teeth).
//
// Physics: the rotor is a toothed permanent magnet. Each phase coil's flux
// linkage varies sinusoidally with the electrical angle theta_e = Nr * theta.
//   back-EMF    e_A = -Kt w sin(theta_e),   e_B = Kt w cos(theta_e)
//   torque      T   = -Kt i_A sin(theta_e) + Kt i_B cos(theta_e) - Td sin(4 theta_e)
// The first two terms are the same physics seen from two sides (the power
// e*i equals T*w). The last is detent torque from the magnet alone.
// With phase currents I cos(phi), I sin(phi) the torque is Kt I sin(phi -
// theta_e): the rotor is pulled toward phi, which is what microstepping sets.
export class StepperMotor extends Component {
  static PARAMS = ['Nr', 'Kt', 'R20', 'L', 'J', 'Td', 'b', 'tauC', 'Cw', 'Cc', 'Gwc', 'Gca'];
  static STATE = ['theta', 'omega', 'Tw', 'Tc', 'tauEM'];
  constructor(id, spec, opts = {}) {
    super(id, { ...opts, kind: 'stepper', size: spec.size ?? [0.042, 0.042, 0.048] });
    this.spec = spec;
    this.Nr = spec.rotorTeeth ?? 50;
    this.Kt = spec.Kt;
    this.R20 = spec.R;
    this.L = spec.L;
    this.J = spec.J + (opts.extraInertia ?? 0);
    this.Td = spec.detent ?? 0;
    this.b = spec.viscous ?? 2e-5;        // bearing + iron-loss drag [N m s/rad]
    this.tauC = spec.coulomb ?? 0.002;    // bearing friction [N m]
    // Thermal: winding and case lumps.
    this.Tw = AMBIENT; this.Tc = AMBIENT;
    this.Cw = spec.Cw ?? 25; this.Cc = spec.Cc ?? 140;
    this.Gwc = spec.Gwc ?? 1.2; this.Gca = spec.Gca ?? 0.16;
    // Rotor state.
    this.theta = opts.theta0 ?? 0;
    this.omega = 0;
    this.tauEM = 0;
    const self = this;
    const coil = (phase) => ({
      label: `${id} coil ${phase}`, i: 0,
      resistance: () => self.R20 * (1 + COPPER.alpha * (self.Tw - 293.15)),
      inductance: () => self.L,
      emf: () => { self.trig(); return phase === 'A' ? -self.Kt * self.omega * self.sinE : self.Kt * self.omega * self.cosE; },
      record(i) { this.i = i; },
    });
    this.coilA = coil('A');
    this.coilB = coil('B');
    this.addPort('A1', 'elec', 'coil', { at: [0, -0.02, -0.02] });
    this.addPort('A2', 'elec', 'coil', { at: [0, -0.021, -0.02] });
    this.addPort('B1', 'elec', 'coil', { at: [0, -0.022, -0.02] });
    this.addPort('B2', 'elec', 'coil', { at: [0, -0.023, -0.02] });
    this.addThrough('A1', 'A2', this.coilA);
    this.addThrough('B1', 'B2', this.coilB);
    this.shaft = this.addPort('shaft', 'rot', 'shaft', { at: [0, 0, 0.03] });
    this.shaft.theta = this.theta; this.shaft.omega = 0; this.shaft.tau = 0;
    this.trigAt = NaN;
    this.trig();
  }
  // sin and cos of the electrical angle, recomputed only when the rotor moved.
  trig() {
    if (this.trigAt === this.theta) return;
    this.trigAt = this.theta;
    this.sinE = Math.sin(this.Nr * this.theta); this.cosE = Math.cos(this.Nr * this.theta);
  }

  // Electromagnetic torque from the currents now flowing in the coils.
  computeTorque() {
    this.trig();
    const s = this.sinE, c = this.cosE;
    const s2 = 2 * s * c, c2 = c * c - s * s; // sin(4x) = 2 sin(2x) cos(2x)
    this.tauEM = this.Kt * (-this.coilA.i * s + this.coilB.i * c) - this.Td * 2 * s2 * c2;
    return this.tauEM;
  }

  // Semi-implicit Euler on the rotor. External torque arrives through the
  // shaft port (from whatever is coupled to it).
  integrate(dt) {
    const w = this.omega;
    let tau = this.computeTorque() + this.shaft.tau - this.b * w;
    // Regularised Coulomb friction (smooth sign, 0.01 rad/s scale).
    tau -= this.tauC * Math.tanh(w / 0.01);
    this.omega = w + (dt * tau) / this.J;
    this.theta += dt * this.omega;
    this.trig();
    this.shaft.theta = this.theta;
    this.shaft.omega = this.omega;
    this.shaft.tau = 0;
  }

  // Demo mode (1 ms steps): the rotor sits exactly where the coil currents
  // point it (zero load angle). Stall, resonance and step loss are gone, but
  // the rotor still only moves because current flows in its coils.
  followCurrent(dt) {
    const iA = this.coilA.i, iB = this.coilB.i;
    if (iA * iA + iB * iB > 1e-6) {
      let d = Math.atan2(iB, iA) - this.Nr * this.theta;
      d -= 2 * Math.PI * Math.round(d / (2 * Math.PI));
      this.omega = d / this.Nr / dt;
      this.theta += d / this.Nr;
    } else this.omega = 0;
    this.tauEM = 0;
    this.shaft.theta = this.theta; this.shaft.omega = this.omega; this.shaft.tau = 0;
  }

  thermal(dt) {
    const P = this.coilA.i ** 2 * this.coilA.resistance() + this.coilB.i ** 2 * this.coilB.resistance();
    const qwc = this.Gwc * (this.Tw - this.Tc);
    this.Tw += (dt * (P - qwc)) / this.Cw;
    this.Tc += (dt * (qwc - this.Gca * (this.Tc - AMBIENT))) / this.Cc;
  }

  inspect() {
    return {
      angle_deg: (this.theta * 180) / Math.PI, speed_rpm: (this.omega * 60) / (2 * Math.PI),
      iA_A: this.coilA.i, iB_A: this.coilB.i, torque_Nm: this.tauEM,
      winding_C: this.Tw - 273.15, case_C: this.Tc - 273.15,
    };
  }
}

// ---------------------------------------------------------------------------
// TMC2209-class driver module on a carrier board.
//
// Logic side: STEP / DIR / EN inputs are Schmitt triggers referenced to VIO.
// Each rising STEP edge advances the internal microstep counter MSCNT
// (1024 counts per electrical cycle = 4 full steps). The counter indexes a
// sine table that gives the two coil current targets.
//
// Power side: two H-bridges. A chopper regulates each coil current by
// switching the bridge every ~25 us. This model uses the chopper's average
// effect: each period the bridge applies the voltage that would bring the
// current to its target by the end of the period, limited to what the supply
// can give (+-VM). At speed the motor's back-EMF eats that headroom, current
// can no longer reach target, and torque falls: the real torque-speed curve.
export class StepperDriver extends Component {
  static PARAMS = ['microsteps', 'Irun', 'holdFrac', 'tPowerDown', 'Rds', 'Iq', 'Cbulk', 'Cth', 'Gth'];
  static STATE = ['mscnt', 'dirLevel', 'enLevel', 'stepLevel', 'tDirChange', 'tStepRise', 'tLastStep', 'steps', 'Tj', 'overTemp', 'uA', 'uB', 'logicOn', 'load.current', 'loopA.i', 'loopB.i'];
  constructor(id, opts = {}) {
    super(id, { ...opts, kind: 'driver', size: [0.02, 0.015, 0.012] });
    this.microsteps = opts.microsteps ?? 16;
    this.Irun = opts.Irun ?? 1.2;                  // peak coil current [A]
    this.holdFrac = opts.holdFrac ?? 1.0;          // IHOLD / IRUN
    this.tPowerDown = opts.tPowerDown ?? 0.44;     // standstill to hold current [s]
    this.Rds = opts.Rds ?? 0.17;                   // per MOSFET; a bridge path has two
    this.Iq = opts.Iq ?? 0.006;                    // quiescent supply current [A]
    this.Cbulk = opts.Cbulk ?? 100e-6;
    this.mscnt = opts.mscnt0 ?? 0;
    this.dirLevel = false; this.enLevel = true;    // EN is active-low; pulled up = disabled
    this.stepLevel = false;
    this.tDirChange = -1; this.tStepRise = -1;
    this.tLastStep = -Infinity;
    this.steps = 0; this.violations = { setup: 0, pulse: 0 };
    this.Tj = AMBIENT; this.Cth = 2.5; this.Gth = 0.045; this.overTemp = false;
    this.uA = 0; this.uB = 0;
    this.logicOn = false;
    this.load = { current: 0 };
    this.addPort('VM', 'elec', 'supply', { at: [-0.012, 0.006, 0.004] });
    this.addPort('GND', 'elec', 'supply', { node: 'gnd', at: [-0.012, 0.002, 0.004] });
    this.addPort('GNDL', 'elec', 'supply', { node: 'gnd', at: [0.012, 0.002, 0.004] });
    this.addPort('VIO', 'elec', 'supply', { at: [0.012, 0.006, 0.004] });
    this.addPort('STEP', 'elec', 'din', { at: [0.012, -0.002, 0.004] });
    this.addPort('DIR', 'elec', 'din', { at: [0.012, -0.004, 0.004] });
    this.addPort('EN', 'elec', 'din', { at: [0.012, -0.006, 0.004] });
    for (const [n, y] of [['A1', 0.0], ['A2', -0.002], ['B1', -0.004], ['B2', -0.006]])
      this.addPort(n, 'elec', 'phase', { at: [-0.012, y, 0.004] });
  }

  dcStamp(net) {
    this.net = net;
    net.addCapacitor(this.port('VM'), this.port('GND'), this.Cbulk);
    net.addLoad(this.port('VM'), this.port('GND'), this.load);
    net.addLoad(this.port('VIO'), this.port('GND'), { current: 0.0005 });
  }
  resolveLoops(traceLoop) {
    this.loopA = traceLoop(this.port('A1'), this.port('A2'));
    this.loopB = traceLoop(this.port('B1'), this.port('B2'));
  }

  // --- logic interface -----------------------------------------------------
  logicGround() { return this.port('GNDL'); }
  inputCapacitance() { return 5e-12; }
  // Carrier board: EN pulled up to VIO (10 k), STEP / DIR pulled down inside the chip.
  idleLevel(pin) { return pin === 'EN'; }
  attachInputNet(pin, net, entry) { (this.inputNets ??= []).push([net, entry]); }
  pullResistance(pin) { return pin === 'EN' ? 10e3 : 100e3; }
  idleHigh() { return this.net.across(this.port('VIO'), this.port('GND')); }
  inputThresholds() {
    const vio = this.net.across(this.port('VIO'), this.port('GND'));
    if (!(vio > 1.5)) return { rise: Infinity, fall: -Infinity }; // logic unpowered
    return { rise: 0.55 * vio, fall: 0.45 * vio };
  }
  digitalIn(pin, level, t) {
    if (pin === 'DIR') { this.dirLevel = level; this.tDirChange = t; return; }
    if (pin === 'EN') { this.enLevel = level; return; }
    if (pin === 'STEP') {
      if (level) {
        this.tStepRise = t;
        // DIR must be stable 20 ns before the STEP edge.
        let dir = this.dirLevel;
        if (t - this.tDirChange < 20e-9) { this.violations.setup++; dir = !dir; }
        if (!this.enLevel) {
          this.mscnt = (this.mscnt + (dir ? 1 : -1) * (256 / this.microsteps) + 1024) & 1023;
          this.steps += dir ? 1 : -1;
          this.tLastStep = t;
        }
      } else if (t - this.tStepRise < 100e-9) this.violations.pulse++;
      this.stepLevel = level;
    }
  }

  // --- power interface: one chopper period ----------------------------------
  update(dt, t) {
    const P = (this.P ??= { VIO: this.port('VIO'), GND: this.port('GND'), VM: this.port('VM') });
    const vio = this.net.across(P.VIO, P.GND);
    const logicOn = vio > 1.5;
    if (logicOn && !this.logicOn) for (const [net, r] of this.inputNets ?? []) net.resync(r, t);
    this.logicOn = logicOn;
    const vbus = this.net.across(P.VM, P.GND);
    const enabled = !this.enLevel && vbus > 4.5 && !this.overTemp;
    let I = this.Irun;
    if (t - this.tLastStep > this.tPowerDown) I *= this.holdFrac;
    const ph = (2 * Math.PI * this.mscnt) / 1024;
    const iRefA = I * Math.cos(ph), iRefB = I * Math.sin(ph);
    this.uA = this.drive(this.loopA, iRefA, vbus, enabled, dt);
    this.uB = this.drive(this.loopB, iRefB, vbus, enabled, dt);
    const iA = this.loopA ? this.loopA.i : 0, iB = this.loopB ? this.loopB.i : 0;
    this.load.current = vbus > 1 ? (this.uA * iA + this.uB * iB) / vbus + this.Iq : 0;
  }

  // Demo mode: an ideal chopper. Each coil carries exactly its sine-table
  // current (no rise time, no back-EMF limit); the bus supplies the copper
  // losses of the coils and wires.
  updateIdeal(dt, t) {
    const P = (this.P ??= { VIO: this.port('VIO'), GND: this.port('GND'), VM: this.port('VM') });
    const logicOn = this.net.across(P.VIO, P.GND) > 1.5;
    if (logicOn && !this.logicOn) for (const [net, r] of this.inputNets ?? []) net.resync(r, t);
    this.logicOn = logicOn;
    const vbus = this.net.across(P.VM, P.GND);
    const enabled = !this.enLevel && vbus > 4.5 && !this.overTemp;
    let I = enabled ? this.Irun : 0;
    if (t - this.tLastStep > this.tPowerDown) I *= this.holdFrac;
    const ph = (2 * Math.PI * this.mscnt) / 1024;
    let Ploss = 0;
    for (const [loop, i] of [[this.loopA, I * Math.cos(ph)], [this.loopB, I * Math.sin(ph)]]) {
      if (!loop) continue;
      const ok = Number.isFinite(loop.Rc);
      loop.setCurrent(ok ? i : 0);
      if (ok) Ploss += i * i * (loop.Rc + 2 * this.Rds);
    }
    this.uA = 0; this.uB = 0;
    this.load.current = vbus > 1 ? Ploss / vbus + this.Iq : 0;
  }

  drive(loop, iRef, vbus, enabled, dt) {
    if (!loop) return 0;
    const Rb = 2 * this.Rds;
    const R = loop.Rc + Rb, L = loop.Lc;
    if (!Number.isFinite(R)) { if (loop.i !== 0) loop.setCurrent(0); return 0; }
    const e = loop.emf();
    let u;
    if (loop.aKey !== dt) { loop.aKey = dt; loop.aDecay = Math.exp((-dt * R) / L); }
    const a = loop.aDecay;
    if (enabled) {
      const iss = (iRef - loop.i * a) / (1 - a);
      u = Math.max(-vbus, Math.min(vbus, e + R * iss));
      // Exact step with the voltage held for the chopper period.
      const iss2 = (u - e) / R;
      loop.setCurrent(iss2 + (loop.i - iss2) * a);
      return u;
    } else {
      // Bridge off: stored coil energy returns through the body diodes into
      // the supply until the current reaches zero.
      if (Math.abs(loop.i) < 1e-4) { loop.setCurrent(0); return 0; }
      u = -Math.sign(loop.i) * (vbus + 1.4);
      const i0 = loop.i;
      loop.step(u, dt, Rb, R, L, e);
      if (Math.sign(loop.i) !== Math.sign(i0)) loop.setCurrent(0);
      return u;
    }
    loop.step(u, dt, Rb, R, L, e);
    return u;
  }

  thermal(dt) {
    const iA = this.loopA ? this.loopA.i : 0, iB = this.loopB ? this.loopB.i : 0;
    const P = (iA * iA + iB * iB) * 2 * this.Rds + 0.05;
    this.Tj += (dt * (P - this.Gth * (this.Tj - AMBIENT))) / this.Cth;
    if (this.Tj > 273.15 + 150) this.overTemp = true;
    if (this.Tj < 273.15 + 120) this.overTemp = false;
  }

  inspect() {
    return {
      enabled: !this.enLevel, MSCNT: this.mscnt, steps: this.steps,
      iA_A: this.loopA?.i ?? 0, iB_A: this.loopB?.i ?? 0, uA_V: this.uA, uB_V: this.uB,
      I_bus_A: this.load.current, chip_C: this.Tj - 273.15,
      setupViolations: this.violations.setup, overTemp: this.overTemp,
    };
  }
}
