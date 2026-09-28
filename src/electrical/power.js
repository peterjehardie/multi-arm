// Power sources and passive distribution parts.

import { Component } from '../core/graph.js';

// Mains-powered switch-mode PSU (for example 24 V / 15 A, 360 W).
// Behaves as a regulated voltage behind a small output resistance. It cannot
// sink current (an output rectifier blocks it), and above its current limit
// it folds into constant-current mode.
export class PowerSupply extends Component {
  constructor(id, opts = {}) {
    super(id, { ...opts, kind: 'psu', size: [0.215, 0.115, 0.05] });
    this.Vset = opts.V ?? 24;
    this.Rout = opts.Rout ?? 0.01;
    this.Ilimit = opts.Ilimit ?? 16;
    this.Cout = opts.Cout ?? 3300e-6;
    this.on = opts.on ?? true;
    this.softStart = opts.softStart ?? 0.02;  // output ramps up over 20 ms after switch-on
    this.tOn = 0;
    this.mode = 'cv';
    this.changed = false;
    this.I = 0; this.Vout = 0;
    this.addPort('V+', 'elec', 'supply', { at: [0.1, 0.05, 0.02] });
    this.addPort('V-', 'elec', 'supply', { at: [0.1, 0.03, 0.02] });
  }
  dcStamp(net) {
    net.setReference(this.port('V-'));
    net.addCapacitor(this.port('V+'), this.port('V-'), this.Cout);
    net.addSource(this.port('V+'), this.port('V-'), this);
  }
  get Vnow() { return this.Vset * Math.min(1, this.tOn / this.softStart); }
  norton() {
    if (!this.on || this.mode === 'off') return { G: 0, I: 0 };
    if (this.mode === 'cc') return { G: 0, I: this.Ilimit };
    return { G: 1 / this.Rout, I: this.Vnow / this.Rout };
  }
  afterSolve(v, dt) {
    this.Vout = v;
    if (this.on) this.tOn += dt; else this.tOn = 0;
    let I = 0;
    if (this.on && this.mode === 'cv') I = (this.Vnow - v) / this.Rout;
    else if (this.on && this.mode === 'cc') I = this.Ilimit;
    this.I = I;
    let next = this.mode;
    if (!this.on) next = 'off';
    else if (this.mode === 'cv' && I < 0) next = 'off';
    else if (this.mode === 'cv' && I > this.Ilimit) next = 'cc';
    else if (this.mode === 'off' && v < this.Vnow) next = 'cv';
    else if (this.mode === 'cc' && v > this.Vnow - this.Ilimit * this.Rout) next = 'cv';
    if (next !== this.mode) { this.mode = next; this.changed = true; }
  }
  setOn(on) { this.on = on; this.changed = true; if (on && this.mode === 'off') this.mode = 'cv'; }
  inspect() { return { mode: this.mode, V_out: this.Vout, I_A: this.I, P_W: this.Vout * this.I }; }
}

// Buck converter module (24 V -> 5 V) feeding logic and the tool-lock servo.
// Output: a regulated source. Input: draws whatever power the output
// delivers, divided by efficiency.
export class BuckConverter extends Component {
  constructor(id, opts = {}) {
    super(id, { ...opts, kind: 'buck', size: [0.045, 0.025, 0.012] });
    this.Vset = opts.V ?? 5.0;
    this.Rout = opts.Rout ?? 0.02;
    this.eff = opts.eff ?? 0.88;
    this.uvlo = opts.uvlo ?? 6.0;
    this.enabled = false;
    this.changed = false;
    this.Iout = 0;
    this.addPort('IN+', 'elec', 'supply', { at: [-0.02, 0.008, 0.006] });
    this.addPort('IN-', 'elec', 'supply', { node: 'gnd', at: [-0.02, -0.008, 0.006] });
    this.addPort('OUT+', 'elec', 'supply', { at: [0.02, 0.008, 0.006] });
    this.addPort('OUT-', 'elec', 'supply', { node: 'gnd', at: [0.02, -0.008, 0.006] });
    this.load = { current: 0 };
    this.src = {
      changed: false,
      norton: () => (this.enabled ? { G: 1 / this.Rout, I: this.Vset / this.Rout } : { G: 0, I: 0 }),
      afterSolve: (v) => {
        this.Vout = v;
        this.Iout = this.enabled ? (this.Vset - v) / this.Rout : 0;
      },
    };
  }
  dcStamp(net) {
    this.net = net;
    net.addCapacitor(this.port('IN+'), this.port('IN-'), 100e-6);
    net.addCapacitor(this.port('OUT+'), this.port('OUT-'), 470e-6);
    net.addSource(this.port('OUT+'), this.port('OUT-'), this.src);
    net.addLoad(this.port('IN+'), this.port('IN-'), this.load);
  }
  // Called every electrical step after the network solve.
  update() {
    const vin = this.net.across(this.port('IN+'), this.port('IN-'));
    const en = vin > this.uvlo;
    if (en !== this.enabled) { this.enabled = en; this.src.changed = true; }
    const pout = Math.max(0, this.Iout * (this.Vout ?? 0));
    this.load.current = en && vin > 1 ? pout / this.eff / vin + 0.003 : 0;
  }
  inspect() { return { enabled: this.enabled, V_out: this.Vout, I_out: this.Iout, I_in: this.load.current }; }
}

// Screw-terminal distribution block: a bar of copper with several screws.
// All ports on the same bar share a node. Passive: circuit tracing may pass
// through it.
export class TerminalBlock extends Component {
  constructor(id, bars, opts = {}) {
    super(id, { ...opts, kind: 'terminal', size: [0.06, 0.03, 0.02] });
    this.passive = true;
    let x = -0.025;
    for (const [bar, n, role] of bars) {
      for (let i = 1; i <= n; i++) {
        this.addPort(`${bar}${i}`, 'elec', role ?? 'supply', { node: bar, required: false, at: [x, 0, 0.01] });
        x += 0.006;
      }
    }
  }
}
