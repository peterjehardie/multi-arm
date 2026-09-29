// Lumped thermal network.
//
// Each piece of metal with a roughly uniform temperature is a ThermalMass
// (capacity C = mass * specific heat). Heat moves between masses only through
// a ThermalContact: a physical interface with a conductance G [W/K] (a
// cartridge pressed into a bore, a thermistor glued in a hole, a plate
// surface losing heat to room air). Heat flow q = G (Ta - Tb).
//
// Thermal ports carry: T (temperature of the owner at that face, K) and q
// (heat flowing INTO the owner this step, W, accumulated by contacts).

import { Component, Connection } from '../core/graph.js';
import { AMBIENT } from '../core/units.js';

export function thermalPort(comp, name, opts) {
  const p = comp.addPort(name, 'therm', 'face', opts);
  p.T = AMBIENT;
  p.q = 0;
  return p;
}

export class ThermalMass extends Component {
  static PARAMS = ['C'];
  static STATE = ['T', 'P'];
  constructor(id, { C, T0 = AMBIENT, faces = ['face'], ...opts }) {
    super(id, { ...opts, kind: opts.kind ?? 'thermal-mass' });
    this.C = C;
    this.T = T0;
    this.P = 0; // internal heat generation [W]
    this.faces = faces.map((f) => thermalPort(this, f, { required: false }));
    this.syncPorts();
  }
  syncPorts() { for (const f of this.faces) f.T = this.T; }
  thermalStep(dt) {
    let q = this.P;
    for (const f of this.faces) { q += f.q; f.q = 0; }
    this.T += (dt * q) / this.C;
    this.syncPorts();
  }
  inspect() { return { T_C: this.T - 273.15, C_JperK: this.C }; }
}

// Room air: an infinitely large mass at fixed temperature.
export class Ambient extends Component {
  static PARAMS = ['T'];
  static STATE = [];
  constructor(id, opts = {}) {
    super(id, { ...opts, kind: 'ambient' });
    this.T = opts.T ?? AMBIENT;
    this.faces = [];
    for (let i = 0; i < (opts.faces ?? 16); i++)
      this.faces.push(thermalPort(this, `air${i}`, { required: false }));
    for (const f of this.faces) f.T = this.T;
  }
  thermalStep() { for (const f of this.faces) { f.q = 0; f.T = this.T; } }
}

export class ThermalContact extends Connection {
  static PARAMS = ['G'];
  static STATE = ['q'];
  constructor(id, a, b, { G, ...opts }) {
    super(id, a, b, { ...opts, kind: opts.kind ?? 'thermal-contact' });
    this.G = G;
    this.q = 0;
  }
  // Conductance may depend on conditions (fan airflow); override conductance().
  conductance() { return this.G; }
  exchange() {
    const q = this.conductance() * (this.a.T - this.b.T);
    this.q = q;
    this.a.q -= q;
    this.b.q += q;
  }
  inspect() { return { G_WperK: this.conductance(), q_W: this.q }; }
}
