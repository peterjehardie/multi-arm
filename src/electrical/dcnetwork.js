// DC power distribution network, solved by nodal analysis.
//
// Every supply terminal (a PSU output, a driver board's VM and GND pins, a
// buck converter's input) is a node. Every wire between supply terminals is a
// branch with the wire's actual resistance. Capacitors on the boards store
// charge. Loads draw the current their own physics demands. Each step the
// solver finds all node voltages, so a sagging supply, a warm ground wire or
// motor current returning through a logic ground wire all appear by
// themselves.
//
// Integration: backward Euler, (G + C/dt) v[n+1] = C/dt v[n] + I_sources - I_loads.
// The matrix only changes when a source changes mode or a wire warms up
// noticeably, so its factorisation is cached.

import { luFactor } from '../core/linalg.js';

export class DCNetwork {
  constructor(dt) {
    this.dt = dt;
    this.keyToIndex = new Map();
    this.refKey = null;
    this.n = 0;
    this.conductances = []; // {a, b, G}
    this.caps = [];         // {a, b, C, vPrev}
    this.loads = [];        // {a, b, obj}   obj.current: amps flowing a -> b
    this.sources = [];      // {a, b, obj}   obj.norton(): {G, I}
    this.wires = [];        // {a, b, wire, G}
    this.dirty = true;
    this.v = null;
  }
  static key(port) { return `${port.owner.id}:${port.node}`; }

  setReference(port) { this.refKey = DCNetwork.key(port); }
  node(port) {
    const k = DCNetwork.key(port);
    if (k === this.refKey) return -1;
    if (!this.keyToIndex.has(k)) this.keyToIndex.set(k, this.n++);
    return this.keyToIndex.get(k);
  }
  addConductance(pa, pb, G) { this.conductances.push({ a: this.node(pa), b: this.node(pb), G }); }
  addCapacitor(pa, pb, C) { this.caps.push({ a: this.node(pa), b: this.node(pb), C }); }
  addLoad(pa, pb, obj) { this.loads.push({ a: this.node(pa), b: this.node(pb), obj }); }
  addSource(pa, pb, obj) { this.sources.push({ a: this.node(pa), b: this.node(pb), obj }); }
  addWire(wire) {
    this.wires.push({ a: this.node(wire.a), b: this.node(wire.b), wire, G: 1 / wire.resistance() });
  }

  finalize() {
    this.v = new Float64Array(this.n);
    this.rhs = new Float64Array(this.n);
    this.factor();
  }

  factor() {
    const n = this.n, A = new Float64Array(n * n);
    const stamp = (a, b, G) => {
      if (a >= 0) A[a * n + a] += G;
      if (b >= 0) A[b * n + b] += G;
      if (a >= 0 && b >= 0) { A[a * n + b] -= G; A[b * n + a] -= G; }
    };
    for (const c of this.conductances) stamp(c.a, c.b, c.G);
    for (const w of this.wires) { w.G = 1 / w.wire.resistance(); stamp(w.a, w.b, w.G); }
    for (const c of this.caps) stamp(c.a, c.b, c.C / this.dt);
    for (const s of this.sources) { s.nort = s.obj.norton(); stamp(s.a, s.b, s.nort.G); }
    // A tiny leak to the reference keeps isolated nodes solvable (like the
    // megaohm insulation resistance every real node has).
    for (let i = 0; i < n; i++) A[i * n + i] += 1e-9;
    this.lu = luFactor(A, n);
    this.dirty = false;
  }

  vOf(i) { return i < 0 ? 0 : this.v[i]; }
  voltage(port) {
    const k = DCNetwork.key(port);
    if (k === this.refKey) return 0;
    const i = this.keyToIndex.get(k);
    return i === undefined ? NaN : this.v[i];
  }
  // Voltage between two ports (p relative to n).
  across(p, n) { return this.voltage(p) - this.voltage(n); }

  step() {
    for (const s of this.sources) if (s.obj.changed) { s.obj.changed = false; this.dirty = true; }
    if (this.dirty) this.factor();
    const rhs = this.rhs.fill(0), v = this.v, dt = this.dt;
    const inj = (a, b, I) => { if (a >= 0) rhs[a] += I; if (b >= 0) rhs[b] -= I; };
    for (const c of this.caps) inj(c.a, c.b, (c.C / dt) * (this.vOf(c.a) - this.vOf(c.b)));
    for (const s of this.sources) inj(s.a, s.b, s.nort.I);
    for (const l of this.loads) inj(l.a, l.b, -l.obj.current);
    this.lu.solve(rhs, v);
    for (const w of this.wires) w.wire.record((this.vOf(w.a) - this.vOf(w.b)) * w.G);
    for (const s of this.sources) s.obj.afterSolve?.(this.vOf(s.a) - this.vOf(s.b));
  }

  // Wire resistance drifts with temperature; refactor when it moved enough.
  checkDrift() {
    for (const w of this.wires) {
      const G = 1 / w.wire.resistance();
      if (Math.abs(G - w.G) / w.G > 0.005) { this.dirty = true; return; }
    }
  }
}
