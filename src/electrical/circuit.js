// Conductors and series circuits.
//
// A Wire is a physical conductor: copper of a given gauge, cut to a length
// that follows its route through the machine, crimped into terminals at both
// ends. Its resistance comes from that geometry and its temperature, and it
// heats up from the current it carries.
//
// A Loop is a closed series path found by walking from one terminal through
// wires and through components (a heater's element, a motor coil, a
// tool-changer contact) back to another terminal. The loop's resistance,
// inductance and back-EMF are the sums over its parts, so a longer wire or a
// dirty contact changes the current exactly as it would on the bench.

import { Connection } from '../core/graph.js';
import { COPPER, AMBIENT, awgArea, awgDiameter } from '../core/units.js';

export class Wire extends Connection {
  constructor(id, a, b, opts = {}) {
    super(id, a, b, { ...opts, kind: opts.kind ?? 'wire' });
    this.awg = opts.awg ?? 22;
    this.color = opts.color ?? '#c0392b';
    this.slack = opts.slack ?? 0.05;            // extra length for service loops [m]
    this.contactR = opts.contactR ?? 0.005;      // crimp + connector per end [ohm]
    this.capPerM = opts.capPerM ?? 60e-12;       // to neighbouring conductors [F/m]
    this.indPerM = opts.indPerM ?? 0.8e-6;       // loop inductance share [H/m]
    this.insulation = opts.insulation ?? 0.35e-3; // insulation thickness [m]
    this.fixedLength = opts.length ?? null;
    this.length = this.fixedLength ?? 0;         // set by measure()
    this.T = AMBIENT;                            // conductor temperature [K]
    this.i = 0;                                  // present current a->b [A]
    this.iRmsAcc = 0; this.accT = 0;             // for heating (mean square current)
  }
  // Cut the wire to length: route length at the build pose plus slack.
  measure() {
    if (this.fixedLength == null) this.length = this.pathLength() + this.slack;
    const A = awgArea(this.awg);
    const d = awgDiameter(this.awg) + 2 * this.insulation;
    this.area = A;
    this.heatCap = COPPER.density * A * this.length * COPPER.cp * 1.4; // copper + insulation
    this.gAir = 9 * Math.PI * d * this.length;   // natural convection, h ~ 9 W/(m^2 K)
  }
  resistance() {
    const rho = COPPER.rho20 * (1 + COPPER.alpha * (this.T - (273.15 + 20)));
    return (rho * this.length) / this.area + 2 * this.contactR;
  }
  capacitance() { return this.capPerM * this.length; }
  inductance() { return this.indPerM * this.length; }
  record(i) { this.i = i; }
  // Called from the slow thermal stage.
  heat(dt) {
    const P = this.i * this.i * this.resistance();
    this.T += (dt * (P - this.gAir * (this.T - AMBIENT))) / this.heatCap;
  }
  inspect() {
    return {
      gauge: `${this.awg} AWG`, length_m: this.length, R_ohm: this.resistance(),
      I_A: this.i, T_C: this.T - 273.15, P_W: this.i * this.i * this.resistance(),
    };
  }
}

// Simple internal series elements used by components.
export class Resistor {
  constructor(R, label = 'R') { this.R = R; this.i = 0; this.label = label; }
  resistance() { return this.R; }
  record(i) { this.i = i; }
}

// A contact that can open (switch, pogo pin, slip-ring brush).
export class Contact {
  constructor(Ron = 0.01, label = 'contact') {
    this.Ron = Ron; this.closed = false; this.i = 0; this.label = label;
    this.ratedA = Infinity; this.overCurrentTime = 0;
  }
  resistance() { return this.closed ? this.Ron : Infinity; }
  record(i) { this.i = i; }
}

// Walk from `start` to `end` through connections and component internals.
// Returns a Loop, or null if no path exists. A closed (finite-resistance)
// path is preferred; if none exists the open path is returned, so the circuit
// still exists physically but carries no current.
// `end` may be a Port or a predicate (port) => boolean.
export function traceLoop(start, end) {
  return tracePass(start, end, true) ?? tracePass(start, end, false);
}

function tracePass(start, end, closedOnly) {
  const isEnd = typeof end === 'function' ? end : (q) => q === end;
  const ok = (el) => !closedOnly || Number.isFinite(el.resistance());
  const items = [];
  const visited = new Set();
  let endPort = null;
  function walkFromPort(port) {
    // Leave this port along one of its connections.
    for (const conn of port.connections) {
      if (visited.has(conn)) continue;
      if (!conn.resistance || !ok(conn)) continue; // only electrical conductors carry current
      visited.add(conn);
      const next = conn.other(port);
      items.push({ el: conn, sign: port === conn.a ? +1 : -1 });
      if (isEnd(next)) { endPort = next; return true; }
      // Continue through the component we arrived at.
      const thr = next.owner.through.get(next.name);
      if (thr && !visited.has(thr.element) && ok(thr.element)) {
        visited.add(thr.element);
        items.push({ el: thr.element, sign: thr.sign });
        const out = next.owner.port(thr.other);
        if (isEnd(out)) { endPort = out; return true; }
        if (walkFromPort(out)) return true;
        items.pop();
        visited.delete(thr.element);
      }
      // Or continue along another wire landing on the same copper (a terminal
      // block, a splice, a pass-through pin).
      if (next.owner.passive || next.role === 'passthru') {
        for (const q of next.owner.ports.values()) {
          if (q.node !== next.node || q.domain !== 'elec') continue;
          if (q !== next && isEnd(q)) { endPort = q; return true; }
          if (walkFromPort(q)) return true;
        }
      }
      items.pop();
      visited.delete(conn);
    }
    return false;
  }
  if (!walkFromPort(start)) return null;
  return new Loop(items, start, endPort);
}

export class Loop {
  constructor(items, start, end) {
    this.items = items;
    this.start = start;
    this.end = end;
    this.i = 0;
  }
  resistance() {
    let R = 0;
    for (const { el } of this.items) R += el.resistance();
    return R;
  }
  inductance() {
    let L = 0;
    for (const { el } of this.items) if (el.inductance) L += el.inductance();
    return L;
  }
  // Sum of internal voltage drops (back-EMF) in the direction of traversal.
  emf() {
    let e = 0;
    for (const { el, sign } of this.items) if (el.emf) e += sign * el.emf();
    return e;
  }
  setCurrent(i) {
    this.i = i;
    for (const { el, sign } of this.items) el.record(sign * i);
  }
  // Advance the loop current with a constant applied voltage u over dt.
  // The loop obeys u = R i + L di/dt + e. With u and e held for the step the
  // solution is an exact exponential, stable for any dt.
  // extraR adds series resistance that lives in the source (switch on-resistance).
  step(u, dt, extraR = 0, R = this.resistance() + extraR, L = this.inductance(), e = this.emf()) {
    let i;
    if (!Number.isFinite(R)) i = 0;
    else if (L < 1e-9) i = (u - e) / R;
    else {
      const iss = (u - e) / R;
      i = iss + (this.i - iss) * Math.exp((-dt * R) / L);
    }
    this.setCurrent(i);
    return i;
  }
  describe() {
    return this.items.map(({ el }) => el.id ?? el.label ?? '?');
  }
}
