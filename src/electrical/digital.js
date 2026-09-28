// Digital signal nets.
//
// A logic signal is a voltage on a wire. When a microcontroller pin switches,
// its output transistor (tens of ohms) charges the capacitance of the wire and
// of the receiving input. The receiver sees a new level once the voltage,
// measured against the receiver's own ground, crosses its input threshold.
// So each edge arrives after:
//   * the RC charging time up to the threshold, and
//   * the travel time along the copper (about 2/3 of light speed).
// If the two grounds sit at different potentials (motor current flowing in a
// shared ground wire), the threshold is effectively shifted. If the high level
// never reaches the receiver's threshold (3.3 V logic into a 5 V input), the
// receiver simply never sees the edge.

import { C_LIGHT } from '../core/units.js';

export class DigitalNet {
  constructor(sim, source, receivers, network) {
    this.sim = sim;
    this.source = source;        // Port with role 'dout' / 'gpio'
    this.receivers = receivers;  // [{ port, R, C, length, wires }]
    this.network = network;      // DCNetwork for ground offsets
    this.level = null;           // null = not driven (pin is an input / high impedance)
    this.lastEdgeT = -Infinity;
    // What each receiver currently sees. Undriven, a receiver sits at the
    // level its pull resistor gives it (a driver's EN pin is pulled high on
    // the carrier board, so the motor starts disabled).
    for (const r of receivers) r.seen = r.port.owner.idleLevel?.(r.port.name) ?? false;
    this.edges = 0;
    // Short history of edges for the oscilloscope: [t, level] pairs.
    this.history = [];
  }

  drive(level, t) { this.transition(level, t); }
  release(t) { this.transition(null, t); }

  transition(level, t) {
    if (level === this.level) return;
    this.level = level;
    this.edges++;
    this.lastEdgeT = t;
    if (this.history.length > 4000) this.history.splice(0, 2000);
    this.history.push(t, level === null ? 0.5 : level ? 1 : 0);
    const src = this.source.owner;
    const vHigh = src.logicHigh(this.source.name);
    const gTx = src.logicGround ? this.network.voltage(src.logicGround(this.source.name)) : 0;
    for (const r of this.receivers) {
      const rx = r.port.owner;
      const target = level === null ? (rx.idleLevel?.(r.port.name) ?? false) : level;
      if (target === r.seen) continue;
      const gRx = rx.logicGround ? this.network.voltage(rx.logicGround(r.port.name)) : 0;
      const offset = level === null ? 0 : (Number.isFinite(gTx) ? gTx : 0) - (Number.isFinite(gRx) ? gRx : 0);
      const th = rx.inputThresholds(r.port.name); // { rise, fall } volts vs receiver ground
      const vRail = level === null ? (rx.idleHigh?.(r.port.name) ?? vHigh) : vHigh;
      const Rdrive = level === null ? (rx.pullResistance?.(r.port.name) ?? 10e3) : src.outputResistance(this.source.name) + r.R;
      const tau = Rdrive * r.C;
      const vFinal = (target ? vRail : 0) + offset;
      const vStart = (target ? 0 : vRail) + offset;
      const vTh = target ? th.rise : th.fall;
      // Does the final level cross the threshold at all?
      const crosses = target ? vFinal > vTh && vStart < vTh : vFinal < vTh && vStart > vTh;
      if (!crosses) {
        r.missed = (r.missed ?? 0) + 1;
        continue;
      }
      r.seen = target;
      const tRC = tau > 0 ? tau * Math.log((vFinal - vStart) / (vFinal - vTh)) : 0;
      const tFlight = r.length / (0.66 * C_LIGHT);
      const tArrive = t + tRC + tFlight;
      this.sim.at(tArrive, (ta) => rx.digitalIn(r.port.name, target, ta), 'edge');
    }
  }

  // A receiver whose logic supply just came up reads the voltage now on its
  // pin (no edge needed: a powered-up gate simply sees the present level).
  resync(r, t) {
    const rx = r.port.owner;
    const target = this.level === null ? (rx.idleLevel?.(r.port.name) ?? false) : this.level;
    const vRail = this.level === null ? (rx.idleHigh?.(r.port.name) ?? this.source.owner.logicHigh(this.source.name)) : this.source.owner.logicHigh(this.source.name);
    const v = target ? vRail : 0;
    const th = rx.inputThresholds(r.port.name);
    const lvl = v > th.rise ? true : v < th.fall ? false : r.seen;
    if (lvl !== r.seen) { r.seen = lvl; rx.digitalIn(r.port.name, lvl, t); }
  }
}

// Find every net that starts at a digital output and build it.
export function buildDigitalNets(assembly, sim, network) {
  const nets = new Map();
  for (const c of assembly.components.values()) {
    for (const p of c.ports.values()) {
      if (p.role !== 'dout' && p.role !== 'gpio') continue;
      const receivers = [];
      walk(p, 0, 0, 0, [], new Set(), receivers);
      if (!receivers.length) continue;
      const net = new DigitalNet(sim, p, receivers, network);
      nets.set(p.id, net);
      for (const r of receivers) r.port.owner.attachInputNet?.(r.port.name, net, r);
      c.attachNet?.(p.name, net);
    }
  }
  return nets;
}

function walk(port, R, C, length, wires, visited, out) {
  for (const w of port.connections) {
    if (!w.resistance || visited.has(w)) continue;
    visited.add(w);
    const next = w.other(port);
    const R2 = R + w.resistance(), C2 = C + w.capacitance(), L2 = length + w.length;
    const ws = [...wires, w];
    if (next.role === 'din') {
      out.push({ port: next, R: R2, C: C2 + next.owner.inputCapacitance(next.name), length: L2, wires: ws });
    }
    // Pass through splices / terminal blocks / connectors on the same copper.
    if (next.owner.passive) {
      for (const q of next.owner.ports.values())
        if (q !== next && q.node === next.node) walk(q, R2, C2, L2, ws, visited, out);
    }
    const thr = next.owner.through.get(next.name);
    if (thr) {
      const q = next.owner.port(thr.other);
      walk(q, R2 + thr.element.resistance(), C2, L2, ws, visited, out);
    }
  }
}
