// Export the built machine as plain data (JSON): the bridge to a native
// implementation. Everything a simulator in another language needs to build
// the same machine is here, with no JavaScript objects or code:
//   bodies        rigid bodies and their poses at build time
//   components    kind, class, mount, size, mass, parameters, ports, state layout
//   connections   kind, class, the two ports, parameters, route, state layout
//   circuits      every traced series loop (which elements, in which direction)
//   supplyNetwork nodes and branches of the DC network
//   logicNets     every signal net: source pin and receivers with R, C, length
//   schedule      the step order (stages, operations)
// A native port reads this file, allocates one struct per component class,
// resolves ports to indices, and runs the schedule. The JavaScript run's
// state hash (core/state.js) and recorded traces then check the port.

import { PORT_FIELDS } from './graph.js';

const round = (v) => (typeof v === 'number' && Number.isFinite(v) ? +v.toPrecision(12) : v);

// Parameters must be plain data: numbers, strings, booleans, arrays and
// objects of those. Anything else is reported rather than silently dropped.
function plain(v, where, problems) {
  if (v == null || typeof v === 'string' || typeof v === 'boolean') return v;
  if (typeof v === 'number') return Number.isFinite(v) ? round(v) : String(v);
  if (Array.isArray(v) || ArrayBuffer.isView(v)) return Array.from(v, (x) => plain(x, where, problems));
  if (typeof v === 'object' && Object.getPrototypeOf(v) === Object.prototype) {
    const o = {};
    for (const [k, x] of Object.entries(v)) o[k] = plain(x, `${where}.${k}`, problems);
    return o;
  }
  problems.push(`${where}: parameter is not plain data (${v?.constructor?.name ?? typeof v})`);
  return null;
}

const pose = (T) => ({ R: T.R.map(round), p: T.p.map(round) });

export function exportModel(m, { schedule } = {}) {
  const problems = [];
  const params = (obj) => {
    const out = {};
    for (const k of obj.constructor.PARAMS ?? []) out[k] = plain(obj[k], `${obj.id}.${k}`, problems);
    return out;
  };
  const components = [...m.A.components.values()].map((c) => ({
    id: c.id, kind: c.kind, class: c.constructor.name, label: c.label,
    mount: c.mount ? { body: c.mount.body.name, p: c.mount.p.map(round), R: c.mount.R?.map(round) ?? null } : null,
    size: c.size.map(round), mass: round(c.mass),
    params: params(c),
    state: c.constructor.STATE,
    ports: [...c.ports.values()].map((p) => ({
      name: p.name, domain: p.domain, role: p.role, node: p.node, at: p.at.map(round),
      fields: PORT_FIELDS[p.domain], through: c.through.get(p.name)?.other ?? null,
    })),
  }));
  const connections = [...m.A.connections.values()].map((w) => ({
    id: w.id, kind: w.kind, class: w.constructor.name, a: w.a.id, b: w.b.id,
    params: params(w), state: w.constructor.STATE,
    route: w.route.map((r) => ({ body: r.body.name, p: r.p.map(round) })),
  }));
  const elemId = (el) => el.id ?? el.label;
  const circuits = [];
  for (const c of m.A.components.values())
    for (const key of ['loop', 'loopA', 'loopB']) {
      const L = c[key];
      if (L && L.items) circuits.push({ owner: c.id, name: key, start: L.start.id, end: L.end?.id ?? null, items: L.items.map(({ el, sign }) => ({ el: elemId(el), sign })) });
    }
  const net = m.net;
  const nodeName = new Map([...net.keyToIndex].map(([k, i]) => [i, k]));
  const supplyNetwork = {
    dt: round(net.dt), reference: net.refKey, nodes: [...net.keyToIndex.keys()],
    wires: net.wires.map((w) => ({ wire: w.wire.id, a: nodeName.get(w.a) ?? net.refKey, b: nodeName.get(w.b) ?? net.refKey })),
    capacitors: net.caps.map((c) => ({ a: nodeName.get(c.a) ?? net.refKey, b: nodeName.get(c.b) ?? net.refKey, C: round(c.C) })),
  };
  const logicNets = [...m.nets.values()].map((n) => ({
    source: n.source.id,
    receivers: n.receivers.map((r) => ({ port: r.port.id, R: round(r.R), C: round(r.C), length: round(r.length) })),
  }));
  return {
    format: 'multi-arm-model', version: 1,
    units: 'SI (m, kg, s, A, V, K, rad)',
    dt: m.sim.dt,
    bodies: [...m.A.bodies.values()].map((b) => ({ name: b.name, T: pose(b.T) })),
    components, connections, circuits, supplyNetwork, logicNets,
    schedule: schedule ?? null,
    problems,
  };
}
