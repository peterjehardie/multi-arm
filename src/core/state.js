// The machine's complete physical state as flat numbers.
//
// Every component and connection declares what changes as the simulation
// runs (its static STATE list). From those declarations this module builds a
// state table and can:
//   * snapshot() the whole plant into plain arrays,
//   * restore() it,
//   * hash() it, so two runs (or a JavaScript run and a native port) can be
//     compared exactly after the same inputs.
// In a native port the same table is simply the struct layout.
//
// Covered: simulator clock, body poses, every declared component and
// connection state, port values, the supply network node voltages, logic net
// levels and the controller pins' electrical levels. Not covered: the
// firmware and the host PC's queues. They are the controller side of the
// machine; in a native port the firmware runs as its own program (on an MCU
// emulator) behind the pin interface.

import { PORT_FIELDS } from './graph.js';

const get = (obj, path) => path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), obj);
function set(obj, path, v) {
  const keys = path.split('.');
  const last = keys.pop();
  const o = keys.reduce((x, k) => (x == null ? undefined : x[k]), obj);
  if (o != null) o[last] = v;
}

// Build the list of state entries for a machine: [{ label, read(), write(v) }]
// for scalars and [{ label, array }] for arrays.
export function stateTable(m) {
  const scalars = [], arrays = [];
  const num = (label, obj, path) => {
    const v0 = get(obj, path);
    const kind = typeof v0 === 'boolean' ? 'bool' : v0 == null ? 'null' : 'num';
    scalars.push({
      label, kind,
      read: () => { const v = get(obj, path); return v == null ? NaN : +v; },
      write: (v) => { if (kind === 'bool') set(obj, path, v !== 0); else if (kind === 'num') set(obj, path, v); },
    });
  };
  const addDeclared = (obj) => {
    for (const decl of obj.constructor.STATE ?? []) {
      const label = `${obj.id}.${decl}`;
      if (decl.endsWith('[]')) {
        const a = get(obj, decl.slice(0, -2));
        if (a && a.length !== undefined) arrays.push({ label, array: a });
      } else if (decl.includes(':')) {
        const [path, names] = decl.split(':');
        const opts = names.split('|');
        scalars.push({
          label, kind: 'enum',
          read: () => opts.indexOf(get(obj, path)),
          write: (v) => set(obj, path, opts[v] ?? opts[0]),
        });
      } else num(label, obj, decl);
    }
    for (const p of obj.ports?.values?.() ?? [])
      for (const f of PORT_FIELDS[p.domain]) num(`${p.id}#${f}`, p, f);
  };

  num('sim.t', m.sim, 't');
  num('sim.stepCount', m.sim, 'stepCount');
  for (const b of m.A.bodies.values())
    for (let k = 0; k < 12; k++) {
      scalars.push({
        label: `body.${b.name}.T${k}`, kind: 'num',
        read: () => (k < 9 ? b.T.R[k] : b.T.p[k - 9]),
        write: (v) => {
          const R = [...b.T.R], p = [...b.T.p];
          if (k < 9) R[k] = v; else p[k - 9] = v;
          b.T = { R, p };
        },
      });
    }
  for (const c of m.A.components.values()) addDeclared(c);
  for (const c of m.A.connections.values()) addDeclared(c);
  arrays.push({ label: 'dcnetwork.v[]', array: m.net.v });
  for (const [id, net] of m.nets) {
    scalars.push({
      label: `net.${id}.level`, kind: 'enum',
      read: () => (net.level === null ? -1 : net.level ? 1 : 0),
      write: (v) => { net.level = v < 0 ? null : v === 1; },
    });
    net.receivers.forEach((r, i) => num(`net.${id}.rx${i}.seen`, r, 'seen'));
  }
  for (const [name, p] of m.board.pins) {
    num(`board.pin.${name}.level`, p, 'level');
    num(`board.pin.${name}.inLevel`, p, 'inLevel');
  }
  return { scalars, arrays };
}

export function snapshot(m, table = stateTable(m)) {
  return {
    scalars: Float64Array.from(table.scalars, (e) => e.read()),
    arrays: table.arrays.map((e) => (ArrayBuffer.isView(e.array) ? e.array.slice() : Float64Array.from(e.array))),
  };
}

export function restore(m, snap, table = stateTable(m)) {
  table.scalars.forEach((e, i) => e.write(snap.scalars[i]));
  table.arrays.forEach((e, i) => {
    if (ArrayBuffer.isView(e.array)) e.array.set(snap.arrays[i]);
    else for (let k = 0; k < e.array.length; k++) e.array[k] = snap.arrays[i][k];
  });
}

// 64-bit FNV-1a over the raw bytes of the state (as two 32-bit halves).
export function hashState(m, table = stateTable(m)) {
  const s = snapshot(m, table);
  let h1 = 0x811c9dc5, h2 = 0x01000193 ^ 0x5bd1e995;
  const feed = (bytes) => {
    for (let i = 0; i < bytes.length; i++) {
      h1 = Math.imul(h1 ^ bytes[i], 0x01000193) >>> 0;
      h2 = Math.imul(h2 ^ bytes[i] ^ (i & 0xff), 0x01000193) >>> 0;
    }
  };
  feed(new Uint8Array(s.scalars.buffer));
  for (const a of s.arrays) feed(new Uint8Array(a.buffer, a.byteOffset, a.byteLength));
  return h1.toString(16).padStart(8, '0') + h2.toString(16).padStart(8, '0');
}
