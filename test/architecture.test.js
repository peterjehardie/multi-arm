// Architecture checks: the properties that make a port to another language
// (C, C++, Rust) a translation rather than a redesign.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { Machine, SCHEDULE } from '../src/machine/machine.js';
import { Component, Connection, PORT_FIELDS } from '../src/core/graph.js';
import { stateTable, snapshot, restore, hashState } from '../src/core/state.js';
import { exportModel } from '../src/core/export.js';
import { goldenRun } from './golden/run.js';

const demo = readFileSync(new URL('../scenarios/demo.gcode', import.meta.url), 'utf8');

test('every part declares its parameters and its state explicitly', () => {
  const m = new Machine();
  const passive = new Set(['TerminalBlock', 'SlipRing', 'Part']); // copper and brackets: no own state
  for (const obj of [...m.A.components.values(), ...m.A.connections.values()]) {
    const C = obj.constructor;
    if (passive.has(C.name)) continue;
    const own = (k) => Object.prototype.hasOwnProperty.call(C, k);
    assert.ok(own('PARAMS') && own('STATE'), `${C.name} (${obj.id}) must declare static PARAMS and STATE`);
  }
  const t = stateTable(m);
  const bad = t.scalars.filter((e) => Number.isNaN(e.read())).map((e) => e.label);
  assert.deepEqual(bad, [], 'every declared state value resolves to a number');
});

test('ports carry only the fields declared for their domain', () => {
  const m = new Machine();
  const base = new Set(['owner', 'name', 'domain', 'role', 'node', 'required', 'at', 'label', 'connections', 'dcIndex', 'worldPos']);
  for (const c of m.A.components.values())
    for (const p of c.ports.values()) {
      const extra = Object.keys(p).filter((k) => !base.has(k) && !PORT_FIELDS[p.domain].includes(k));
      assert.deepEqual(extra, [], `${p.id} has undeclared fields`);
    }
});

test('the simulation is deterministic: identical runs give identical state', () => {
  const run = () => { const m = new Machine({ preheated: true, startHomed: true }); m.host.load(demo); m.run(2); return hashState(m); };
  assert.equal(run(), run());
});

test('snapshot and restore reproduce the exact state', () => {
  const m = new Machine({ preheated: true, startHomed: true });
  m.host.load(demo);
  m.run(1);
  const table = stateTable(m);
  const snap = snapshot(m, table);
  const h0 = hashState(m, table);
  m.run(0.3);
  assert.notEqual(hashState(m, table), h0);
  restore(m, snap, table);
  assert.equal(hashState(m, table), h0);
});

test('the machine exports as plain data with no gaps', () => {
  const m = new Machine();
  const model = exportModel(m, { schedule: SCHEDULE });
  assert.deepEqual(model.problems, []);
  assert.equal(model.components.length, m.A.components.size);
  assert.equal(model.connections.length, m.A.connections.size);
  const back = JSON.parse(JSON.stringify(model));
  assert.equal(back.components.find((c) => c.id === 'arm').params.spec.joints.length, 4);
  for (const st of SCHEDULE) for (const [group, method] of st.ops) assert.ok(group && method);
});

test('physics matches the recorded reference trace (conformance for ports)', { timeout: 120000 }, () => {
  const ref = JSON.parse(readFileSync(new URL('./golden/demo-quickstart-3s.json', import.meta.url), 'utf8'));
  const now = goldenRun(demo, ref.samples.length * ref.every, ref.every);
  assert.deepEqual(now.signals, ref.signals);
  for (let k = 0; k < ref.samples.length; k++)
    for (let i = 0; i < ref.signals.length; i++) {
      const a = now.samples[k][i], b = ref.samples[k][i];
      assert.ok(Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b)), `${ref.signals[i]} at sample ${k}: ${a} vs ${b}`);
    }
  assert.equal(now.hash, ref.hash, 'final state hash (rerun tools/golden.js only for intended physics changes)');
});
