// Thermal network and process (deposition, cutting, extrusion) checks.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Assembly, Body } from '../src/core/graph.js';
import { ThermalMass, Ambient, ThermalContact } from '../src/thermal/thermal.js';
import { Workpiece, MATERIALS } from '../src/process/workpiece.js';
import { Extruder } from '../src/process/extruder.js';
import { AMBIENT } from '../src/core/units.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg}: ${a} vs ${b}`);
const world = new Body('world');

test('lumped mass cools exponentially with tau = C / G', () => {
  const A = new Assembly();
  const m = A.add(new ThermalMass('m', { C: 11, T0: AMBIENT + 100, mount: { body: world, p: [0, 0, 0] } }));
  const air = A.add(new Ambient('air', { mount: { body: world, p: [0, 0, 0] }, faces: 1 }));
  const c = A.connect(new ThermalContact('c', m.port('face'), air.port('air0'), { G: 0.055 }));
  const dt = 0.01, tau = 11 / 0.055;
  for (let t = 0; t < 100; t += dt) { c.exchange(); m.thermalStep(dt); air.thermalStep(); }
  near(m.T - AMBIENT, 100 * Math.exp(-100 / tau), 5e-3, 'temperature rise after 100 s');
});

test('heater steady state: temperature rise = power / conductance', () => {
  const A = new Assembly();
  const m = A.add(new ThermalMass('m', { C: 11, mount: { body: world, p: [0, 0, 0] } }));
  const air = A.add(new Ambient('air', { mount: { body: world, p: [0, 0, 0] }, faces: 1 }));
  const c = A.connect(new ThermalContact('c', m.port('face'), air.port('air0'), { G: 0.055 }));
  m.P = 10;
  for (let t = 0; t < 3000; t += 0.05) { c.exchange(); m.thermalStep(0.05); air.thermalStep(); }
  near(m.T - AMBIENT, 10 / 0.055, 1e-3, 'steady rise');
});

test('deposition conserves volume and cannot rise above the nozzle', () => {
  const wp = new Workpiece('wp', { body: world, size: 0.04, cell: 0.4e-3 });
  const V = 20e-9; // 20 mm^3
  wp.deposit([-0.005, 0, 0.3e-3], [0.005, 0, 0.3e-3], V, 0.22e-3, MATERIALS.pla);
  let sum = 0, maxH = 0;
  for (const h of wp.h) { sum += h * wp.cell * wp.cell; maxH = Math.max(maxH, h); }
  near(sum, V, 1e-6, 'deposited volume (float32 heights)');
  assert.ok(maxH <= 0.3e-3 + 1e-10, `bead height ${maxH}`);
});

test('cutting removes exactly the material above the tip inside the cutter radius', () => {
  const wp = new Workpiece('wp', { body: world, size: 0.04, cell: 0.2e-3 });
  wp.addStock(MATERIALS.wax, 0.03, 0.03, 0.005);
  const R = 1.5875e-3;
  const { removed } = wp.cut([0, 0, 0.004], R);
  near(removed, Math.PI * R * R * 1e-3, 0.07, 'removed volume (0.2 mm grid vs 1.6 mm radius)');
  assert.ok(Math.abs(wp.heightAt(0, 0) - 0.004) < 1e-9);
});

test('extruder: steady flow equals filament feed x area; pressure lags the motor', () => {
  const ex = new Extruder('ex', { mount: { body: world, p: [0, 0, 0] } });
  ex.Tmelt = 273.15 + 210;
  const feed = 2e-3; // 2 mm/s of filament -> 4.8 mm^3/s
  const w = feed / ex.r;
  const dt = 25e-6;
  let t63 = null;
  const Qss = feed * ex.A;
  for (let t = 0; t < 1.0; t += dt) {
    ex.drive.theta += w * dt; ex.drive.omega = w;
    ex.update(dt);
    if (t63 === null && ex.Q > 0.632 * Qss) t63 = t;
  }
  near(ex.Q, Qss, 0.01, 'steady melt flow');
  assert.ok(t63 > 0.003 && t63 < 0.2, `pressure time constant ${t63}s`);
  assert.ok(ex.F > 5 && ex.F < 40, `extrusion force ${ex.F} N`);
});
