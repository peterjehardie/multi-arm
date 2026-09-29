// Slicer and CAM checks.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { knobMesh } from '../src/cam/models.js';
import { writeBinarySTL, parseSTL, rasterizeTop, meshBounds } from '../src/cam/mesh.js';
import { sliceMesh, printGcode } from '../src/cam/slicer.js';
import { dropCutter, cutterLift, finishPass } from '../src/cam/cam.js';
import { parseDirectives } from '../src/cam/scenario.js';

const knob = knobMesh({ R: 8, wallH: 3.5, H: 6.5, flutes: 9 });

test('STL write/read round trip keeps every triangle', () => {
  const back = parseSTL(writeBinarySTL(knob));
  assert.equal(back.length, knob.length);
  for (let i = 0; i < knob.length; i += 97) assert.ok(Math.abs(back[i] - knob[i]) < 1e-5);
  const { lo, hi } = meshBounds(back);
  assert.ok(Math.abs(hi[2] - 6.5) < 1e-4 && Math.abs(lo[2]) < 1e-6);
});

test('slicer: layer count, walls inside the outline, plausible filament use', () => {
  const s = sliceMesh(knob, { layer: 0.3, width: 0.5 });
  assert.equal(s.layers.length, Math.ceil(6.5 / 0.3));
  // Every outer wall point lies inside the part radius minus half a bead (flutes add 6 %).
  const L0 = s.layers[0];
  for (const { loop, p } of L0.walls)
    if (p === 0) for (const [x, y] of loop) assert.ok(Math.hypot(x, y) < 8 * 1.06 - 0.25 + 0.05, `wall point at r=${Math.hypot(x, y)}`);
  const job = printGcode(s);
  // Deposited volume = filament x area; must lie between an empty shell and the full solid.
  const vol = job.stats.filament_mm * Math.PI * 0.875 ** 2;
  const full = Math.PI * 8 * 8 * 5.2; // rough solid volume of the knob, mm^3
  assert.ok(vol > 0.25 * full && vol < 1.05 * full, `deposited ${vol.toFixed(0)} mm^3 vs solid ~${full.toFixed(0)}`);
  assert.ok(job.lines.some((l) => /^G1 X.* E/.test(l)));
});

test('drop cutter never gouges the target surface', () => {
  const hm = rasterizeTop(knob, { x0: -11, y0: -11, cell: 0.2, nx: 110, ny: 110 });
  const R = 1.5875;
  for (let k = 0; k < 400; k++) {
    const x = -10 + (k % 20), y = -10 + Math.floor(k / 20);
    const z = dropCutter(hm, x, y, { R, shape: 'ball' });
    // The ball's surface over every target cell under it must be at or above that cell.
    for (let j = 0; j < hm.ny; j++)
      for (let i = 0; i < hm.nx; i++) {
        const px = hm.x0 + (i + 0.5) * hm.cell, py = hm.y0 + (j + 0.5) * hm.cell;
        const r = Math.hypot(px - x, py - y);
        if (r > R) continue;
        assert.ok(z + cutterLift('ball', R, r) >= hm.h[j * hm.nx + i] - 1e-6, `gouge at (${x},${y})`);
      }
  }
  const pass = finishPass(hm, { R, stepover: 0.5 });
  assert.ok(pass.lines.length > 10);
});

test('scenario directives describe the stock on the plate', () => {
  const d = parseDirectives('; @stock wax 24 24 8 -30 0\nG28');
  assert.deepEqual(d.stock, [{ material: 'wax', size: [24, 24, 8], centre: [-30, 0] }]);
});
