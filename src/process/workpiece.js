// The workpiece on the build plate, stored as a height map (one column of
// material per grid cell, in the plate's own rotating frame).
//
// Both processes act on the same map:
//   additive:    molten plastic leaving the nozzle raises cells under the nozzle
//   subtractive: a spinning cutter lowers every cell under its end face
// Volume is conserved in both directions, so what the extruder pushed out is
// what the part gains, and what the cutter removes sets the cutting power.

import { Component } from '../core/graph.js';
import { tfInv, tfApply, m3mulv, m3T } from '../core/linalg.js';

export const MATERIALS = {
  none: { id: 0, name: 'none', density: 0, cutEnergy: 0, color: '#000000' },
  pla: { id: 1, name: 'PLA (printed)', density: 1240, cutEnergy: 1.0e8, color: '#e67e22' },
  wax: { id: 2, name: 'machinable wax', density: 920, cutEnergy: 2.5e7, color: '#5dade2' },
  foam: { id: 3, name: 'PU tooling foam', density: 320, cutEnergy: 4e6, color: '#f7dc6f' },
  pine: { id: 4, name: 'pine', density: 500, cutEnergy: 5e7, color: '#d4ac6e' },
};
export const MATERIAL_BY_ID = Object.values(MATERIALS);

export class Workpiece extends Component {
  constructor(id, { body, size = 0.12, cell = 0.4e-3, ...opts }) {
    super(id, { ...opts, kind: 'workpiece', mount: { body, p: [0, 0, 0] } });
    this.body = body;
    this.size = [size, size, 0.001];
    this.cell = cell;
    this.n = Math.round(size / cell);
    this.half = size / 2;
    this.h = new Float32Array(this.n * this.n);
    this.mat = new Uint8Array(this.n * this.n);
    this.dirty = { x0: 0, y0: 0, x1: this.n - 1, y1: this.n - 1, any: true };
    this.volumeAdded = 0; this.volumeRemoved = 0;
    this.surface = this.addPort('surface', 'mat', 'surface', { required: false });
  }

  // Place a rectangular block of stock centred on the plate.
  addStock(material, sx, sy, sz, cx = 0, cy = 0) {
    for (let j = 0; j < this.n; j++)
      for (let i = 0; i < this.n; i++) {
        const x = (i + 0.5) * this.cell - this.half, y = (j + 0.5) * this.cell - this.half;
        if (Math.abs(x - cx) <= sx / 2 && Math.abs(y - cy) <= sy / 2) {
          const k = j * this.n + i;
          this.h[k] = sz; this.mat[k] = material.id;
        }
      }
    this.markAll();
  }
  markAll() { this.dirty = { x0: 0, y0: 0, x1: this.n - 1, y1: this.n - 1, any: true }; }
  mark(i0, j0, i1, j1) {
    const d = this.dirty;
    if (!d.any) { d.x0 = i0; d.y0 = j0; d.x1 = i1; d.y1 = j1; d.any = true; return; }
    d.x0 = Math.min(d.x0, i0); d.y0 = Math.min(d.y0, j0); d.x1 = Math.max(d.x1, i1); d.y1 = Math.max(d.y1, j1);
  }
  toLocal(pWorld) { return tfApply(tfInv(this.body.T), pWorld); }
  toWorldDir(dLocal) { return m3mulv(this.body.T.R, dLocal); }
  toLocalDir(dWorld) { return m3mulv(m3T(this.body.T.R), dWorld); }
  cellIndex(x, y) {
    const i = Math.floor((x + this.half) / this.cell), j = Math.floor((y + this.half) / this.cell);
    if (i < 0 || j < 0 || i >= this.n || j >= this.n) return -1;
    return j * this.n + i;
  }
  heightAt(x, y) { const k = this.cellIndex(x, y); return k < 0 ? 0 : this.h[k]; }

  // Deposit volume V along the local segment a->b (nozzle tip positions).
  // Molten plastic is squeezed between nozzle and surface: it cannot rise
  // above the nozzle tip, so it spreads sideways until it fits. That is what
  // sets the bead width.
  deposit(a, b, V, nozzleR, material = MATERIALS.pla) {
    if (V <= 0) return 0;
    const A = this.cell * this.cell;
    const zTip = Math.min(a[2], b[2]);
    let left = V;
    const cx = 0.5 * (a[0] + b[0]), cy = 0.5 * (a[1] + b[1]);
    const segLen = Math.hypot(b[0] - a[0], b[1] - a[1]);
    for (let ring = 0; ring < 8 && left > 1e-15; ring++) {
      const r = nozzleR + ring * this.cell;
      const cells = this.cellsNearSegment(a, b, r);
      if (!cells.length) break;
      let room = 0;
      for (const k of cells) room += Math.max(0, zTip - this.h[k]) * A;
      if (room <= 0) continue;
      const frac = Math.min(1, left / room);
      for (const k of cells) {
        const add = Math.max(0, zTip - this.h[k]) * frac;
        if (add > 0) { this.h[k] += add; this.mat[k] = material.id; }
      }
      left -= Math.min(left, room);
      this.markAround(cx, cy, r + segLen);
    }
    // Nozzle far above the surface (or no room): the strand still lands
    // somewhere below. Drop the rest directly under the nozzle.
    if (left > 0) {
      const k = this.cellIndex(b[0], b[1]);
      if (k >= 0) { this.h[k] += left / A; this.mat[k] = material.id; this.markAround(b[0], b[1], this.cell); }
    }
    this.volumeAdded += V;
    return V;
  }

  // End mill of radius R with its tip at local point p (axis close to
  // vertical). A flat mill removes everything above the tip within the
  // radius; a ball-nosed mill removes everything above its spherical end.
  cut(p, R, shape = 'flat') {
    const A = this.cell * this.cell;
    let removed = 0;
    const byMat = new Float64Array(MATERIAL_BY_ID.length);
    const i0 = Math.max(0, Math.floor((p[0] - R + this.half) / this.cell));
    const i1 = Math.min(this.n - 1, Math.floor((p[0] + R + this.half) / this.cell));
    const j0 = Math.max(0, Math.floor((p[1] - R + this.half) / this.cell));
    const j1 = Math.min(this.n - 1, Math.floor((p[1] + R + this.half) / this.cell));
    for (let j = j0; j <= j1; j++)
      for (let i = i0; i <= i1; i++) {
        const x = (i + 0.5) * this.cell - this.half, y = (j + 0.5) * this.cell - this.half;
        const r2 = (x - p[0]) ** 2 + (y - p[1]) ** 2;
        if (r2 > R * R) continue;
        const k = j * this.n + i;
        const lift = shape === 'ball' ? R - Math.sqrt(R * R - r2) : 0;
        const dh = this.h[k] - Math.max(0, p[2] + lift);
        if (dh > 0) {
          removed += dh * A;
          byMat[this.mat[k]] += dh * A;
          this.h[k] -= dh;
          if (this.h[k] <= 1e-7) { this.h[k] = 0; this.mat[k] = 0; }
        }
      }
    if (removed > 0) this.mark(i0, j0, i1, j1);
    this.volumeRemoved += removed;
    return { removed, byMat };
  }

  cellsNearSegment(a, b, r) {
    const out = [];
    const i0 = Math.max(0, Math.floor((Math.min(a[0], b[0]) - r + this.half) / this.cell));
    const i1 = Math.min(this.n - 1, Math.floor((Math.max(a[0], b[0]) + r + this.half) / this.cell));
    const j0 = Math.max(0, Math.floor((Math.min(a[1], b[1]) - r + this.half) / this.cell));
    const j1 = Math.min(this.n - 1, Math.floor((Math.max(a[1], b[1]) + r + this.half) / this.cell));
    const dx = b[0] - a[0], dy = b[1] - a[1], L2 = dx * dx + dy * dy;
    for (let j = j0; j <= j1; j++)
      for (let i = i0; i <= i1; i++) {
        const x = (i + 0.5) * this.cell - this.half, y = (j + 0.5) * this.cell - this.half;
        let t = L2 > 0 ? ((x - a[0]) * dx + (y - a[1]) * dy) / L2 : 0;
        t = Math.max(0, Math.min(1, t));
        const ex = a[0] + t * dx - x, ey = a[1] + t * dy - y;
        if (ex * ex + ey * ey <= r * r) out.push(j * this.n + i);
      }
    return out;
  }
  markAround(x, y, r) {
    const i0 = Math.max(0, Math.floor((x - r + this.half) / this.cell));
    const i1 = Math.min(this.n - 1, Math.floor((x + r + this.half) / this.cell));
    const j0 = Math.max(0, Math.floor((y - r + this.half) / this.cell));
    const j1 = Math.min(this.n - 1, Math.floor((y + r + this.half) / this.cell));
    if (i1 >= i0 && j1 >= j0) this.mark(i0, j0, i1, j1);
  }

  // Mass and polar moment of inertia of the part (for the turntable).
  massProperties() {
    const A = this.cell * this.cell;
    let m = 0, J = 0;
    for (let j = 0; j < this.n; j++)
      for (let i = 0; i < this.n; i++) {
        const k = j * this.n + i;
        if (!this.h[k]) continue;
        const x = (i + 0.5) * this.cell - this.half, y = (j + 0.5) * this.cell - this.half;
        const dm = this.h[k] * A * MATERIAL_BY_ID[this.mat[k]].density;
        m += dm; J += dm * (x * x + y * y);
      }
    return { m, J };
  }
  inspect() {
    const { m } = this.massProperties();
    return { added_mm3: this.volumeAdded * 1e9, removed_mm3: this.volumeRemoved * 1e9, mass_g: m * 1e3 };
  }
}
