// CAM for the spindle: cutter paths that carve material down to a target
// shape, for a 3-axis cutter coming from above.
//
// Core idea ("drop cutter"): for each point (x, y) the cutter might visit,
// imagine lowering the cutter straight down until it first touches the
// target surface anywhere under its tip. That height is the lowest the
// cutter may go there without gouging the part. A path is a sequence of such
// points along raster lines.
//
//   finishing: one pass of closely spaced raster lines that follows the
//              target surface exactly (for a ball-nosed cutter the stepover
//              sets the small scallops left between passes).
//   roughing:  the same, but taken in horizontal slabs from the top down, each
//              slab no deeper than the cutter can take at once, leaving a thin
//              allowance for the finishing pass.
//
// The target is a height map of the part's top surface (see rasterizeTop),
// in millimetres, in the part's own frame; `place` shifts it on the plate.

export function cutterLift(shape, R, r) {
  // Height of the cutter's surface above its tip at radius r.
  if (r > R) return Infinity;
  return shape === 'ball' ? R - Math.sqrt(R * R - r * r) : 0;
}

// Lowest safe tip height at (x, y) over target height map `hm` (+ allowance).
export function dropCutter(hm, x, y, { R, shape = 'ball', allowance = 0, floor = 0 }) {
  const c = hm.cell;
  const n = Math.ceil(R / c) + 1;
  const ic = Math.floor((x - hm.x0) / c), jc = Math.floor((y - hm.y0) / c);
  let z = floor;
  for (let dj = -n; dj <= n; dj++) {
    const j = jc + dj;
    if (j < 0 || j >= hm.ny) continue;
    const py = hm.y0 + (j + 0.5) * c;
    for (let di = -n; di <= n; di++) {
      const i = ic + di;
      if (i < 0 || i >= hm.nx) continue;
      const px = hm.x0 + (i + 0.5) * c;
      const r = Math.hypot(px - x, py - y);
      if (r > R) continue;
      const h = hm.h[j * hm.nx + i];
      if (h <= 0) continue;
      const zt = h + allowance - cutterLift(shape, R, r);
      if (zt > z) z = zt;
    }
  }
  return z;
}

// Raster toolpath over the rectangle [x0,x1] x [y0,y1] (part frame).
// Each line: points every `step` mm at the drop-cutter height, clamped to
// no lower than `level` (roughing slab) and `floor` (keeps off the plate).
function rasterLines(hm, rect, { stepover, step = 0.4, level = -Infinity, clip, ...cut }) {
  const lines = [];
  let flip = false;
  for (let y = rect.y0; y <= rect.y1 + 1e-9; y += stepover) {
    const pts = [];
    for (let x = rect.x0; x <= rect.x1 + 1e-9; x += step) {
      if (clip && !clip(x, y)) continue;
      pts.push([x, y, Math.max(level, dropCutter(hm, x, y, cut))]);
    }
    if (pts.length < 2) continue;
    lines.push(flip ? pts.reverse() : pts);
    flip = !flip;
  }
  return lines;
}

// Drop points that lie on a straight line between their neighbours.
function simplify(pts, tol = 0.01) {
  if (pts.length < 3) return pts;
  const out = [pts[0]];
  for (let i = 1; i < pts.length - 1; i++) {
    const a = out[out.length - 1], b = pts[i], c = pts[i + 1];
    const f = (b[0] - a[0]) / ((c[0] - a[0]) || 1e-9);
    const zi = a[2] + f * (c[2] - a[2]);
    if (Math.abs(zi - b[2]) > tol || Math.abs(c[1] - a[1]) > 1e-9) out.push(b);
  }
  out.push(pts[pts.length - 1]);
  return out;
}

export function millGcode(passes, { place = [0, 0], safeZ, feed = 600, plunge = 120, lines = [] } = {}) {
  const [ox, oy] = place;
  const g = [...lines];
  const f3 = (v) => v.toFixed(3);
  let length = 0, time = 0;
  for (const pass of passes) {
    g.push(`; ${pass.name}`);
    const ls = pass.lines;
    for (let n = 0; n < ls.length; n++) {
      const pts = simplify(ls[n]);
      const [x, y, z] = pts[0];
      // Lift and reposition between lines unless the next line starts right
      // beside the last point (zig-zag link along the edge).
      if (n === 0 || pass.lift) {
        g.push(`G0 Z${f3(safeZ)}`, `G0 X${f3(x + ox)} Y${f3(y + oy)}`, `G1 Z${f3(z)} F${plunge}`);
      } else g.push(`G1 X${f3(x + ox)} Y${f3(y + oy)} Z${f3(z)} F${feed}`);
      for (let i = 1; i < pts.length; i++) {
        const p = pts[i], q = pts[i - 1];
        const d = Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
        length += d; time += d / (feed / 60);
        g.push(`G1 X${f3(p[0] + ox)} Y${f3(p[1] + oy)} Z${f3(p[2])} F${feed}`);
      }
    }
    g.push(`G0 Z${f3(safeZ)}`);
  }
  return { lines: g, stats: { cutPath_mm: length, time_s: time } };
}

// Finishing pass over the part's footprint (plus the cutter radius).
export function finishPass(hm, { R, shape = 'ball', stepover = 0.4, step = 0.4, floor = 0.5, margin, clip } = {}) {
  const m = margin ?? R + 0.5;
  const b = bbox(hm);
  const rect = { x0: b.x0 - m, x1: b.x1 + m, y0: b.y0 - m, y1: b.y1 + m };
  return { name: `finishing, ${shape} R${R} mm, stepover ${stepover} mm`, lines: rasterLines(hm, rect, { R, shape, stepover, step, floor, clip }) };
}

// Roughing slabs from `top` down to `bottom` inside `rect`.
export function roughPasses(hm, { R, shape = 'ball', top, bottom, stepdown, stepover, allowance = 0.3, rect, step = 0.6 }) {
  const passes = [];
  for (let level = top - stepdown; ; level -= stepdown) {
    const lv = Math.max(level, bottom);
    passes.push({
      name: `roughing slab at z=${lv.toFixed(2)} mm (allowance ${allowance} mm)`, lift: false,
      lines: rasterLines(hm, rect, { R, shape, stepover, step, level: lv, allowance, floor: bottom }),
    });
    if (lv <= bottom) break;
  }
  return passes;
}

function bbox(hm) {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (let j = 0; j < hm.ny; j++)
    for (let i = 0; i < hm.nx; i++)
      if (hm.h[j * hm.nx + i] > 0) {
        const x = hm.x0 + (i + 0.5) * hm.cell, y = hm.y0 + (j + 0.5) * hm.cell;
        x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
      }
  return { x0, x1, y0, y1 };
}
