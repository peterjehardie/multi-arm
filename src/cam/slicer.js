// Slicer: turns a triangle mesh into printing G-code.
//
// What a slicer does, step by step:
//   1. Cut the mesh with horizontal planes, one per layer. Each triangle that
//      crosses the plane leaves a short line segment; chained end to end the
//      segments form closed outlines (outer boundaries and holes).
//   2. Walls ("perimeters"): trace each outline, shrunk inward by half a bead
//      width, then again one bead further in, so the beads sit inside the
//      part's true edge.
//   3. Fill: hatch the area inside the walls with parallel lines. Near the top
//      and bottom of the part the lines touch (solid skin); inside they are
//      spaced out (sparse infill) to save plastic and time.
//   4. Order the paths, add travel moves with retraction, and compute how much
//      filament each move pushes: bead cross-section x length / filament area.
//
// Coordinates are millimetres in the part's frame; `place` shifts the part
// on the plate. Output is work-coordinate G-code for this machine's firmware.
//
// Simplifications (demo): outlines are offset by moving vertices along their
// averaged normals (good for smooth, rounded parts; sharp inner corners and
// thin features are not handled the way a production slicer's polygon
// clipping library would); top/bottom skin is found from the part's top-
// surface height map, so skins over internal overhangs are not detected.

import { meshBounds, rasterizeTop } from './mesh.js';

export const DEFAULT_PRINT = {
  layer: 0.3, width: 0.5, perimeters: 2, infill: 0.25, topLayers: 3, bottomLayers: 2,
  filament: 1.75, place: [0, 0], sliceAt: 'mid',
  feed: { first: 900, perimeter: 1200, infill: 1800, travel: 3000 },
  retract: 0.8, retractMin: 2.0,
};

// ---------------------------------------------------------------- geometry
function sliceLayer(tris, z) {
  const segs = [];
  for (let t = 0; t < tris.length; t += 9) {
    const pts = [];
    for (let e = 0; e < 3; e++) {
      const a = t + e * 3, b = t + ((e + 1) % 3) * 3;
      const za = tris[a + 2], zb = tris[b + 2];
      if ((za < z) !== (zb < z)) {
        const f = (z - za) / (zb - za);
        pts.push([tris[a] + f * (tris[b] - tris[a]), tris[a + 1] + f * (tris[b + 1] - tris[a + 1])]);
      }
    }
    if (pts.length === 2) segs.push(pts);
  }
  return chain(segs);
}

// Join segments that share end points into closed loops.
function chain(segs) {
  const key = (p) => `${Math.round(p[0] * 1e4)},${Math.round(p[1] * 1e4)}`;
  const byPt = new Map();
  segs.forEach((s, i) => {
    for (const end of [0, 1]) {
      const k = key(s[end]);
      if (!byPt.has(k)) byPt.set(k, []);
      byPt.get(k).push([i, end]);
    }
  });
  const used = new Uint8Array(segs.length);
  const loops = [];
  for (let i = 0; i < segs.length; i++) {
    if (used[i]) continue;
    used[i] = 1;
    const loop = [segs[i][0], segs[i][1]];
    for (let guard = 0; guard < segs.length; guard++) {
      const tail = loop[loop.length - 1];
      const cand = (byPt.get(key(tail)) ?? []).find(([j]) => !used[j]);
      if (!cand) break;
      const [j, end] = cand;
      used[j] = 1;
      loop.push(segs[j][1 - end]);
    }
    if (loop.length > 3 && key(loop[0]) === key(loop[loop.length - 1])) loop.pop();
    if (loop.length >= 3) loops.push(loop);
  }
  return loops;
}

// Remove points closer than minLen to the previous kept point, and points
// on a straight line between their neighbours. Sliced outlines carry many
// tiny segments where the plane grazes triangle corners; offsetting those
// would create little zig-zags.
function clean(L, minLen = 0.15, tol = 0.01) {
  const pts = [];
  for (const p of L) {
    const q = pts[pts.length - 1];
    if (!q || Math.hypot(p[0] - q[0], p[1] - q[1]) >= minLen) pts.push(p);
  }
  if (pts.length > 2 && Math.hypot(pts[0][0] - pts[pts.length - 1][0], pts[0][1] - pts[pts.length - 1][1]) < minLen) pts.pop();
  const out = [];
  for (let i = 0; i < pts.length; i++) {
    const a = pts[(i - 1 + pts.length) % pts.length], b = pts[i], c = pts[(i + 1) % pts.length];
    const cr = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    if (Math.abs(cr) / (Math.hypot(c[0] - a[0], c[1] - a[1]) || 1) > tol) out.push(b);
  }
  return out.length >= 3 ? out : null;
}

const area = (L) => {
  let a = 0;
  for (let i = 0; i < L.length; i++) { const p = L[i], q = L[(i + 1) % L.length]; a += p[0] * q[1] - q[0] * p[1]; }
  return a / 2;
};
function inside(pt, L) {
  let c = false;
  for (let i = 0, j = L.length - 1; i < L.length; j = i++) {
    const [xi, yi] = L[i], [xj, yj] = L[j];
    if ((yi > pt[1]) !== (yj > pt[1]) && pt[0] < ((xj - xi) * (pt[1] - yi)) / (yj - yi) + xi) c = !c;
  }
  return c;
}
// Orient loops: outer boundaries counter-clockwise, holes clockwise, so that
// "into the material" is always to the left of the direction of travel.
function orient(loops) {
  return loops.map((L) => {
    const depth = loops.filter((M) => M !== L && inside(L[0], M)).length;
    const wantCCW = depth % 2 === 0;
    return (area(L) > 0) === wantCCW ? L : [...L].reverse();
  });
}
// Shift a loop d millimetres to its left (into the material).
function offset(L, d) {
  const n = L.length, out = [];
  for (let i = 0; i < n; i++) {
    const p = L[(i - 1 + n) % n], c = L[i], q = L[(i + 1) % n];
    const e1 = norm2([c[0] - p[0], c[1] - p[1]]), e2 = norm2([q[0] - c[0], q[1] - c[1]]);
    const n1 = [-e1[1], e1[0]], n2 = [-e2[1], e2[0]];
    let m = norm2([n1[0] + n2[0], n1[1] + n2[1]]);
    const cos = m[0] * n1[0] + m[1] * n1[1];
    const k = d / Math.max(0.3, cos); // miter, limited at sharp corners
    out.push([c[0] + m[0] * k, c[1] + m[1] * k]);
  }
  // A loop that turned inside out has collapsed: drop it.
  return Math.sign(area(out)) === Math.sign(area(L)) && Math.abs(area(out)) > d * d ? out : null;
}
const norm2 = (v) => { const L = Math.hypot(v[0], v[1]) || 1; return [v[0] / L, v[1] / L]; };

// Hatch lines at angle `ang` with spacing `sp` across the region bounded by
// `loops` (even-odd rule). Returns segments [[x,y],[x,y]].
function hatch(loops, ang, sp, phase = 0) {
  const c = Math.cos(ang), s = Math.sin(ang);
  const rot = (p) => [c * p[0] + s * p[1], -s * p[0] + c * p[1]];
  const unrot = (p) => [c * p[0] - s * p[1], s * p[0] + c * p[1]];
  const R = loops.map((L) => L.map(rot));
  let y0 = Infinity, y1 = -Infinity;
  for (const L of R) for (const p of L) { y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
  const out = [];
  for (let y = Math.floor(y0 / sp) * sp + phase * sp; y <= y1; y += sp) {
    const xs = [];
    for (const L of R)
      for (let i = 0; i < L.length; i++) {
        const a = L[i], b = L[(i + 1) % L.length];
        if ((a[1] > y) !== (b[1] > y)) xs.push(a[0] + ((y - a[1]) * (b[0] - a[0])) / (b[1] - a[1]));
      }
    xs.sort((a, b) => a - b);
    for (let k = 0; k + 1 < xs.length; k += 2)
      if (xs[k + 1] - xs[k] > 0.2) out.push([unrot([xs[k], y]), unrot([xs[k + 1], y])]);
  }
  return out;
}

// Split a segment into the pieces where pred(point) holds.
function splitBy(seg, pred, step = 0.4) {
  const [a, b] = seg, L = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const n = Math.max(1, Math.ceil(L / step)), out = [];
  let start = null;
  for (let i = 0; i <= n; i++) {
    const f = i / n, p = [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])];
    const ok = pred(p);
    if (ok && !start) start = p;
    if ((!ok || i === n) && start) {
      const end = ok ? p : [a[0] + ((i - 0.5) / n) * (b[0] - a[0]), a[1] + ((i - 0.5) / n) * (b[1] - a[1])];
      if (Math.hypot(end[0] - start[0], end[1] - start[1]) > 0.3) out.push([start, end]);
      start = null;
    }
  }
  return out;
}

// ---------------------------------------------------------------- slicing
export function sliceMesh(tris, opts = {}) {
  const o = { ...DEFAULT_PRINT, ...opts, feed: { ...DEFAULT_PRINT.feed, ...(opts.feed ?? {}) } };
  const { lo, hi } = meshBounds(tris);
  const top = rasterizeTop(tris, { x0: lo[0] - 1, y0: lo[1] - 1, cell: 0.25, nx: Math.ceil((hi[0] - lo[0] + 2) / 0.25), ny: Math.ceil((hi[1] - lo[1] + 2) / 0.25) });
  const nLayers = Math.ceil((hi[2] - lo[2]) / o.layer - 1e-6);
  const layers = [];
  for (let k = 0; k < nLayers; k++) {
    const zb = lo[2] + k * o.layer, zt = zb + o.layer;
    // 'bottom' slices each layer at its lower face: up-facing surfaces come
    // out slightly oversize, leaving material for a finishing cut.
    const zs = o.sliceAt === 'bottom' ? zb + 1e-3 : zb + o.layer / 2;
    const outlines = orient(sliceLayer(tris, zs).map((L) => clean(L)).filter(Boolean));
    if (!outlines.length) continue;
    const walls = [];
    for (let p = 0; p < o.perimeters; p++)
      for (const L of outlines) { const w = offset(L, o.width * (p + 0.5)); if (w) walls.push({ loop: w, p }); }
    const fillLoops = outlines.map((L) => offset(L, o.width * (o.perimeters - 0.15))).filter(Boolean);
    const ang = ((k % 2 ? 45 : -45) * Math.PI) / 180;
    const solidHere = k < o.bottomLayers;
    const nearTop = (p) => top.at(p[0], p[1]) - zt < o.topLayers * o.layer - 1e-6;
    const fill = [];
    if (fillLoops.length) {
      for (const s of hatch(fillLoops, ang, o.width))
        for (const piece of solidHere ? [s] : splitBy(s, nearTop)) fill.push({ seg: piece, solid: true });
      if (!solidHere)
        for (const s of hatch(fillLoops, ang, o.width / o.infill, 0.5))
          for (const piece of splitBy(s, (p) => !nearTop(p))) fill.push({ seg: piece, solid: false });
    }
    layers.push({ k, z: zt, outlines, walls, fill });
  }
  return { layers, opts: o, bounds: { lo, hi } };
}

// ---------------------------------------------------------------- G-code
export function printGcode(sliced, { preamble = [], postamble = [] } = {}) {
  const o = sliced.opts;
  const [ox, oy] = o.place;
  const eArea = (Math.PI * o.filament * o.filament) / 4;
  const ePerMm = (o.layer * o.width) / eArea;
  const g = [...preamble];
  const f3 = (v) => v.toFixed(3);
  let pos = null, retracted = false, length = 0, time = 0, filament = 0;
  const travel = (p, z) => {
    const d = pos ? Math.hypot(p[0] - pos[0], p[1] - pos[1]) : Infinity;
    if (d < 1e-3) return;
    if (d > o.retractMin && !retracted) { g.push(`G1 E-${o.retract} F1800`); retracted = true; }
    g.push(`G0 X${f3(p[0] + ox)} Y${f3(p[1] + oy)} Z${f3(z)}`);
    time += (Number.isFinite(d) ? d : 20) / (o.feed.travel / 60);
    pos = p;
  };
  const extrude = (p, F) => {
    if (retracted) { g.push(`G1 E${o.retract} F1800`); retracted = false; }
    const d = Math.hypot(p[0] - pos[0], p[1] - pos[1]);
    if (d < 1e-3) return;
    g.push(`G1 X${f3(p[0] + ox)} Y${f3(p[1] + oy)} E${(d * ePerMm).toFixed(4)} F${F}`);
    length += d; time += d / (F / 60); filament += d * ePerMm;
    pos = p;
  };
  for (const L of sliced.layers) {
    g.push(`; layer ${L.k + 1}/${sliced.layers.length}, z=${f3(L.z)}`);
    const first = L.k === 0;
    // Walls: inner first, outer last (the outer bead sets the surface).
    const walls = [...L.walls].sort((a, b) => b.p - a.p);
    for (const { loop, p } of walls) {
      const F = first ? o.feed.first : o.feed.perimeter * (p === 0 ? 0.8 : 1);
      // Start the loop at the vertex nearest the nozzle.
      let s = 0, best = Infinity;
      if (pos) loop.forEach((q, i) => { const d = (q[0] - pos[0]) ** 2 + (q[1] - pos[1]) ** 2; if (d < best) { best = d; s = i; } });
      travel(loop[s], L.z);
      for (let i = 1; i <= loop.length; i++) extrude(loop[(s + i) % loop.length], F);
    }
    // Fill: greedy nearest-segment ordering.
    const left = L.fill.slice();
    while (left.length) {
      let bi = 0, bd = Infinity, flip = false;
      left.forEach(({ seg }, i) => {
        for (const [e, fl] of [[0, false], [1, true]]) {
          const d = pos ? (seg[e][0] - pos[0]) ** 2 + (seg[e][1] - pos[1]) ** 2 : 0;
          if (d < bd) { bd = d; bi = i; flip = fl; }
        }
      });
      const { seg, solid } = left.splice(bi, 1)[0];
      const [a, b] = flip ? [seg[1], seg[0]] : seg;
      travel(a, L.z);
      extrude(b, first ? o.feed.first : solid ? o.feed.perimeter : o.feed.infill);
    }
  }
  if (!retracted) g.push(`G1 E-${o.retract} F1800`);
  g.push(...postamble);
  return { lines: g, stats: { layers: sliced.layers.length, extrudedPath_mm: length, filament_mm: filament, time_s: time } };
}
