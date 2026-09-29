// Wires as smooth tubes. Every wire (and the USB cable) is a centripetal
// Catmull-Rom spline through its terminals and route waypoints, swept with a
// small circular section sized from its gauge (slightly exaggerated so thin
// signal wires stay visible). Wires that share waypoints are laid side by
// side in a hexagonal bundle: each wire's offset at a waypoint is fixed in
// that waypoint's body frame, so the bundle bends with the arm.
//
// Cost: all wires live in one merged BufferGeometry (one draw call). The
// vertex layout is fixed at build time; per frame only wires with a point on
// a body that moved are re-swept, in place, and only that part of the buffer
// is uploaded.

import * as THREE from 'three';
import { awgDiameter } from '../../src/core/units.js';

const RADIAL = 6;
const VISUAL_SCALE = 1.35;   // thickness exaggeration
const COS = [], SIN = [];
for (let j = 0; j < RADIAL; j++) { COS.push(Math.cos((j / RADIAL) * 2 * Math.PI)); SIN.push(Math.sin((j / RADIAL) * 2 * Math.PI)); }

// Hexagonal lattice offsets, nearest first.
const HEX = (() => {
  const pts = [];
  for (let a = -8; a <= 8; a++) for (let b = -8; b <= 8; b++) {
    const x = a + b / 2, y = (b * Math.sqrt(3)) / 2;
    pts.push([x, y, Math.hypot(x, y), Math.atan2(y, x)]);
  }
  pts.sort((p, q) => p[2] - q[2] || p[3] - q[3]);
  return pts.map((p) => [p[0], p[1]]);
})();

export function wireRadius(c) {
  if (c.kind === 'usb-cable') return 0.0022;
  const d = awgDiameter(c.awg ?? 22) + 2 * (c.insulation ?? 0.35e-3);
  return (d / 2) * VISUAL_SCALE;
}

// Body-frame location of a port (its component is rigidly mounted on a body).
function portAnchor(port) {
  const mnt = port.owner.mount;
  const R = mnt.R ?? [1, 0, 0, 0, 1, 0, 0, 0, 1], p = mnt.p ?? [0, 0, 0], a = port.at ?? [0, 0, 0];
  return {
    body: mnt.body,
    p: [R[0] * a[0] + R[1] * a[1] + R[2] * a[2] + p[0], R[3] * a[0] + R[4] * a[1] + R[5] * a[2] + p[1], R[6] * a[0] + R[7] * a[1] + R[8] * a[2] + p[2]],
  };
}
function apply(T, p, out, k) {
  const R = T.R;
  out[k] = R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + T.p[0];
  out[k + 1] = R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + T.p[1];
  out[k + 2] = R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + T.p[2];
}

// Centripetal Catmull-Rom between P1 and P2 (Barry-Goldman), t in [0,1].
function crPoint(P, i0, i1, i2, i3, t, out, k) {
  const d = (a, b) => Math.max(1e-6, Math.sqrt(Math.hypot(P[b] - P[a], P[b + 1] - P[a + 1], P[b + 2] - P[a + 2])));
  const t0 = 0, t1 = t0 + d(i0, i1), t2 = t1 + d(i1, i2), t3 = t2 + d(i2, i3);
  const u = t1 + (t2 - t1) * t;
  for (let c = 0; c < 3; c++) {
    const p0 = P[i0 + c], p1 = P[i1 + c], p2 = P[i2 + c], p3 = P[i3 + c];
    const a1 = ((t1 - u) * p0 + (u - t0) * p1) / (t1 - t0);
    const a2 = ((t2 - u) * p1 + (u - t1) * p2) / (t2 - t1);
    const a3 = ((t3 - u) * p2 + (u - t2) * p3) / (t3 - t2);
    const b1 = ((t2 - u) * a1 + (u - t0) * a2) / (t2 - t0);
    const b2 = ((t3 - u) * a2 + (u - t1) * a3) / (t3 - t1);
    out[k + c] = ((t2 - u) * b1 + (u - t1) * b2) / (t2 - t1);
  }
}

export class CableView {
  constructor(m, conns, track) {
    this.m = m;
    this.track = track;
    this.colorByCurrent = false;
    const staticBodies = new Set([m.A.bodies.get('world'), m.A.bodies.get('base')]);

    // ---- control points per wire: [{ body, p (body frame, bundle offset included) }]
    const wires = conns.map((c, idx) => ({ c, idx, r: wireRadius(c), anchors: null }));
    const through = new Map(); // waypoint object -> [wire]
    for (const w of wires) for (const wp of w.c.route ?? []) {
      if (!through.has(wp)) through.set(wp, []);
      through.get(wp).push(w);
    }
    // Rank inside bundles: wires that run furthest take the inner slots, so
    // the ones that peel off early leave from the outside.
    for (const list of through.values()) list.sort((a, b) => (b.c.route.length - a.c.route.length) || (a.idx - b.idx));
    for (const w of wires) {
      const c = w.c;
      const A = portAnchor(c.a), B = portAnchor(c.b);
      const pts = [A];
      for (const wp of c.route ?? []) pts.push({ body: wp.body, p: wp.p, wp });
      pts.push(B);
      if (!c.route?.length) this.dress(pts, w.r);
      w.anchors = pts;
    }
    // Bundle offsets in each waypoint's body frame.
    const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
    const worldOf = (a) => { const o = [0, 0, 0]; apply(a.body.T, a.p, o, 0); return v3(o); };
    for (const [wp, list] of through) {
      const lead = list[0];
      const k = lead.anchors.findIndex((a) => a.wp === wp);
      const prev = worldOf(lead.anchors[k - 1]), next = worldOf(lead.anchors[k + 1]);
      const t = next.sub(prev).normalize();
      let u = new THREE.Vector3().crossVectors(t, new THREE.Vector3(0, 0, 1));
      if (u.lengthSq() < 1e-6) u.crossVectors(t, new THREE.Vector3(1, 0, 0));
      u.normalize();
      const v = new THREE.Vector3().crossVectors(t, u).normalize();
      const R = wp.body.T.R;
      const toLocal = (x) => [R[0] * x.x + R[3] * x.y + R[6] * x.z, R[1] * x.x + R[4] * x.y + R[7] * x.z, R[2] * x.x + R[5] * x.y + R[8] * x.z];
      const ul = toLocal(u), vl = toLocal(v);
      const spacing = 2.05 * Math.max(...list.map((w) => w.r));
      list.forEach((w, slot) => {
        const [ox, oy] = HEX[Math.min(slot, HEX.length - 1)];
        const a = w.anchors.find((x) => x.wp === wp);
        a.p = [0, 1, 2].map((i) => wp.p[i] + spacing * (ox * ul[i] + oy * vl[i]));
      });
    }

    // ---- sampling layout (fixed): samples per span from the build pose.
    let nVert = 0, nIdx = 0, nCtrl = 0;
    for (const w of wires) {
      const P = w.anchors.map(worldOf);
      w.spanSamples = [];
      for (let s = 0; s < P.length - 1; s++) {
        const L = P[s].distanceTo(P[s + 1]);
        w.spanSamples.push(Math.max(2, Math.min(14, Math.ceil(L / 0.007))));
      }
      w.nSamples = w.spanSamples.reduce((a, b) => a + b, 0) + 1;
      w.vStart = nVert;
      w.iStart = nIdx;
      w.ctrlStart = nCtrl;
      nVert += w.nSamples * RADIAL;
      nIdx += (w.nSamples - 1) * RADIAL * 6;
      nCtrl += w.anchors.length * 3;
      w.bodies = [...new Set(w.anchors.map((a) => a.body))];
      w.moving = w.bodies.some((b) => !staticBodies.has(b));
      w.centers = new Float32Array(w.nSamples * 3);
      w.lastRgb = [-1, -1, -1];
    }
    // Moving wires last, so the per-frame upload is one contiguous range.
    wires.sort((a, b) => (a.moving - b.moving) || (a.idx - b.idx));
    nVert = 0; nIdx = 0;
    for (const w of wires) { w.vStart = nVert; w.iStart = nIdx; nVert += w.nSamples * RADIAL; nIdx += (w.nSamples - 1) * RADIAL * 6; }
    this.wires = wires;
    this.byId = new Map(wires.map((w) => [w.c.id, w]));
    this.ctrl = new Float64Array(Math.max(...wires.map((w) => w.anchors.length)) * 3 + 6);

    const pos = new Float32Array(nVert * 3), nor = new Float32Array(nVert * 3), col = new Float32Array(nVert * 3);
    const index = new Uint32Array(nIdx);
    for (const w of wires) {
      let q = w.iStart;
      for (let i = 0; i < w.nSamples - 1; i++) for (let j = 0; j < RADIAL; j++) {
        const a = w.vStart + i * RADIAL + j, b = w.vStart + i * RADIAL + ((j + 1) % RADIAL);
        const c = a + RADIAL, d = b + RADIAL;
        index[q++] = a; index[q++] = c; index[q++] = b;
        index[q++] = b; index[q++] = c; index[q++] = d;
      }
      w.base = new THREE.Color(w.c.kind === 'usb-cable' ? '#2b2e33' : (w.c.color ?? '#aaaaaa'));
    }
    const g = track(new THREE.BufferGeometry());
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('normal', new THREE.BufferAttribute(nor, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3).setUsage(THREE.DynamicDrawUsage));
    g.setIndex(new THREE.BufferAttribute(index, 1));
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(-0.1, 0, 0.2), 2);
    this.geom = g;
    this.material = track(new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.42, metalness: 0.0 }));
    this.mesh = new THREE.Mesh(g, this.material);
    this.mesh.name = 'cables';
    this.mesh.frustumCulled = false;
    this.mesh.castShadow = true;
    this.mesh.receiveShadow = true;
    this.mesh.matrixAutoUpdate = false;
    this.firstMoving = wires.findIndex((w) => w.moving);
    this.bodyState = new Map();
    for (const w of wires) this.sweep(w);
    this.updateColors(true);
    this.upload(0, nVert);
  }

  // Visual dressing for a wire with no route: leave the terminal, lie on the
  // bench (world) and rise to the other terminal.
  dress(pts, r) {
    const A = pts[0], B = pts[pts.length - 1];
    if (A.body !== B.body || A.body.name !== 'world') return;
    const d = Math.hypot(B.p[0] - A.p[0], B.p[1] - A.p[1]);
    if (d < 0.03) return;
    const z = r + 0.0006, lift = Math.min(0.012, d * 0.2);
    const at = (f, zz) => [A.p[0] + (B.p[0] - A.p[0]) * f, A.p[1] + (B.p[1] - A.p[1]) * f, zz];
    const f = Math.min(0.3, 0.02 / d);
    pts.splice(1, 0,
      { body: A.body, p: at(f, Math.max(z, A.p[2] - lift * 0.3)) },
      { body: A.body, p: at(Math.min(0.45, f * 2.2), z) },
      { body: A.body, p: at(Math.max(0.55, 1 - f * 2.2), z) },
      { body: A.body, p: at(1 - f, Math.max(z, B.p[2] - lift * 0.3)) });
  }

  // Recompute one wire's centreline and tube, in place.
  sweep(w) {
    const n = w.anchors.length, P = this.ctrl;
    // Control points with a reflected phantom point at each end.
    for (let k = 0; k < n; k++) apply(w.anchors[k].body.T, w.anchors[k].p, P, (k + 1) * 3);
    for (let c = 0; c < 3; c++) {
      P[c] = 2 * P[3 + c] - P[6 + c];
      P[(n + 1) * 3 + c] = 2 * P[n * 3 + c] - P[(n - 1) * 3 + c];
    }
    const C = w.centers;
    let q = 0;
    for (let s = 0; s < n - 1; s++) {
      const S = w.spanSamples[s];
      for (let i = 0; i < S; i++, q += 3) crPoint(P, s * 3, (s + 1) * 3, (s + 2) * 3, (s + 3) * 3, i / S, C, q);
    }
    C[q] = P[n * 3]; C[q + 1] = P[n * 3 + 1]; C[q + 2] = P[n * 3 + 2];
    this.tube(C, w.nSamples, w.r, this.geom.attributes.position.array, this.geom.attributes.normal.array, w.vStart);
  }

  // Sweep a circle along centreline C (nS points) into pos/nor from vertex v0.
  tube(C, nS, r, pos, nor, v0) {
    let tx = 0, ty = 0, tz = 1, nx = 1, ny = 0, nz = 0;
    for (let i = 0; i < nS; i++) {
      const a = Math.max(0, i - 1) * 3, b = Math.min(nS - 1, i + 1) * 3;
      let dx = C[b] - C[a], dy = C[b + 1] - C[a + 1], dz = C[b + 2] - C[a + 2];
      const L = Math.hypot(dx, dy, dz);
      if (L > 1e-9) { tx = dx / L; ty = dy / L; tz = dz / L; }
      if (i === 0) {
        // Any normal perpendicular to the first tangent.
        if (Math.abs(tz) < 0.9) { nx = -ty; ny = tx; nz = 0; } else { nx = 0; ny = -tz; nz = ty; }
      }
      // Parallel transport: project the previous normal onto the new plane.
      const dp = nx * tx + ny * ty + nz * tz;
      nx -= dp * tx; ny -= dp * ty; nz -= dp * tz;
      let nl = Math.hypot(nx, ny, nz);
      if (nl < 1e-9) { if (Math.abs(tz) < 0.9) { nx = -ty; ny = tx; nz = 0; } else { nx = 0; ny = -tz; nz = ty; } nl = Math.hypot(nx, ny, nz); }
      nx /= nl; ny /= nl; nz /= nl;
      const bx = ty * nz - tz * ny, by = tz * nx - tx * nz, bz = tx * ny - ty * nx;
      const cx = C[i * 3], cy = C[i * 3 + 1], cz = C[i * 3 + 2];
      let o = (v0 + i * RADIAL) * 3;
      for (let j = 0; j < RADIAL; j++, o += 3) {
        const ex = COS[j] * nx + SIN[j] * bx, ey = COS[j] * ny + SIN[j] * by, ez = COS[j] * nz + SIN[j] * bz;
        nor[o] = ex; nor[o + 1] = ey; nor[o + 2] = ez;
        pos[o] = cx + r * ex; pos[o + 1] = cy + r * ey; pos[o + 2] = cz + r * ez;
      }
    }
  }

  upload(v0, v1) {
    for (const name of ['position', 'normal']) {
      const a = this.geom.attributes[name];
      a.clearUpdateRanges();
      a.addUpdateRange(v0 * 3, (v1 - v0) * 3);
      a.needsUpdate = true;
    }
  }

  // Which bodies moved since the last call.
  movedBodies() {
    const moved = new Set();
    for (const body of this.m.A.bodies.values()) {
      const T = body.T;
      let s = this.bodyState.get(body);
      if (!s) { s = new Float64Array(12).fill(NaN); this.bodyState.set(body, s); }
      const R = T.R, p = T.p;
      let changed = false;
      for (let k = 0; k < 9; k++) if (s[k] !== R[k]) { changed = true; s[k] = R[k]; }
      for (let k = 0; k < 3; k++) if (s[9 + k] !== p[k]) { changed = true; s[9 + k] = p[k]; }
      if (changed) moved.add(body);
    }
    return moved;
  }

  update() {
    const moved = this.movedBodies();
    let lo = Infinity, hi = -1;
    if (moved.size) {
      for (let k = Math.max(0, this.firstMoving); k < this.wires.length; k++) {
        const w = this.wires[k];
        if (!w.moving || !w.bodies.some((b) => moved.has(b))) continue;
        this.sweep(w);
        lo = Math.min(lo, w.vStart); hi = Math.max(hi, w.vStart + w.nSamples * RADIAL);
      }
    }
    if (hi > lo) this.upload(lo, hi);
    this.updateColors(false);
    return hi > lo;
  }

  updateColors(force) {
    const col = this.geom.attributes.color.array;
    const rgb = [0, 0, 0];
    let lo = Infinity, hi = -1;
    for (const w of this.wires) {
      if (this.colorByCurrent && w.c.kind === 'wire') this.heat(Math.abs(w.c.i ?? 0), rgb);
      else { rgb[0] = w.base.r; rgb[1] = w.base.g; rgb[2] = w.base.b; }
      const L = w.lastRgb;
      if (!force && Math.abs(L[0] - rgb[0]) + Math.abs(L[1] - rgb[1]) + Math.abs(L[2] - rgb[2]) < 0.01) continue;
      L[0] = rgb[0]; L[1] = rgb[1]; L[2] = rgb[2];
      const n = w.nSamples * RADIAL;
      for (let v = 0, o = w.vStart * 3; v < n; v++, o += 3) { col[o] = rgb[0]; col[o + 1] = rgb[1]; col[o + 2] = rgb[2]; }
      lo = Math.min(lo, w.vStart); hi = Math.max(hi, w.vStart + n);
    }
    if (hi > lo) {
      const a = this.geom.attributes.color;
      a.clearUpdateRanges();
      a.addUpdateRange(lo * 3, (hi - lo) * 3);
      a.needsUpdate = true;
    }
  }

  // Nearest wire whose centreline passes within `threshold` (+ its radius)
  // of the ray. Returns { id, distance } or null.
  pick(ray, threshold) {
    const a = new THREE.Vector3(), b = new THREE.Vector3(), onRay = new THREE.Vector3();
    let best = null;
    for (const w of this.wires) {
      const C = w.centers, lim = (threshold + w.r) ** 2;
      for (let i = 0; i < w.nSamples - 1; i++) {
        a.fromArray(C, i * 3); b.fromArray(C, i * 3 + 3);
        const d2 = ray.distanceSqToSegment(a, b, onRay);
        if (d2 > lim) continue;
        const dist = onRay.distanceTo(ray.origin);
        if (!best || dist < best.distance) best = { id: w.c.id, distance: dist };
      }
    }
    return best;
  }

  // Tube for highlighting one wire (fatter, drawn on top).
  highlightGeometry(id, geom) {
    const w = this.byId.get(id);
    if (!w) return null;
    const nV = w.nSamples * RADIAL;
    let g = geom;
    if (!g || g.userData.wire !== id) {
      g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nV * 3), 3));
      g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(nV * 3), 3));
      const idx = this.geom.index.array.subarray(w.iStart, w.iStart + (w.nSamples - 1) * RADIAL * 6);
      g.setIndex(new THREE.BufferAttribute(Uint32Array.from(idx, (v) => v - w.vStart), 1));
      g.userData.wire = id;
      g.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 10);
    }
    this.tube(w.centers, w.nSamples, w.r + 0.0009, g.attributes.position.array, g.attributes.normal.array, 0);
    g.attributes.position.needsUpdate = true;
    g.attributes.normal.needsUpdate = true;
    return g;
  }

  // Current magnitude -> heat colour (linear RGB), 0 A dim, >= 2 A bright.
  heat(iAbs, out) {
    const x = Math.min(1, Math.max(0, iAbs / 2));
    const t = Math.sqrt(x);
    const stops = HEAT_STOPS;
    let k = 1;
    while (k < stops.length - 1 && t > stops[k][0]) k++;
    const a = stops[k - 1], b = stops[k];
    const f = (t - a[0]) / (b[0] - a[0] || 1);
    out[0] = a[1] + (b[1] - a[1]) * f; out[1] = a[2] + (b[2] - a[2]) * f; out[2] = a[3] + (b[3] - a[3]) * f;
    return out;
  }
}

const HEAT_STOPS = [
  [0.00, 0.035, 0.04, 0.05],
  [0.15, 0.25, 0.02, 0.01],
  [0.45, 0.85, 0.12, 0.01],
  [0.75, 1.0, 0.45, 0.02],
  [1.00, 1.0, 0.95, 0.55],
];
