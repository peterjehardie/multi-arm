// Procedural models for every kind of component, in the component's own
// frame (the frame its ports are given in). Each builder adds pieces to a
// Builder; the view merges them per material and tags the meshes for picking.
// All dimensions in metres. Proportions follow the real parts (NEMA 17
// steppers, StepStick drivers, 775 spindle, ...); where the simulation's
// port positions and a real part disagree, the model follows the ports so
// every wire lands on a terminal.

import * as THREE from 'three';
import { G, M, Builder, roundRect, chamferSquare, circlePath } from './kit.js';

const PI = Math.PI;

// ------------------------------------------------------------------ shapes
function gearShape(rRoot, rTip, teeth) {
  const sh = new THREE.Shape();
  const n = teeth * 4;
  for (let k = 0; k <= n; k++) {
    const a = (k / n) * 2 * PI;
    const phase = k % 4;
    const r = phase === 1 || phase === 2 ? rTip : rRoot;
    const x = r * Math.cos(a), y = r * Math.sin(a);
    if (k === 0) sh.moveTo(x, y); else sh.lineTo(x, y);
  }
  return sh;
}

// Convex hull (monotone chain) of 2D points.
function hull(pts) {
  pts = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of pts) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = pts.length - 1; i >= 0; i--) { const p = pts[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  up.pop(); lo.pop();
  return lo.concat(up);
}
function twoCircleOutline(c1, r1, c2, r2, n = 48) {
  const pts = [];
  for (let k = 0; k < n; k++) {
    const a = (k / n) * 2 * PI;
    pts.push([c1[0] + r1 * Math.cos(a), c1[1] + r1 * Math.sin(a)]);
    pts.push([c2[0] + r2 * Math.cos(a), c2[1] + r2 * Math.sin(a)]);
  }
  return hull(pts);
}
// Timing belt around two pulleys (in the XY plane), height h from z0.
export function beltGeometry(c1, r1, c2, r2, t, h) {
  const outer = twoCircleOutline(c1, r1 + t, c2, r2 + t);
  const inner = twoCircleOutline(c1, r1, c2, r2);
  const sh = new THREE.Shape(outer.map(([x, y]) => new THREE.Vector2(x, y)));
  sh.holes.push(new THREE.Path(inner.map(([x, y]) => new THREE.Vector2(x, y)).reverse()));
  return G.extrude(sh, h, 0, 4);
}

// Heat-sink fins stacked along Z (a lathe profile).
function finStack(rFin, rCore, z0, z1, fins) {
  const pts = [[0, z0]];
  const pitch = (z1 - z0) / fins;
  for (let k = 0; k < fins; k++) {
    const a = z0 + k * pitch;
    pts.push([rCore, a], [rFin, a + pitch * 0.12], [rFin, a + pitch * 0.5], [rCore, a + pitch * 0.62]);
  }
  pts.push([rCore, z1], [0, z1]);
  return G.lathe(pts, 32);
}

// Fan: square frame with a round opening, a hub and blades (returns pieces).
export function fanParts(size, depth, blades = 7) {
  const frameShape = roundRect(size, size, size * 0.12);
  frameShape.holes.push(circlePath(size * 0.46));
  const frame = G.extrude(frameShape, depth, 0, 24).translate(0, 0, -depth / 2);
  const hub = G.cylZ(size * 0.17, depth * 0.8, 24);
  const bladeGeoms = [];
  for (let k = 0; k < blades; k++) {
    const b = G.box(size * 0.29, size * 0.12, depth * 0.12);
    b.translate(size * 0.29, 0, 0);
    b.rotateX(0.5);
    b.rotateZ((k / blades) * 2 * PI);
    bladeGeoms.push(b);
  }
  return { frame, hub, bladeGeoms };
}

// ------------------------------------------------------------------ motors
// NEMA-style stepper, axis along +Z in the motor frame (shaft at +z).
// `A` maps the motor frame into the component frame.
export function stepperModel(b, mats, size, A, { round = false, shaftLen = 0.02 } = {}) {
  const s = size[0], L = size[2];
  const cap = Math.min(0.008, L * 0.22);
  const tr = (g, z) => g.applyMatrix4(new THREE.Matrix4().makeTranslation(0, 0, z)).applyMatrix4(A);
  if (round) {
    b.add(tr(G.cylZ(s / 2, L - 2 * cap, 40), 0), mats.get('motor'));
    b.add(tr(G.cylZ(s / 2 - 0.0005, cap, 40), L / 2 - cap / 2), mats.get('alu'));
    b.add(tr(G.cylZ(s / 2 - 0.0005, cap, 40), -L / 2 + cap / 2), mats.get('alu'));
  } else {
    const body = G.extrude(chamferSquare(s, 0.0042), L - 2 * cap, 0.0004);
    b.add(tr(body, -L / 2 + cap), mats.get('motor'));
    for (const z of [L / 2 - cap, -L / 2]) b.add(tr(G.extrude(chamferSquare(s - 0.0006, 0.005), cap, 0.0006), z), mats.get('alu'));
    for (const [x, y] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const g = G.cylZ(0.0027, 0.0012, 12).translate(x * 0.0155, y * 0.0155, 0);
      b.add(tr(g, L / 2 + 0.0004), mats.get('steelDark'));
    }
  }
  b.add(tr(G.cylZ(0.011, 0.002, 32), L / 2 + 0.001), mats.get('alu'));
  b.add(tr(G.cylZ(0.0025, shaftLen, 16), L / 2 + shaftLen / 2), mats.get('steel'));
}

// Small white JST connector at a group of terminal positions.
function connectorAt(b, mats, ports) {
  if (!ports.length) return;
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of ports) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p.at[k]); hi[k] = Math.max(hi[k], p.at[k]); }
  const c = [0, 1, 2].map((k) => (lo[k] + hi[k]) / 2);
  const sz = [0, 1, 2].map((k) => Math.max(0.004, hi[k] - lo[k] + 0.003));
  b.add(G.rbox(sz[0], sz[1], sz[2], 0.0006), mats.get('whitePlastic'), M(c[0], c[1], c[2]));
}

// Planetary gearbox between a motor face and a joint (axis along the
// component-frame direction `axis`, from `from` to `to` along it).
export function gearboxModel(b, mats, r, center, axisKey, from, to) {
  const len = Math.abs(to - from), mid = (from + to) / 2;
  const mk = (rr, h, off) => {
    const g = axisKey === 'y' ? G.cylY(rr, h, 40) : G.cylZ(rr, h, 40);
    const p = [...center];
    p[axisKey === 'y' ? 1 : 2] = off;
    return g.translate(p[0], p[1], p[2]);
  };
  b.add(mk(r, len, mid), mats.get('aluSatin'));
  const dir = Math.sign(to - from);
  b.add(mk(r + 0.0012, 0.0025, from + dir * 0.0014), mats.get('steelDark'));
  b.add(mk(r * 0.8, 0.002, to + dir * 0.001), mats.get('steel'));
}

// ------------------------------------------------------------------ boards
function pcb(b, mat, sx, sy, z, t = 0.0016) {
  b.add(G.rbox(sx, sy, t, 0.0012), mat, M(0, 0, z + t / 2));
}
function standoffs(b, mats, sx, sy, h) {
  for (const [x, y] of [[1, 1], [1, -1], [-1, 1], [-1, -1]])
    b.add(G.hexZ(0.0022, h).translate(x * (sx / 2 - 0.003), y * (sy / 2 - 0.003), h / 2), mats.get('brass'));
}
function screwTerminal(b, mats, x, y, z, n, pitch, alongY = true, mat = 'greenTerminal') {
  const w = n * pitch;
  b.add(G.rbox(alongY ? 0.0075 : w, alongY ? w : 0.0075, 0.0085, 0.0006), mats.get(mat), M(x, y, z + 0.00425));
  for (let k = 0; k < n; k++) {
    const o = -w / 2 + pitch * (k + 0.5);
    b.add(G.cylZ(0.0014, 0.0008, 12), mats.get('steel'), M(x + (alongY ? 0 : o), y + (alongY ? o : 0), z + 0.0086));
  }
}
function pinHeader(b, mats, pts, z0, len = 0.0062) {
  for (const p of pts) b.add(G.box(0.00064, 0.00064, len), mats.get('gold'), M(p[0], p[1], z0 + len / 2));
}

export const MODELS = {
  controller(b, c, { mats }) {
    const [sx, sy] = c.size;
    standoffs(b, mats, sx, sy, 0.003);
    pcb(b, mats.get('pcbBlack'), sx, sy, 0.003);
    const top = 0.0046;
    b.add(G.rbox(0.013, 0.013, 0.0012, 0.0003), mats.get('chip'), M(0.004, 0, top + 0.0006));
    b.add(G.rbox(0.006, 0.006, 0.001, 0.0002), mats.get('chip'), M(-0.014, 0.008, top + 0.0005));
    b.add(G.rbox(0.004, 0.0022, 0.0012, 0.0004), mats.get('steel'), M(-0.012, -0.007, top + 0.0006));
    b.add(G.rbox(0.008, 0.0095, 0.0032, 0.0012), mats.get('steel'), M(-0.0405, 0, top + 0.0016));
    b.add(G.box(0.0016, 0.0009, 0.0005), mats.get('ledGreen'), M(-0.03, 0.012, top + 0.00025));
    b.add(G.box(0.0035, 0.0035, 0.0012), mats.get('blackPlastic'), M(-0.028, -0.012, top + 0.0006));
    for (let k = 0; k < 10; k++) b.add(G.box(0.0012, 0.0006, 0.0005), mats.get(k % 3 ? 'chip' : 'copper'), M(0.018 + (k % 5) * 0.003, -0.006 - (k / 5 | 0) * 0.004, top + 0.00025));
    // Headers along the pin rows the wiring uses.
    const pins = [];
    for (const p of c.ports.values()) if (p.at[2] > 0.007 && p.name !== 'USB') pins.push(p.at);
    const rows = [[-0.0415, 0.03, -0.026], [-0.0415, 0.03, 0.026]];
    for (const [x0, x1, y] of rows) b.add(G.box(x1 - x0 + 0.004, 0.0025, 0.0025), mats.get('blackPlastic'), M((x0 + x1) / 2, y, top + 0.00125));
    b.add(G.box(0.0025, 0.052, 0.0025), mats.get('blackPlastic'), M(0.042, 0, top + 0.00125));
    pinHeader(b, mats, pins, top + 0.0025, 0.0016);
  },

  driver(b, c, { mats }) {
    const base = -0.01; // carrier board on the bench
    pcb(b, mats.get('pcbGreen'), 0.034, 0.03, base, 0.0016);
    for (const x of [-0.0115, 0.0115]) b.add(G.box(0.0026, 0.021, 0.0082), mats.get('blackPlastic'), M(x, 0, base + 0.0016 + 0.0041));
    const top = base + 0.0016 + 0.0082;
    pcb(b, mats.get('pcbPurple'), 0.0155, 0.0205, top, 0.0014);
    const z = top + 0.0014;
    b.add(G.box(0.009, 0.009, 0.0012), mats.get('aluSatin'), M(0.0005, 0.001, z + 0.0006));
    for (let k = 0; k < 5; k++) b.add(G.box(0.0011, 0.009, 0.0065), mats.get('alu'), M(0.0005 - 0.0036 + k * 0.0018, 0.001, z + 0.0012 + 0.00325));
    b.add(G.cylZ(0.0012, 0.0012, 12), mats.get('steel'), M(-0.004, -0.0078, z + 0.0006));
    const pins = [...c.ports.values()].map((p) => p.at);
    pinHeader(b, mats, pins, base + 0.0016 + 0.0082 - 0.0001, 0.0024);
  },

  mosfet(b, c, { mats, unique }) {
    const [sx, sy] = c.size;
    standoffs(b, mats, sx, sy, 0.002);
    pcb(b, mats.get('pcbBlue'), sx, sy, 0.002);
    const z = 0.0036;
    screwTerminal(b, mats, -0.0135, 0, z, 2, 0.006);
    screwTerminal(b, mats, 0.0135, 0, z, 2, 0.006);
    b.add(G.rbox(0.0065, 0.006, 0.0014, 0.0003), mats.get('chip'), M(-0.002, 0.003, z + 0.0007));
    b.add(G.rbox(0.0065, 0.006, 0.0014, 0.0003), mats.get('chip'), M(0.005, 0.003, z + 0.0007));
    b.add(G.box(0.006, 0.0025, 0.0055), mats.get('blackPlastic'), M(0.002, -0.0075, z + 0.00275));
    const led = unique('ledRed', { emissiveIntensity: 0 });
    b.add(G.box(0.0016, 0.0009, 0.0006), led, M(-0.006, -0.004, z + 0.0003));
    return { led };
  },

  buck(b, c, { mats }) {
    const [sx, sy] = c.size;
    standoffs(b, mats, sx, sy, 0.002);
    pcb(b, mats.get('pcbBlue'), sx, sy, 0.002);
    const z = 0.0036;
    screwTerminal(b, mats, -0.0185, 0, z, 2, 0.008);
    screwTerminal(b, mats, 0.0185, 0, z, 2, 0.008);
    b.add(G.torusZ(0.0042, 0.0022, 24, 10), mats.get('copper'), M(-0.004, 0.003, z + 0.0024));
    b.add(G.cylZ(0.0018, 0.004, 12), mats.get('blackPlastic'), M(-0.004, 0.003, z + 0.002));
    for (const [x, y] of [[0.007, 0.006], [0.007, -0.005]]) {
      b.add(G.cylZ(0.0033, 0.008, 20), mats.get('blackPlastic'), M(x, y, z + 0.004));
      b.add(G.cylZ(0.0031, 0.0004, 20), mats.get('steel'), M(x, y, z + 0.0082));
    }
    b.add(G.box(0.0095, 0.0045, 0.004), mats.get('bluePlastic'), M(-0.004, -0.0075, z + 0.002));
    b.add(G.cylZ(0.0009, 0.0006, 10), mats.get('brass'), M(-0.008, -0.0075, z + 0.0042));
    b.add(G.rbox(0.005, 0.004, 0.0012, 0.0003), mats.get('chip'), M(0.001, -0.002, z + 0.0006));
  },

  terminal(b, c, { mats }) {
    const ports = [...c.ports.values()];
    const xs = ports.map((p) => p.at[0]);
    const x0 = Math.min(...xs) - 0.004, x1 = Math.max(...xs) + 0.004;
    b.add(G.box(x1 - x0 + 0.008, 0.035, 0.001), mats.get('steel'), M((x0 + x1) / 2, 0, 0.0005));
    for (const p of ports) {
      const bar = p.name.replace(/\d+$/, '');
      const top = bar === 'P' || bar === 'V' ? 'redPlastic' : bar === 'N' || bar === 'G' ? 'bluePlastic' : 'whitePlastic';
      b.add(G.rbox(0.0056, 0.026, 0.012, 0.0008), mats.get('greyPlastic'), M(p.at[0], 0, 0.007));
      b.add(G.box(0.0056, 0.008, 0.0016), mats.get(top), M(p.at[0], -0.008, 0.0134));
      b.add(G.cylZ(0.0019, 0.001, 14), mats.get('steel'), M(p.at[0], 0.003, 0.0132));
      b.add(G.box(0.0038, 0.0005, 0.0003), mats.get('slot'), M(p.at[0], 0.003, 0.0138));
    }
  },

  psu(b, c, { mats }) {
    const [sx, sy, sz] = c.size;
    const caseX1 = sx / 2 - 0.02;
    const cx = (-sx / 2 + caseX1) / 2, cw = caseX1 + sx / 2;
    b.add(G.rbox(cw, sy, sz, 0.002), mats.get('psuSheet'), M(cx, 0, sz / 2));
    // Top cover lip over the terminal strip.
    b.add(G.box(0.02, sy, 0.004), mats.get('psuSheet'), M(caseX1 + 0.01, 0, sz - 0.002));
    b.add(G.box(0.019, sy - 0.004, 0.02), mats.get('blackPlastic'), M(caseX1 + 0.0095, 0, 0.01));
    const screws = [0.05, 0.04, 0.03, 0.02, 0.005, -0.015, -0.035];
    screws.forEach((y, k) => {
      b.add(G.cylZ(0.0024, 0.0016, 14), mats.get('steel'), M(0.1, y, 0.0208));
      b.add(G.box(0.0044, 0.0006, 0.0004), mats.get('slot'), M(0.1, y, 0.0217));
      if (k < screws.length - 1) b.add(G.box(0.016, 0.0012, 0.004), mats.get('blackPlastic'), M(caseX1 + 0.0095, (y + screws[k + 1]) / 2, 0.022));
    });
    b.add(G.cylZ(0.0012, 0.0015, 10), mats.get('ledGreen'), M(caseX1 - 0.004, -0.045, sz + 0.0002));
    // Fan grille on top.
    const fx = -0.03, R = 0.029;
    b.add(G.cylZ(R + 0.001, 0.0004, 48), mats.get('slot'), M(fx, 0, sz + 0.0001));
    for (let k = 1; k <= 4; k++) b.add(G.torusZ((R * k) / 4.3, 0.0007, 48, 6), mats.get('steelDark'), M(fx, 0, sz + 0.0006));
    b.add(G.box(2 * R, 0.0014, 0.0008), mats.get('steelDark'), M(fx, 0, sz + 0.0006));
    b.add(G.box(0.0014, 2 * R, 0.0008), mats.get('steelDark'), M(fx, 0, sz + 0.0006));
    b.add(G.cylZ(0.009, 0.001, 24), mats.get('steelDark'), M(fx, 0, sz + 0.0007));
    // Vent slots on the top and the long sides.
    for (let i = 0; i < 6; i++) for (let j = 0; j < 7; j++)
      b.add(G.box(0.0022, 0.012, 0.0004), mats.get('slot'), M(0.025 + i * 0.008, -0.042 + j * 0.014, sz + 0.0001));
    for (const y of [sy / 2 + 0.0001, -sy / 2 - 0.0001]) for (let i = 0; i < 14; i++)
      b.add(G.box(0.0022, 0.0004, 0.02), mats.get('slot'), M(-0.095 + i * 0.012, y, sz * 0.55));
    // Rating label.
    b.add(G.box(0.07, 0.0004, 0.028), mats.get('whitePlastic'), M(-0.02, -sy / 2 - 0.0003, 0.022));
    b.add(G.box(0.07, 0.00045, 0.006), mats.get('blackPlastic'), M(-0.02, -sy / 2 - 0.0004, 0.031));
  },

  host(b, c, { mats, screenMat }) {
    const [sx, sy] = c.size;
    b.add(G.rbox(sx, sy, 0.014, 0.004), mats.get('laptop'), M(0, 0, 0.007));
    b.add(G.rbox(sx - 0.04, sy * 0.46, 0.0006, 0.002), mats.get('keyboard'), M(0, 0.022, 0.0141));
    b.add(G.rbox(0.09, 0.055, 0.0004, 0.004), mats.get('aluSatin'), M(0, -0.06, 0.0141));
    // Lid, hinged at the +y edge, opened about 110 degrees.
    const lid = M(0, sy / 2 - 0.003, 0.014, -1.92, 0, 0);
    const lidShell = G.rbox(sx, sy, 0.006, 0.003).translate(0, -sy / 2, 0.003);
    b.add(lidShell.applyMatrix4(lid), mats.get('laptop'));
    const scr = new THREE.PlaneGeometry(sx - 0.02, sy - 0.03).rotateX(PI).translate(0, -sy / 2 - 0.004, -0.0002);
    b.add(scr.applyMatrix4(lid), screenMat);
    const port = c.ports.get('USB');
    if (port) b.add(G.box(0.002, 0.009, 0.004), mats.get('slot'), M(port.at[0] + 0.0005, port.at[1], port.at[2]));
  },

  'limit-switch'(b, c, { mats }) {
    b.add(G.rbox(0.02, 0.0064, 0.0102, 0.0008), mats.get('blackPlastic'), M(0, 0, 0));
    b.add(G.box(0.019, 0.0004, 0.0035), mats.get('steel'), M(0.001, 0.0045, 0.0), );
    b.add(G.box(0.019, 0.0028, 0.0004), mats.get('steel'), M(0.001, 0.0045 - 0.0012, 0.0035 * 0));
    b.add(G.cylZ(0.0019, 0.003, 14), mats.get('blackPlastic'), M(0.0105, 0.0062, 0));
    for (const x of [-0.0065, 0, 0.0065]) b.add(G.box(0.0032, 0.0045, 0.0005), mats.get('brass'), M(x, -0.0052, 0));
    b.add(G.box(0.004, 0.0004, 0.0104), mats.get('redPlastic'), M(-0.004, 0.0033, 0));
  },

  servo(b, c, { mats }) {
    b.add(G.rbox(0.0225, 0.0118, 0.0225, 0.0012), mats.get('bluePlastic'), M(0, 0, -0.003));
    b.add(G.rbox(0.032, 0.0118, 0.0022, 0.0006), mats.get('bluePlastic'), M(0, 0, 0.0045));
    b.add(G.cylZ(0.0058, 0.004, 24), mats.get('bluePlastic'), M(0.0055, 0, 0.0095));
    b.add(G.cylZ(0.0024, 0.003, 16), mats.get('whitePlastic'), M(0.0055, 0, 0.0125));
    b.add(G.rbox(0.019, 0.0048, 0.0016, 0.0008), mats.get('whitePlastic'), M(0.0055, 0, 0.0138));
  },

  'slip-ring'(b, c, { mats }) {
    const [sx, , sz] = c.size;
    const r = sx / 2;
    b.add(G.cylZ(r, sz * 0.8, 32), mats.get('blackPlastic'), M(0, 0, -sz * 0.05));
    for (let i = 1; i <= 6; i++) b.add(G.cylZ(r + 0.0004, 0.0012, 32), mats.get('brass'), M(0, 0, -0.012 + i * 0.003));
    b.add(G.cylZ(r + 0.006, 0.002, 32), mats.get('alu'), M(0, 0, sz * 0.36));
    b.add(G.cylZ(0.004, 0.022, 16), mats.get('steel'), M(0, 0, sz * 0.36 + 0.011));
  },

  'changer-master'(b, c, { mats }) {
    const [sx] = c.size;
    b.add(G.cylX(0.025, sx, 48), mats.get('aluDark'), M(0, 0, 0));
    b.add(G.cylX(0.019, 0.0015, 48), mats.get('alu'), M(sx / 2 + 0.0002, 0, 0));
    b.add(G.cylX(0.004, 0.004, 16), mats.get('steel'), M(sx / 2 + 0.002, 0, 0));
    b.add(G.rbox(sx, 0.061, 0.009, 0.0015), mats.get('blackPlastic'), M(0, -0.0125, 0.02));
    for (const p of c.ports.values()) if (p.domain === 'elec') b.add(G.cylX(0.0009, 0.0024, 10), mats.get('gold'), M(p.at[0] + 0.0006, p.at[1], p.at[2]));
    b.add(G.box(0.004, 0.006, 0.02), mats.get('steel'), M(0, 0.02, -0.018));
  },

  'tool-plate'(b, c, { mats, toolColor }) {
    const [sx] = c.size;
    b.add(G.cylX(0.025, sx, 48), mats.get('alu'), M(0, 0, 0));
    b.add(G.cylX(0.0255, 0.002, 48), mats.get(toolColor ?? 'printed'), M(sx / 2 - 0.001, 0, 0));
    b.add(G.rbox(sx, 0.061, 0.009, 0.0015), mats.get('blackPlastic'), M(0, -0.0125, 0.02));
    for (const p of c.ports.values()) if (p.domain === 'elec') b.add(G.cylX(0.0012, 0.0005, 12), mats.get('gold'), M(p.at[0] - 0.0002, p.at[1], p.at[2]));
    b.add(G.cylX(0.0045, 0.002, 16), mats.get('steelDark'), M(-sx / 2 - 0.0005, 0, 0));
  },

  extruder(b, c, { mats }) {
    // Component origin sits at tool (0.022, 0, 0.022); the hot end's heat
    // sink runs down the tool axis (tool z = 0) below the plate.
    const oz = -0.022, ox = -0.022;
    b.add(G.rbox(0.004, 0.044, 0.052, 0.0015), mats.get('printed'), M(ox + 0.010, 0, oz + 0.012));
    b.add(G.rbox(0.026, 0.034, 0.024, 0.003), mats.get('printed'), M(-0.001, 0, 0.004));
    b.add(G.rbox(0.014, 0.006, 0.01, 0.002), mats.get('printedDark'), M(-0.004, 0.02, 0.006));
    b.add(G.cylX(0.0022, 0.012, 12), mats.get('whitePlastic'), M(-0.019, 0, 0.004));
    // Heat sink and heat break along the tool axis.
    const hs = finStack(0.011, 0.0055, 0, 0.022, 10).rotateY(PI / 2);
    b.add(hs, mats.get('alu'), M(ox + 0.014, 0, oz));
    b.add(G.cylX(0.0016, 0.006, 12), mats.get('steel'), M(ox + 0.039, 0, oz));
    // Hot-end fan on the +y side.
    const { frame, hub, bladeGeoms } = fanParts(0.03, 0.008);
    const fanM = M(ox + 0.025, 0.017, oz, -PI / 2, 0, 0);
    b.add(frame.applyMatrix4(fanM), mats.get('blackPlastic'));
    b.add(hub.applyMatrix4(fanM), mats.get('blackPlastic'));
    for (const g of bladeGeoms) b.add(g.applyMatrix4(fanM), mats.get('greyPlastic'));
  },

  'thermal-mass'(b, c, { mats, hotMat }) {
    if (c.round) {
      const r = c.round, h = c.size[2];
      b.add(G.lathe([[0, -h / 2], [r - 0.0015, -h / 2], [r, -h / 2 + 0.0015], [r, h / 2 - 0.0012], [r - 0.0005, h / 2 - 0.0005], [0, h / 2 - 0.0005]], 96), hotMat ?? mats.get('alu'));
      b.add(G.cylZ(r - 0.0015, 0.0005, 96), mats.get('pei'), M(0, 0, h / 2 - 0.00025));
      b.add(G.box(r * 0.25, 0.003, 0.0002), mats.get('printed'), M(r * 0.82, 0, h / 2 + 0.0001));
      for (let k = 0; k < 3; k++) {
        const a = (k / 3) * 2 * PI + PI / 6;
        b.add(G.cylZ(0.0028, 0.0008, 16), mats.get('steel'), M((r - 0.008) * Math.cos(a), (r - 0.008) * Math.sin(a), h / 2 + 0.0001));
      }
      return;
    }
    if (c.id === 'hot_block') {
      const [sx, sy, sz] = c.size;
      b.add(G.rbox(sx, sy, sz, 0.0008), hotMat, M(0, 0, 0));
      // Nozzle: hex then cone, apex along +x.
      b.add(G.hexZ(0.0042, 0.003).rotateY(PI / 2), mats.get('brass'), M(sx / 2 + 0.0015, 0, 0));
      b.add(G.cylX(0.0032, 0.0032, 20, 0.0007), mats.get('brass'), M(sx / 2 + 0.0046, 0, 0));
      b.add(G.box(0.0006, sy * 0.7, 0.002), mats.get('steel'), M(-sx / 2 + 0.003, 0, sz / 2 + 0.0004));
      return;
    }
    b.add(G.rbox(...c.size, 0.001), hotMat ?? mats.get('alu'));
  },

  heater(b, c, { hotMat }) {
    if (c.size[0] > 0.1) {
      b.add(G.cylZ(Math.min(c.size[0], c.size[1]) / 2, c.size[2], 72), hotMat);
      return;
    }
    // Cartridge heater across the block.
    b.add(G.cylY(0.003, 0.024, 20), hotMat);
  },

  thermistor(b, c, { mats }) {
    b.add(G.cylY(0.0014, 0.005, 12), mats.get('glass'));
    b.add(G.sphere(0.0011, 10), mats.get('blackPlastic'), M(0, 0, 0));
  },

  probe(b, c, { mats }) {
    const tip = c.stylus.at[0];
    b.add(G.cylX(0.0105, 0.02, 40), mats.get('aluDark'), M(-0.009, 0, 0));
    b.add(G.cylX(0.0107, 0.0018, 40), mats.get('ledBlue'), M(0.0018, 0, 0));
    b.add(G.cylX(0.0105, 0.012, 40, 0.006), mats.get('steel'), M(0.0087, 0, 0));
    b.add(G.cylX(0.0028, 0.004, 16), mats.get('steel'), M(0.0165, 0, 0));
    b.add(G.cylX(0.001, tip - 0.018 - 0.001, 12), mats.get('steelDark'), M((0.018 + tip - 0.001) / 2, 0, 0));
    b.add(G.sphere(0.001, 16), mats.get('ruby'), M(tip - 0.001, 0, 0));
  },

  'dc-motor'(b, c, { mats, spec }) {
    const bitX = c.bit.at[0];
    const r = spec.tools.spindle.cutterRadius;
    // 775 can (shortened to fit the drawn tool length).
    b.add(G.cylX(0.0212, 0.046, 48), mats.get('steel'), M(-0.012, 0, 0));
    b.add(G.cylX(0.0214, 0.006, 48), mats.get('blackPlastic'), M(-0.035, 0, 0));
    for (let k = 0; k < 8; k++) {
      const a = (k / 8) * 2 * PI;
      b.add(G.box(0.008, 0.004, 0.0015), mats.get('slot'), M(-0.025, 0.0205 * Math.cos(a), 0.0205 * Math.sin(a), a, 0, 0));
    }
    for (const y of [-0.006, 0.006]) b.add(G.box(0.004, 0.0028, 0.0006), mats.get('brass'), M(-0.039, y, 0.012));
    // Clamp.
    b.add(G.rbox(0.014, 0.045, 0.045, 0.004), mats.get('aluSatin'), M(-0.02, 0, 0));
    b.add(G.cylZ(0.0025, 0.047, 12), mats.get('steelDark'), M(-0.02, 0.017, 0));
    // Front boss, ER11 collet nut, bit.
    b.add(G.cylX(0.009, 0.004, 32), mats.get('steel'), M(0.013, 0, 0));
    b.add(G.cylX(0.006, 0.004, 24), mats.get('steel'), M(0.017, 0, 0));
    b.add(G.hexZ(0.0095, 0.009).rotateY(PI / 2), mats.get('steel'), M(0.023, 0, 0));
    b.add(G.cylX(0.008, 0.004, 24, 0.0055), mats.get('steel'), M(0.0295, 0, 0));
    const shank0 = 0.031, flute0 = bitX - 0.011;
    b.add(G.cylX(r, flute0 - shank0, 16), mats.get('steel'), M((shank0 + flute0) / 2, 0, 0));
    b.add(G.cylX(r * 0.98, bitX - r - flute0, 16), mats.get('steelDark'), M((flute0 + bitX - r) / 2, 0, 0));
    for (let k = 0; k < 5; k++) b.add(G.torusZ(r * 0.95, r * 0.18, 16, 6).rotateY(PI / 2).rotateX(0.5), mats.get('steel'), M(flute0 + 0.001 + k * 0.0018, 0, 0));
    b.add(G.sphere(r * 0.98, 16), mats.get('steelDark'), M(bitX - r, 0, 0));
  },

  ambient() {},
};

// Stepper motor components: orientation of the motor axis in the component
// frame, and whether a gearbox is drawn.
export function stepperComponent(b, c, ctx) {
  const axis = ctx.motorAxis?.[c.id] ?? '+z';
  const A = axis === '-y' ? M(0, 0, 0, PI / 2, 0, 0) : new THREE.Matrix4();
  const round = c.size[0] < 0.04;
  stepperModel(b, ctx.mats, c.size, A, { round, shaftLen: axis === '-y' ? 0.006 : 0.012 });
  connectorAt(b, ctx.mats, ['A1', 'A2', 'B1', 'B2'].map((n) => c.ports.get(n)).filter(Boolean));
}

// ------------------------------------------------------------------ arm
// Link structure, drawn in each link's body frame. geo: { baseHeight,
// shoulderHeight, L2, L3, L4 }.
export function armModels(geo, mats) {
  const out = { base: new Builder(), link1: new Builder(), link2: new Builder(), link3: new Builder(), link4: new Builder() };
  const orange = mats.get('printed'), dark = mats.get('aluDark'), satin = mats.get('aluSatin'), steel = mats.get('steel');
  const h = geo.baseHeight;

  // Base: foot, column, bearing seat, motor riser and bridge.
  const B = out.base;
  B.add(G.lathe([[0, 0], [0.07, 0], [0.07, 0.006], [0.066, 0.0095], [0, 0.0095]], 72), dark);
  for (let k = 0; k < 4; k++) {
    const a = PI / 4 + (k * PI) / 2;
    B.add(G.cylZ(0.0042, 0.0028, 6), steel, M(0.058 * Math.cos(a), 0.058 * Math.sin(a), 0.0108));
  }
  B.add(G.cylZ(0.028, h - 0.024, 48), satin, M(0, 0, 0.0095 + (h - 0.024) / 2));
  B.add(G.lathe([[0.028, h - 0.02], [0.037, h - 0.017], [0.037, h - 0.006], [0.02, h - 0.006]], 64), orange);
  B.add(G.ring(0.02, 0.034, 0.005, 64), steel, M(0, 0, h - 0.0035));
  B.add(G.rbox(0.046, 0.046, 0.0056, 0.002), orange, M(0, -0.05, 0.0123));
  B.add(G.rbox(0.03, 0.028, 0.006, 0.003), orange, M(0, -0.036, h - 0.0115));
  B.add(G.torusZ(0.006, 0.0022, 24, 8), mats.get('blackPlastic'), M(-0.0305, 0, 0.018, 0, PI / 2, 0));

  // Link 1 (turret): ring gear, turntable disc, two cheeks up to the shoulder.
  const L1 = out.link1, sh = geo.shoulderHeight;
  L1.add(G.extrude(gearShape(0.0345, 0.0368, 56), 0.007, 0, 2), steel, M(0, 0, -0.008));
  L1.add(G.lathe([[0.012, 0], [0.045, 0], [0.047, 0.002], [0.047, 0.009], [0.044, 0.012], [0.012, 0.012]], 64), orange);
  const cheek = new THREE.Shape();
  cheek.moveTo(-0.034, 0.006); cheek.lineTo(0.034, 0.006); cheek.lineTo(0.03, sh);
  cheek.absarc(0, sh, 0.03, 0, PI, false); cheek.lineTo(-0.034, 0.006);
  for (const y of [-0.034, 0.026]) L1.add(G.extrude(cheek, 0.008, 0.0012).rotateX(PI / 2).applyMatrix4(M(0, y + 0.008, 0)), dark);
  L1.add(G.cylY(0.017, 0.004, 40), steel, M(0, -0.036, sh));
  L1.add(G.cylY(0.006, 0.0045, 24), dark, M(0, -0.0375, sh));
  L1.add(G.rbox(0.012, 0.012, 0.03, 0.003), mats.get('blackPlastic'), M(-0.04, 0, 0.03));

  // Links 2 and 3: hub at the start, beam, fork at the end.
  const beamLink = (Bd, L, w, t) => {
    Bd.add(G.cylY(0.024, 0.0496, 48), orange, M(0, 0, 0));
    Bd.add(G.cylY(0.0105, 0.0502, 32), steel, M(0, 0, 0));
    const x0 = 0.018, x1 = L - 0.05;
    Bd.add(G.extrude(roundRect(t, w, 0.008), x1 - x0, 0.001, 6).rotateY(PI / 2), satin, M(x0, 0, 0));
    Bd.add(G.rbox(0.03, 0.064, t, 0.004), dark, M(L - 0.05, 0, 0));
    const plate = new THREE.Shape();
    plate.moveTo(-0.055, -t / 2);
    plate.lineTo(-0.02, -0.024);
    plate.absarc(0, 0, 0.026, -PI * 0.62, PI * 0.62, false);
    plate.lineTo(-0.055, t / 2);
    plate.closePath();
    for (const y of [-0.032, 0.025]) Bd.add(G.extrude(plate, 0.007, 0.001).rotateX(-PI / 2).applyMatrix4(M(L, y, 0)), dark);
    Bd.add(G.cylY(0.012, 0.003, 32), steel, M(L, -0.0335, 0));
    // Cable clips on the harness face (+z).
    for (const x of [0.035, L - 0.07]) Bd.add(G.rbox(0.008, 0.02, 0.004, 0.0015), mats.get('blackPlastic'), M(x, 0, t / 2 + 0.002));
  };
  beamLink(out.link2, geo.L2, 0.036, 0.028);
  beamLink(out.link3, geo.L3, 0.032, 0.026);

  // Link 4 (wrist).
  const L4b = out.link4, L4 = geo.L4;
  L4b.add(G.cylY(0.022, 0.0496, 48), orange, M(0, 0, 0));
  L4b.add(G.cylY(0.0095, 0.0502, 32), steel, M(0, 0, 0));
  L4b.add(G.rbox(L4 - 0.012, 0.04, 0.04, 0.005), dark, M(0.006 + (L4 - 0.012) / 2, 0, 0));
  L4b.add(G.rbox(0.008, 0.02, 0.004, 0.0015), mats.get('blackPlastic'), M(0.02, 0, 0.022));
  return out;
}

// ------------------------------------------------------------------ rack
// Two stands with forks under the parked tool plates. `toWorld` is not
// needed: everything is given in world coordinates and shifted by `origin`.
export function rackModel(b, mats, spec, origin) {
  const dark = mats.get('aluDark'), satin = mats.get('aluSatin'), orange = mats.get('printed');
  const P = (x, y, z) => M(x - origin[0], y - origin[1], z - origin[2]);
  const railX = 0.112, railZ = spec.rack.hotend[2] - 0.0135;
  const groups = [
    ['hotend'].map((k) => spec.rack[k]),
    ['probe', 'spindle'].map((k) => spec.rack[k]),
  ];
  for (const g of groups) {
    const ys = g.map((p) => p[1]);
    const y0 = Math.min(...ys) - 0.045, y1 = Math.max(...ys) + 0.045;
    // 20x20 rail on one post (at the end away from the front).
    b.add(G.rbox(0.02, y1 - y0, 0.02, 0.0015), satin, P(railX, (y0 + y1) / 2, railZ));
    const yp = y1 - 0.01;
    b.add(G.rbox(0.02, 0.02, railZ - 0.01, 0.0015), satin, P(railX, yp, (railZ - 0.01) / 2));
    b.add(G.rbox(0.06, 0.04, 0.006, 0.002), dark, P(railX, yp, 0.003));
    for (const H of g) {
      // Fork plate: a U open towards -x (towards the arm).
      const fork = new THREE.Shape();
      const len = railX - H[0] + 0.003, w = 0.062, slot = 0.046;
      fork.moveTo(0, -w / 2); fork.lineTo(len, -w / 2); fork.lineTo(len, w / 2); fork.lineTo(0, w / 2);
      fork.lineTo(0, slot / 2); fork.lineTo(0.02, slot / 2);
      fork.absarc(0.02, 0, slot / 2, PI / 2, -PI / 2, true);
      fork.lineTo(0, -slot / 2); fork.closePath();
      const t = 0.005;
      b.add(G.extrude(fork, t, 0.0008), orange, P(H[0] - 0.02 + 0.0, H[1], H[2] - 0.0085 - t));
      b.add(G.rbox(0.012, 0.03, 0.012, 0.002), orange, P(railX - 0.014, H[1], railZ + 0.001));
    }
  }
}
