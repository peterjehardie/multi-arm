// Building blocks for the 3D scene: a small PBR material library, geometry
// helpers (all in metres, Z up) and a Builder that collects the pieces of one
// model and merges them into one mesh per material, so a detailed part costs
// a handful of draw calls rather than dozens.

import * as THREE from 'three';
import { mergeGeometries } from '../vendor/BufferGeometryUtils.js';
import { RoundedBoxGeometry } from '../vendor/RoundedBoxGeometry.js';

// ------------------------------------------------------------------ materials
const STD = (color, metalness, roughness, extra = {}) => ({ color, metalness, roughness, ...extra });
export const MATERIALS = {
  alu: STD('#c6ccd2', 1.0, 0.34),
  aluSatin: STD('#b4bbc2', 0.9, 0.46),
  aluDark: STD('#2c3036', 0.75, 0.42),
  steel: STD('#dfe2e6', 1.0, 0.22),
  steelDark: STD('#6d737a', 1.0, 0.38),
  brass: STD('#d8a84c', 1.0, 0.3),
  copper: STD('#c47a45', 1.0, 0.32),
  gold: STD('#e9bf5f', 1.0, 0.24),
  motor: STD('#1f2124', 0.35, 0.5),
  blackPlastic: STD('#17181a', 0.0, 0.55),
  greyPlastic: STD('#80868d', 0.0, 0.6),
  whitePlastic: STD('#ebe6d8', 0.0, 0.5),
  bluePlastic: STD('#2462b8', 0.0, 0.45),
  greenTerminal: STD('#2e9d5a', 0.0, 0.5),
  redPlastic: STD('#c43a2c', 0.0, 0.5),
  printed: STD('#ef7b2d', 0.0, 0.52),
  printedDark: STD('#34383e', 0.0, 0.6),
  pcbGreen: STD('#1f6e3d', 0.1, 0.42),
  pcbPurple: STD('#5c2d8c', 0.1, 0.42),
  pcbBlue: STD('#1d4f9e', 0.1, 0.42),
  pcbBlack: STD('#18191c', 0.1, 0.45),
  chip: STD('#121315', 0.2, 0.35),
  rubber: STD('#141414', 0.0, 0.88),
  silicone: STD('#b3402f', 0.0, 0.72),
  pei: STD('#222326', 0.1, 0.78),
  ruby: STD('#c3163f', 0.0, 0.08),
  glass: STD('#d9e4e8', 0.0, 0.1),
  psuSheet: STD('#b7bdc3', 0.85, 0.48),
  slot: STD('#08090a', 0.0, 0.9),
  ledRed: STD('#ff3b30', 0.0, 0.4, { emissive: '#ff2a1a', emissiveIntensity: 1.6 }),
  ledGreen: STD('#35e06b', 0.0, 0.4, { emissive: '#1fd25a', emissiveIntensity: 1.6 }),
  ledBlue: STD('#3aa0ff', 0.0, 0.4, { emissive: '#2a8cff', emissiveIntensity: 1.6 }),
  screen: STD('#0c1a2b', 0.0, 0.25, { emissive: '#1a3a5c', emissiveIntensity: 0.9 }),
  laptop: STD('#9aa1a8', 0.85, 0.42),
  keyboard: STD('#1b1d20', 0.0, 0.7),
  filament: STD('#f2f2f0', 0.0, 0.35),
  wood: STD('#caa77a', 0.0, 0.62),
};

export class MaterialLib {
  constructor(track) {
    this.track = track;
    this.cache = new Map();
  }
  get(name) {
    let m = this.cache.get(name);
    if (!m) {
      const def = MATERIALS[name];
      if (!def) throw new Error(`no material ${name}`);
      m = this.track(new THREE.MeshStandardMaterial(def));
      m.name = name;
      this.cache.set(name, m);
    }
    return m;
  }
  // A private copy (for parts that glow with temperature or change colour).
  unique(name, over = {}) {
    const m = this.track(new THREE.MeshStandardMaterial({ ...MATERIALS[name], ...over }));
    m.name = `${name}*`;
    return m;
  }
  color(hex, metalness = 0, roughness = 0.5) {
    const key = `#${hex}|${metalness}|${roughness}`;
    let m = this.cache.get(key);
    if (!m) {
      m = this.track(new THREE.MeshStandardMaterial({ color: hex, metalness, roughness }));
      this.cache.set(key, m);
    }
    return m;
  }
}

// ------------------------------------------------------------------ transforms
const _e = new THREE.Euler(), _q = new THREE.Quaternion(), _v = new THREE.Vector3(), _s = new THREE.Vector3(1, 1, 1);
// Matrix from a position and XYZ Euler angles (radians).
export function M(x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0) {
  _e.set(rx, ry, rz);
  _q.setFromEuler(_e);
  return new THREE.Matrix4().compose(_v.set(x, y, z), _q, _s);
}
// Matrix that maps +Z onto the direction d, placed at p.
export function alongZ(p, d) {
  _v.set(d[0], d[1], d[2]).normalize();
  _q.setFromUnitVectors(new THREE.Vector3(0, 0, 1), _v);
  return new THREE.Matrix4().compose(new THREE.Vector3(p[0], p[1], p[2]), _q, _s);
}

// ------------------------------------------------------------------ geometry
export const G = {
  box: (sx, sy, sz) => new THREE.BoxGeometry(sx, sy, sz),
  rbox: (sx, sy, sz, r, seg = 2) => new RoundedBoxGeometry(sx, sy, sz, seg, Math.min(r, sx / 2, sy / 2, sz / 2) * 0.999),
  // Cylinders with the axis along Z / X / Y.
  cylZ: (r, h, seg = 32, r2 = r) => new THREE.CylinderGeometry(r2, r, h, seg).rotateX(Math.PI / 2),
  cylX: (r, h, seg = 32, r2 = r) => new THREE.CylinderGeometry(r, r2, h, seg).rotateZ(-Math.PI / 2),
  cylY: (r, h, seg = 32, r2 = r) => new THREE.CylinderGeometry(r2, r, h, seg),
  sphere: (r, seg = 16) => new THREE.SphereGeometry(r, seg, Math.max(6, seg * 0.75 | 0)),
  torusZ: (R, r, seg = 48, rs = 10) => new THREE.TorusGeometry(R, r, rs, seg),
  // Surface of revolution about Z from [r, z] pairs.
  lathe: (pts, seg = 40) => new THREE.LatheGeometry(pts.map(([r, z]) => new THREE.Vector2(Math.max(0, r), z)), seg).rotateX(Math.PI / 2),
  // Ring (washer / race) about Z.
  ring: (r0, r1, h, seg = 48) => G.lathe([[r0, -h / 2], [r1, -h / 2], [r1, h / 2], [r0, h / 2], [r0, -h / 2]], seg),
  // 2D shape extruded along +Z from 0 to depth.
  extrude: (shape, depth, bevel = 0, curveSegments = 8) => new THREE.ExtrudeGeometry(shape, {
    depth: depth - 2 * bevel, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments,
  }).translate(0, 0, bevel),
  hexZ: (r, h) => new THREE.CylinderGeometry(r, r, h, 6).rotateX(Math.PI / 2),
};

// Rounded rectangle centred on the origin.
export function roundRect(w, h, r, shape = new THREE.Shape()) {
  const x = -w / 2, y = -h / 2;
  r = Math.min(r, w / 2, h / 2);
  shape.moveTo(x + r, y);
  shape.lineTo(x + w - r, y); shape.quadraticCurveTo(x + w, y, x + w, y + r);
  shape.lineTo(x + w, y + h - r); shape.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  shape.lineTo(x + r, y + h); shape.quadraticCurveTo(x, y + h, x, y + h - r);
  shape.lineTo(x, y + r); shape.quadraticCurveTo(x, y, x + r, y);
  return shape;
}
export function circlePath(r, cx = 0, cy = 0, path = new THREE.Path()) {
  path.absarc(cx, cy, r, 0, Math.PI * 2, true);
  return path;
}
// Square with chamfered corners (NEMA motor outline).
export function chamferSquare(s, c) {
  const h = s / 2, sh = new THREE.Shape();
  sh.moveTo(-h + c, -h); sh.lineTo(h - c, -h); sh.lineTo(h, -h + c); sh.lineTo(h, h - c);
  sh.lineTo(h - c, h); sh.lineTo(-h + c, h); sh.lineTo(-h, h - c); sh.lineTo(-h, -h + c); sh.closePath();
  return sh;
}

// ------------------------------------------------------------------ builder
// Collects (geometry, material, matrix) triples in one local frame and emits
// one merged mesh per material.
export class Builder {
  constructor() { this.groups = new Map(); }
  add(geom, mat, matrix) {
    if (matrix) geom.applyMatrix4(matrix);
    if (!this.groups.has(mat)) this.groups.set(mat, []);
    this.groups.get(mat).push(geom);
    return this;
  }
  get empty() { return this.groups.size === 0; }
  // Returns an array of meshes (not yet parented).
  build({ castShadow = true, receiveShadow = true } = {}) {
    const out = [];
    for (const [mat, geoms] of this.groups) {
      const keepUv = !!(mat.map || mat.emissiveMap);
      const anyIndexed = geoms.some((g) => g.index), allIndexed = geoms.every((g) => g.index);
      const clean = geoms.map((g) => {
        let h = g;
        if (anyIndexed && !allIndexed && h.index) h = h.toNonIndexed();
        for (const k of Object.keys(h.attributes)) if (k !== 'position' && k !== 'normal' && !(keepUv && k === 'uv')) h.deleteAttribute(k);
        h.morphAttributes = {};
        h.clearGroups();
        return h;
      });
      const merged = clean.length === 1 ? clean[0] : mergeGeometries(clean, false);
      for (const g of clean) if (g !== merged) g.dispose();
      for (const g of geoms) if (g !== merged) g.dispose();
      merged.computeBoundingSphere();
      const mesh = new THREE.Mesh(merged, mat);
      mesh.castShadow = castShadow && !mat.transparent;
      mesh.receiveShadow = receiveShadow;
      out.push(mesh);
    }
    this.groups.clear();
    return out;
  }
}
