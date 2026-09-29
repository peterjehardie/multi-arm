// 3D view of the machine: one three.js Group per simulation Body, every
// component as a box on its body, arm links from their structure parts, the
// plate and height-map workpiece on the turntable body, and every wire and
// mechanical/thermal connection drawn along its physical path.

import * as THREE from 'three';
import { MATERIAL_BY_ID } from '../src/process/workpiece.js';

export const KIND_COLORS = {
  stepper: '#3b4046',
  driver: '#7d3c98',
  controller: '#1e8449',
  psu: '#b8bec4',
  buck: '#1f5f8b',
  terminal: '#16a085',
  mosfet: '#2e86de',
  heater: '#c0392b',
  thermistor: '#f4f6f7',
  'limit-switch': '#141414',
  servo: '#1b4f72',
  'slip-ring': '#b7950b',
  'changer-master': '#a3acb3',
  'tool-plate': '#c3cbd1',
  extruder: '#34495e',
  'dc-motor': '#6c7a89',
  'thermal-mass': '#cfd4d8',
  ambient: '#5dade2',
  turntable: '#4d5656',
  host: '#4a5058',
  part: '#5d6d7e',
  arm: '#d5d8dc',
};
const METALLIC = new Set(['psu', 'thermal-mass', 'changer-master', 'tool-plate', 'slip-ring', 'arm', 'turntable']);

export const CONN_COLORS = {
  gearbox: '#f39c12',
  cam: '#95a5a6',
  linkage: '#95a5a6',
  'sensor-coupling': '#95a5a6',
  'gear-mesh': '#f39c12',
  'thermal-contact': '#ff6b6b',
  convection: '#74b9ff',
  'pogo-contact': '#ffe066',
  deposition: '#e67e22',
  cutting: '#5dade2',
};

const tmpM = new THREE.Matrix4();
function setFromTf(obj, T) {
  const R = T.R, p = T.p;
  obj.matrix.set(R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
}
function fixed(obj) { obj.matrixAutoUpdate = false; return obj; }

// Cylinder whose axis is Z (three.js cylinders run along Y).
function zCylinder(r, h, seg = 40) {
  const g = new THREE.CylinderGeometry(r, r, h, seg);
  g.rotateX(Math.PI / 2);
  return g;
}

// Current magnitude -> heat colour (linear RGB), 0 A dim, >= 2 A bright.
export function heatColor(iAbs, out) {
  const x = Math.min(1, Math.max(0, iAbs / 2));
  const t = Math.sqrt(x);
  const stops = [
    [0.00, 0.035, 0.04, 0.05],
    [0.15, 0.25, 0.02, 0.01],
    [0.45, 0.85, 0.12, 0.01],
    [0.75, 1.0, 0.45, 0.02],
    [1.00, 1.0, 0.95, 0.55],
  ];
  let k = 1;
  while (k < stops.length - 1 && t > stops[k][0]) k++;
  const a = stops[k - 1], b = stops[k];
  const f = (t - a[0]) / (b[0] - a[0] || 1);
  out[0] = a[1] + (b[1] - a[1]) * f;
  out[1] = a[2] + (b[2] - a[2]) * f;
  out[2] = a[3] + (b[3] - a[3]) * f;
  return out;
}

export class MachineView {
  constructor(m) {
    this.m = m;
    this.root = new THREE.Group();
    this.root.name = 'machine';
    this.pickables = [];          // meshes with userData.pick
    this.componentMeshes = new Map(); // component id -> [meshes]
    this.colorByCurrent = false;
    this.showCouplings = true;
    this.selection = null;
    this.highlight = null;
    this.disposables = [];

    this.bodyGroups = new Map();
    for (const body of m.A.bodies.values()) {
      const g = fixed(new THREE.Group());
      g.name = `body:${body.name}`;
      this.bodyGroups.set(body, g);
      this.root.add(g);
    }
    this.updateBodies();
    this.buildComponents();
    this.buildArm();
    this.buildWorkpiece();
    this.buildConnections();
  }

  track(x) { this.disposables.push(x); return x; }

  // ---------------------------------------------------------------- components
  material(kind, colorOverride) {
    const color = colorOverride ?? KIND_COLORS[kind] ?? '#888888';
    return this.track(new THREE.MeshStandardMaterial({
      color, roughness: METALLIC.has(kind) ? 0.5 : 0.7, metalness: METALLIC.has(kind) ? 0.45 : 0.1,
    }));
  }

  addPickMesh(comp, geom, mat, parent, localMatrix) {
    const mesh = fixed(new THREE.Mesh(this.track(geom), mat));
    if (localMatrix) mesh.matrix.copy(localMatrix);
    mesh.userData.pick = { type: 'component', id: comp.id };
    parent.add(mesh);
    this.pickables.push(mesh);
    if (!this.componentMeshes.has(comp.id)) this.componentMeshes.set(comp.id, []);
    this.componentMeshes.get(comp.id).push(mesh);
    return mesh;
  }

  buildComponents() {
    const m = this.m;
    for (const c of m.A.components.values()) {
      // Room air has a nominal mount but no real shape; the workpiece and arm are drawn specially.
      if (!c.mount || c.kind === 'workpiece' || c.kind === 'arm' || c.kind === 'ambient') continue;
      const parent = this.bodyGroups.get(c.mount.body);
      const local = new THREE.Matrix4();
      setFromTf({ matrix: local }, { R: c.mount.R ?? [1, 0, 0, 0, 1, 0, 0, 0, 1], p: c.mount.p ?? [0, 0, 0] });
      const [sx, sy, sz] = c.size;
      let geom;
      const mat = this.material(c.kind);
      if (c.round) {
        geom = zCylinder(c.round, sz, 64);                         // round bed plate
      } else if (c.kind === 'heater' && sx > 0.1) {
        geom = zCylinder(Math.min(sx, sy) / 2, sz, 64);            // round silicone pad
      } else if (c.kind === 'turntable') {
        geom = zCylinder(0.05, 0.05, 48);
      } else if (c.kind === 'ambient') {
        geom = new THREE.SphereGeometry(0.012, 16, 12);
        mat.transparent = true; mat.opacity = 0.5;
      } else if (c.kind === 'slip-ring') {
        geom = zCylinder(sx / 2, sz, 24);
      } else {
        geom = new THREE.BoxGeometry(sx, sy, sz);
      }
      if (c.kind === 'part') { mat.transparent = true; mat.opacity = 0.45; mat.depthWrite = false; } // see-through rack
      const mesh = this.addPickMesh(c, geom, mat, parent, local);
      mesh.userData.kind = c.kind;

      // Small extras that make the tools and plate readable.
      if (c.round) {
        // Radial marker line on the plate top so rotation is visible.
        const marker = fixed(new THREE.Mesh(this.track(new THREE.BoxGeometry(c.round * 0.95, 0.004, 0.0008)),
          this.track(new THREE.MeshBasicMaterial({ color: '#ff9f1c' }))));
        marker.matrix.makeTranslation(c.round * 0.475, 0, sz / 2 + 0.0004);
        marker.userData.pick = { type: 'component', id: c.id };
        mesh.add(marker);
        const rim = fixed(new THREE.Mesh(this.track(new THREE.TorusGeometry(c.round, 0.0012, 6, 96)),
          this.track(new THREE.MeshBasicMaterial({ color: '#7f8c8d' }))));
        rim.matrix.makeTranslation(0, 0, sz / 2);
        mesh.add(rim);
      }
      if (c.id === 'hot_block') {
        const cone = new THREE.ConeGeometry(0.004, 0.007, 20);
        cone.rotateZ(-Math.PI / 2);                              // apex along +x (the tool axis)
        cone.translate(sx / 2 + 0.0035, 0, 0);
        this.addPickMesh(c, cone, this.material('nozzle', '#d4a017'), mesh);
      }
      if (c.kind === 'dc-motor') {
        const bit = c.ports.get('bit');
        const r = m.spec.tools.spindle.cutterRadius;
        const L = 0.018;
        const g = new THREE.CylinderGeometry(r, r, L, 16);
        g.rotateZ(-Math.PI / 2);
        g.translate(bit.at[0] - L / 2, bit.at[1], bit.at[2]);
        this.addPickMesh(c, g, this.material('cutter', '#e5e8e8'), mesh);
        const collet = new THREE.CylinderGeometry(0.006, 0.008, 0.012, 20);
        collet.rotateZ(-Math.PI / 2);
        collet.translate(sx / 2 + 0.004, 0, 0);
        this.addPickMesh(c, collet, this.material('collet', '#909497'), mesh);
      }
      if (c.kind === 'heater' || c.id === 'hot_block') this.hotParts = [...(this.hotParts ?? []), { c, mat }];
    }
  }

  buildArm() {
    const m = this.m, arm = m.arm;
    const mat = this.material('arm');
    // Base column on the base body (floor to the yaw bearing).
    const h = m.truth.baseHeight;
    const base = new THREE.Matrix4().makeTranslation(0, 0, h / 2);
    this.addPickMesh(arm, zCylinder(0.055, h, 48), this.material('arm', '#aeb6bf'), this.bodyGroups.get(arm.baseBody), base);
    arm.spec.joints.forEach((j, i) => {
      const body = arm.bodies[i];
      const parent = this.bodyGroups.get(body);
      for (const part of j.structure ?? []) {
        const local = new THREE.Matrix4().makeTranslation(part.p[0], part.p[1], part.p[2]);
        this.addPickMesh(arm, new THREE.BoxGeometry(...part.size), mat, parent, local);
      }
      if (i > 0) {
        // Joint hub (axis along the joint axis, here always y for j2..j4).
        const hub = new THREE.CylinderGeometry(0.024, 0.024, 0.05, 32);
        this.addPickMesh(arm, hub, this.material('arm', '#85929e'), parent);
      }
    });
  }

  // ---------------------------------------------------------------- workpiece
  buildWorkpiece() {
    const wp = this.m.workpiece;
    const n = wp.n;
    this.wp = wp;
    const pos = new Float32Array(n * n * 3);
    const nor = new Float32Array(n * n * 3);
    const col = new Float32Array(n * n * 3);
    for (let j = 0; j < n; j++)
      for (let i = 0; i < n; i++) {
        const k = (j * n + i) * 3;
        pos[k] = (i + 0.5) * wp.cell - wp.half;
        pos[k + 1] = (j + 0.5) * wp.cell - wp.half;
        pos[k + 2] = -0.0008;
        nor[k + 2] = 1;
      }
    const idx = new Uint32Array((n - 1) * (n - 1) * 6);
    let q = 0;
    for (let j = 0; j < n - 1; j++)
      for (let i = 0; i < n - 1; i++) {
        const a = j * n + i, b = a + 1, c = a + n, d = c + 1;
        idx[q++] = a; idx[q++] = b; idx[q++] = d;
        idx[q++] = a; idx[q++] = d; idx[q++] = c;
      }
    const g = this.track(new THREE.BufferGeometry());
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('normal', new THREE.BufferAttribute(nor, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3).setUsage(THREE.DynamicDrawUsage));
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0.02), wp.half * 1.5);
    g.boundingBox = new THREE.Box3(new THREE.Vector3(-wp.half, -wp.half, -0.001), new THREE.Vector3(wp.half, wp.half, 0.1));
    const mat = this.track(new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.0 }));
    const mesh = fixed(new THREE.Mesh(g, mat));
    mesh.userData.pick = { type: 'component', id: wp.id };
    this.bodyGroups.get(wp.body).add(mesh);
    this.pickables.push(mesh);
    this.componentMeshes.set(wp.id, [mesh]);
    this.wpMesh = mesh;
    this.matColors = MATERIAL_BY_ID.map((mm) => new THREE.Color(mm.id === 0 ? '#3d4450' : mm.color));
    this.updateWorkpiece();
  }

  updateWorkpiece() {
    const wp = this.wp, d = wp.dirty;
    if (!d.any) return;
    const n = wp.n, h = wp.h, cell = wp.cell;
    const g = this.wpMesh.geometry;
    const pos = g.attributes.position.array, nor = g.attributes.normal.array, col = g.attributes.color.array;
    const x0 = Math.max(0, d.x0 - 1), x1 = Math.min(n - 1, d.x1 + 1);
    const y0 = Math.max(0, d.y0 - 1), y1 = Math.min(n - 1, d.y1 + 1);
    const z = (i, j) => { const v = h[j * n + i]; return v > 0 ? v : -0.0008; };
    for (let j = y0; j <= y1; j++)
      for (let i = x0; i <= x1; i++) {
        const k = j * n + i, k3 = k * 3;
        pos[k3 + 2] = z(i, j);
        const c = this.matColors[wp.mat[k]] ?? this.matColors[0];
        col[k3] = c.r; col[k3 + 1] = c.g; col[k3 + 2] = c.b;
        const il = Math.max(0, i - 1), ir = Math.min(n - 1, i + 1);
        const jd = Math.max(0, j - 1), ju = Math.min(n - 1, j + 1);
        const dx = (z(ir, j) - z(il, j)) / ((ir - il) * cell);
        const dy = (z(i, ju) - z(i, jd)) / ((ju - jd) * cell);
        const inv = 1 / Math.hypot(dx, dy, 1);
        nor[k3] = -dx * inv; nor[k3 + 1] = -dy * inv; nor[k3 + 2] = inv;
      }
    const start = (y0 * n + x0) * 3, count = ((y1 * n + x1) - (y0 * n + x0) + 1) * 3;
    for (const name of ['position', 'normal', 'color']) {
      const a = g.attributes[name];
      a.clearUpdateRanges();
      a.addUpdateRange(start, count);
      a.needsUpdate = true;
    }
    d.any = false;
  }

  // ---------------------------------------------------------------- connections
  buildConnections() {
    const m = this.m;
    const all = [...m.A.connections.values()];
    // Solid: wires and the USB cable. Dashed: everything else.
    this.solid = this.buildLineSet(all.filter((c) => c.kind === 'wire' || c.kind === 'usb-cable'), false);
    this.dashed = this.buildLineSet(all.filter((c) => c.kind !== 'wire' && c.kind !== 'usb-cable'), true);
    this.updateConnections();
  }

  buildLineSet(conns, dashed) {
    let segs = 0;
    const layout = conns.map((c, idx) => {
      const n = dashed ? 2 : c.path().length;
      const entry = { c, first: segs, n, idx };
      segs += Math.max(0, n - 1);
      return entry;
    });
    const owner = new Int32Array(segs);
    for (const e of layout) for (let s = 0; s < e.n - 1; s++) owner[e.first + s] = e.idx;
    const pos = new Float32Array(segs * 6);
    const col = new Float32Array(segs * 6);
    const g = this.track(new THREE.BufferGeometry());
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3).setUsage(THREE.DynamicDrawUsage));
    const mat = this.track(dashed
      ? new THREE.LineDashedMaterial({ vertexColors: true, dashSize: 0.006, gapSize: 0.004, transparent: true, opacity: 0.8 })
      : new THREE.LineBasicMaterial({ vertexColors: true }));
    const lines = new THREE.LineSegments(g, mat);
    lines.frustumCulled = false;
    lines.userData.lineSet = true;
    this.root.add(lines);
    const baseColors = conns.map((c) => new THREE.Color(c.color ?? CONN_COLORS[c.kind] ?? '#aaaaaa'));
    return { conns, layout, owner, lines, geom: g, dashed, baseColors, segs };
  }

  // Where to draw a port. Arm joint ports carry no location of their own
  // (they sit at the arm's mount origin), so use the joint's body origin.
  portPos(port) {
    const o = port.owner;
    if (o === this.m.arm) {
      const i = o.jointPorts.indexOf(port);
      if (i >= 0) return o.bodies[i].T.p;
    }
    return port.worldPos() ?? [0, 0, 0];
  }

  // Drawing path for a non-wire connection (always exactly two points, so the
  // segment layout never changes).
  couplingPath(c) {
    const a = this.portPos(c.a);
    if (c.kind === 'convection') return [a, [a[0], a[1], a[2] + 0.04]]; // heat rising off the surface
    if (c.kind === 'deposition' || c.kind === 'cutting') {
      // Tool tip straight down to the part surface below it.
      const wp = this.m.workpiece;
      const l = wp.toLocal(a);
      const T = wp.body.T, R = T.R, h = wp.heightAt(l[0], l[1]);
      const p = [R[0] * l[0] + R[1] * l[1] + R[2] * h + T.p[0], R[3] * l[0] + R[4] * l[1] + R[5] * h + T.p[1], R[6] * l[0] + R[7] * l[1] + R[8] * h + T.p[2]];
      return [a, p];
    }
    return [a, this.portPos(c.b)];
  }

  // Visual-only fan-out so wires sharing a harness route read as a bundle.
  static fan(i) {
    const s = 0.0009;
    return [(((i * 37) % 7) - 3) * s, (((i * 53) % 7) - 3) * s, (((i * 19) % 5) - 2) * s];
  }

  updateConnections() {
    const tool = this.m.master.tool;
    for (const set of [this.solid, this.dashed]) {
      if (set.dashed && !this.showCouplings) { set.lines.visible = false; continue; }
      set.lines.visible = true;
      const pos = set.geom.attributes.position.array, col = set.geom.attributes.color.array;
      const rgb = [0, 0, 0];
      for (const e of set.layout) {
        const c = e.c;
        const pts = set.dashed ? this.couplingPath(c) : c.path();
        const off = set.dashed ? null : MachineView.fan(e.idx);
        let visible = true;
        // Process contacts only mean something while their tool is on the arm,
        // and pogo pins only touch anything while a tool is locked on.
        if (c.kind === 'deposition') visible = tool?.toolName === 'hotend';
        if (c.kind === 'cutting') visible = tool?.toolName === 'spindle';
        if (c.kind === 'pogo-contact') visible = c.mated;
        // Colour.
        if (!set.dashed && this.colorByCurrent && c.kind === 'wire') heatColor(Math.abs(c.i), rgb);
        else {
          const b = set.baseColors[e.idx];
          let f = 1;
          if (c.kind === 'pogo-contact') f = c.mated ? 1 : 0.25;
          if (c.kind === 'convection') f = 0.5;
          rgb[0] = b.r * f; rgb[1] = b.g * f; rgb[2] = b.b * f;
        }
        for (let s = 0; s < e.n - 1; s++) {
          const k = (e.first + s) * 6;
          for (let v = 0; v < 2; v++) {
            const pi = s + v, p = pts[pi];
            const interior = off && pi > 0 && pi < e.n - 1;
            const o = k + v * 3;
            if (!visible) { pos[o] = pos[o + 1] = pos[o + 2] = 0; }
            else {
              pos[o] = p[0] + (interior ? off[0] : 0);
              pos[o + 1] = p[1] + (interior ? off[1] : 0);
              pos[o + 2] = p[2] + (interior ? off[2] : 0);
            }
            col[o] = rgb[0]; col[o + 1] = rgb[1]; col[o + 2] = rgb[2];
          }
        }
      }
      set.geom.attributes.position.needsUpdate = true;
      set.geom.attributes.color.needsUpdate = true;
      if (set.dashed) set.lines.computeLineDistances();
    }
  }

  // ---------------------------------------------------------------- per frame
  updateBodies() {
    for (const [body, g] of this.bodyGroups) setFromTf(g, body.T);
  }

  update() {
    this.updateBodies();
    this.updateWorkpiece();
    this.updateConnections();
    this.updateHotParts();
    this.updateHighlight();
  }

  updateHotParts() {
    for (const { c, mat } of this.hotParts ?? []) {
      const T = (c.T ?? 293) - 273.15;
      const glow = Math.max(0, Math.min(1, (T - 40) / 200));
      mat.emissive.setRGB(glow * 0.9, glow * 0.18, glow * 0.02);
    }
  }

  // ---------------------------------------------------------------- picking
  pick(raycaster) {
    const targets = [...this.pickables, this.solid.lines];
    if (this.showCouplings) targets.push(this.dashed.lines);
    const hits = raycaster.intersectObjects(targets, true);
    if (!hits.length) return null;
    const refOf = (h) => {
      let o = h.object;
      if (o.userData.lineSet) {
        const set = o === this.solid.lines ? this.solid : this.dashed;
        const seg = Math.floor(h.index / 2);
        const c = set.conns[set.owner[seg]];
        return c ? { type: 'connection', id: c.id } : null;
      }
      while (o && !o.userData.pick) o = o.parent;
      return o ? o.userData.pick : null;
    };
    // Prefer a wire just behind the surface it runs along, and wires over
    // dashed couplings at about the same depth. Hidden (collapsed) coupling
    // segments sit at the origin and must not be picked.
    const valid = hits.filter((h) => !(h.object === this.dashed.lines && !this.dashedVisible(h.index)));
    const firstMesh = valid.find((h) => !h.object.userData.lineSet);
    const firstWire = valid.find((h) => h.object === this.solid.lines);
    const firstDash = valid.find((h) => h.object === this.dashed.lines);
    const near = (h) => h && (!firstMesh || h.distance < firstMesh.distance + 0.006);
    if (near(firstWire) && (!firstDash || firstWire.distance < firstDash.distance + 0.03)) return refOf(firstWire);
    if (near(firstDash)) return refOf(firstDash);
    if (near(firstWire)) return refOf(firstWire);
    return firstMesh ? refOf(firstMesh) : null;
  }

  dashedVisible(vertexIndex) {
    const p = this.dashed.geom.attributes.position.array, k = Math.floor(vertexIndex / 2) * 6;
    return p[k] !== 0 || p[k + 1] !== 0 || p[k + 2] !== 0 || p[k + 3] !== 0 || p[k + 4] !== 0 || p[k + 5] !== 0;
  }

  setSelection(ref) {
    this.clearHighlight();
    this.selection = ref;
    if (!ref) return;
    if (ref.type === 'component') {
      const meshes = this.componentMeshes.get(ref.id) ?? [];
      const hl = [];
      for (const mesh of meshes) {
        if (mesh === this.wpMesh) continue;
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 30),
          new THREE.LineBasicMaterial({ color: '#00e5ff', depthTest: false, transparent: true }));
        edges.renderOrder = 10;
        mesh.add(edges);
        hl.push(edges);
        if (mesh.material.emissive && !this.isHot(mesh.material)) {
          mesh.userData.savedEmissive = mesh.material.emissive.getHex();
          mesh.material.emissive.set('#0a4a66');
        }
      }
      this.highlight = { type: 'component', objs: hl, meshes };
    } else {
      const mat = new THREE.MeshBasicMaterial({ color: '#00e5ff', depthTest: false, transparent: true, opacity: 0.85 });
      const tube = new THREE.Mesh(new THREE.BufferGeometry(), mat);
      tube.renderOrder = 10;
      tube.frustumCulled = false;
      this.root.add(tube);
      this.highlight = { type: 'connection', tube, id: ref.id };
      this.updateHighlight(true);
    }
  }

  isHot(mat) { return (this.hotParts ?? []).some((h) => h.mat === mat); }

  clearHighlight() {
    const h = this.highlight;
    if (!h) return;
    if (h.type === 'component') {
      for (const o of h.objs) { o.parent?.remove(o); o.geometry.dispose(); o.material.dispose(); }
      for (const mesh of h.meshes) {
        if (mesh.userData.savedEmissive !== undefined) { mesh.material.emissive.setHex(mesh.userData.savedEmissive); delete mesh.userData.savedEmissive; }
      }
    } else {
      this.root.remove(h.tube); h.tube.geometry.dispose(); h.tube.material.dispose();
    }
    this.highlight = null;
  }

  updateHighlight(force = false) {
    const h = this.highlight;
    if (!h || h.type !== 'connection') return;
    this.hlTick = (this.hlTick ?? 0) + 1;
    if (!force && this.hlTick % 2) return;
    const c = this.m.A.connections.get(h.id);
    if (!c) return;
    const raw = c.kind === 'wire' || c.kind === 'usb-cable' ? c.path() : this.couplingPath(c);
    const pts = raw.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
    if (pts.length < 2) return;
    // Degenerate (zero-length) paths still get a small marker.
    if (pts[0].distanceTo(pts[pts.length - 1]) < 1e-6 && pts.length === 2) pts[1] = pts[1].clone().add(new THREE.Vector3(0, 0, 0.004));
    const path = new THREE.CurvePath();
    for (let i = 1; i < pts.length; i++) if (pts[i].distanceTo(pts[i - 1]) > 1e-7) path.add(new THREE.LineCurve3(pts[i - 1], pts[i]));
    if (!path.curves.length) return;
    const geom = new THREE.TubeGeometry(path, Math.max(8, path.curves.length * 6), 0.0016, 6, false);
    h.tube.geometry.dispose();
    h.tube.geometry = geom;
  }

  // Planned toolpath from the loaded job, drawn on the plate (it turns with
  // the plate, like the part). segs: Float32Array of x,y,z pairs in metres,
  // kinds: Uint8Array per segment (0 travel, 1 extrude, 2 cut).
  setToolpath(segs, kinds) {
    if (this.toolpath) { this.toolpath.parent?.remove(this.toolpath); this.toolpath.geometry.dispose(); this.toolpath.material.dispose(); this.toolpath = null; }
    if (!segs || !segs.length) return;
    const colors = [new THREE.Color('#5dade2'), new THREE.Color('#ff9f1c'), new THREE.Color('#c39bd3')];
    const col = new Float32Array(segs.length);
    for (let s = 0; s < kinds.length; s++)
      for (let v = 0; v < 2; v++) { const c = colors[kinds[s]]; col.set([c.r, c.g, c.b], (2 * s + v) * 3); }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(segs, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const mat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false });
    this.toolpath = fixed(new THREE.LineSegments(g, mat));
    this.toolpath.renderOrder = 2;
    this.toolpath.visible = this.showToolpath ?? true;
    this.bodyGroups.get(this.m.workpiece.body).add(this.toolpath);
  }
  set toolpathVisible(v) { this.showToolpath = v; if (this.toolpath) this.toolpath.visible = v; }

  dispose() {
    this.clearHighlight();
    this.root.parent?.remove(this.root);
    for (const d of this.disposables) d.dispose?.();
  }
}
