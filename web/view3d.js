// 3D view of the machine. One three.js Group per simulation Body; every
// component is a procedural model on its body (web/scene/models.js), the arm
// links and gearboxes are drawn from the machine's geometry, wires are swept
// tubes that bundle along shared routes (web/scene/cables.js), the enclosure
// has a working door and fan (web/scene/enclosure.js), and the stage adds
// image-based lighting, soft shadows and the workbench (web/scene/stage.js).
//
// Public API (used by app.js): new MachineView(m), .root, .update(), .pick(),
// .setSelection(), .setToolpath(), .toolpathVisible, .colorByCurrent,
// .showCouplings, .showEnclosure, .enclosureOpacity, .updateWorkpiece(),
// .attach(renderer, scene), .dispose(), plus .bodyGroups / .componentMeshes.

import * as THREE from 'three';
import { MATERIAL_BY_ID } from '../src/process/workpiece.js';
import { MaterialLib, Builder, G, M } from './scene/kit.js';
import { MODELS, stepperComponent, gearboxModel, armModels, rackModel, beltGeometry } from './scene/models.js';
import { Stage } from './scene/stage.js';
import { EnclosureView } from './scene/enclosure.js';
import { CableView } from './scene/cables.js';

export const KIND_COLORS = {
  stepper: '#3b4046', driver: '#7d3c98', controller: '#1e8449', psu: '#b8bec4', buck: '#1f5f8b',
  terminal: '#16a085', mosfet: '#2e86de', heater: '#c0392b', thermistor: '#f4f6f7', 'limit-switch': '#141414',
  servo: '#1b4f72', 'slip-ring': '#b7950b', 'changer-master': '#a3acb3', 'tool-plate': '#c3cbd1',
  extruder: '#34495e', 'dc-motor': '#6c7a89', 'thermal-mass': '#cfd4d8', ambient: '#5dade2',
  turntable: '#4d5656', host: '#4a5058', part: '#5d6d7e', arm: '#d5d8dc', probe: '#2c3e50', enclosure: '#9fb3c8',
};

export const CONN_COLORS = {
  gearbox: '#f39c12', cam: '#95a5a6', linkage: '#95a5a6', 'sensor-coupling': '#95a5a6', 'gear-mesh': '#f39c12',
  'thermal-contact': '#ff6b6b', convection: '#74b9ff', 'pogo-contact': '#ffe066', deposition: '#e67e22',
  cutting: '#5dade2', 'probe-contact': '#2ecc71',
};

function setFromTf(obj, T) {
  const R = T.R, p = T.p;
  obj.matrix.set(R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
}
function fixed(obj) { obj.matrixAutoUpdate = false; return obj; }
function mountMatrix(mount) {
  const m = new THREE.Matrix4();
  setFromTf({ matrix: m }, { R: mount.R ?? [1, 0, 0, 0, 1, 0, 0, 0, 1], p: mount.p ?? [0, 0, 0] });
  return m;
}

// Current magnitude -> heat colour (linear RGB), 0 A dim, >= 2 A bright.
export function heatColor(iAbs, out) {
  return CableView.prototype.heat(iAbs, out);
}

// Laptop screen: a dim terminal with a few lines of G-code-like text bars.
function screenTexture() {
  const c = document.createElement('canvas');
  c.width = 512; c.height = 320;
  const g = c.getContext('2d');
  const grd = g.createLinearGradient(0, 0, 0, 320);
  grd.addColorStop(0, '#0f2236'); grd.addColorStop(1, '#0a1522');
  g.fillStyle = grd; g.fillRect(0, 0, 512, 320);
  g.fillStyle = '#18324d'; g.fillRect(0, 0, 512, 22);
  const cols = ['#5dade2', '#e6e6e6', '#ff9f1c', '#8fd18f', '#c39bd3'];
  let seed = 5;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let row = 0; row < 16; row++) {
    let x = 14;
    const y = 36 + row * 17;
    const n = 2 + (rnd() * 4 | 0);
    for (let k = 0; k < n; k++) {
      const w = 18 + rnd() * 70;
      g.fillStyle = cols[(row + k) % cols.length];
      g.globalAlpha = 0.75;
      g.fillRect(x, y, w, 7);
      x += w + 10;
    }
  }
  g.globalAlpha = 1;
  g.strokeStyle = '#ff9f1c'; g.lineWidth = 2;
  g.beginPath(); g.arc(420, 200, 50, 0, Math.PI * 2); g.stroke();
  g.beginPath(); g.arc(420, 200, 30, 0, Math.PI * 2); g.stroke();
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

const TOOL_COLORS = { hotend: 'printed', spindle: 'bluePlastic', probe: 'greenTerminal' };

export class MachineView {
  constructor(m) {
    this.m = m;
    this.root = new THREE.Group();
    this.root.name = 'machine';
    this.pickables = [];              // meshes with userData.pick
    this.componentMeshes = new Map(); // component id -> [meshes]
    this.connectionMeshes = new Map(); // connection id -> [meshes] (gearboxes, belt)
    this._colorByCurrent = false;
    this.showCouplings = true;
    this.selection = null;
    this.highlight = null;
    this.disposables = [];
    this.hotParts = [];
    this.leds = [];
    this.hlMats = new Map();
    this.mats = new MaterialLib((x) => this.track(x));
    this.lastT = performance.now();
    this.frame = 0;

    this.stage = new Stage((x) => this.track(x));
    this.root.add(this.stage.group);

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
    this.buildDrives();
    this.buildWorkpiece();
    this.buildEnclosure();
    this.buildContactShadows();
    const all = [...m.A.connections.values()];
    this.cables = new CableView(m, all.filter((c) => c.kind === 'wire' || c.kind === 'usb-cable'), (x) => this.track(x));
    this.root.add(this.cables.mesh);
    this.dashed = this.buildLineSet(all.filter((c) => c.kind !== 'wire' && c.kind !== 'usb-cable'));
    this.updateConnections();
    this.installAttachHook();
  }

  track(x) { this.disposables.push(x); return x; }

  // ---------------------------------------------------------------- renderer
  // Let the view set up lighting, environment, tone mapping and shadows. The
  // host may call this explicitly; otherwise it happens on the first frame.
  attach(renderer, scene) {
    this.stage.attach(renderer, scene, this.root);
    this.attached = true;
    if (this.hook) this.hook.visible = false;
  }

  installAttachHook() {
    const g = this.track(new THREE.BufferGeometry());
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(9), 3));
    const mat = this.track(new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: false, depthTest: false }));
    const hook = new THREE.Mesh(g, mat);
    hook.frustumCulled = false;
    hook.renderOrder = -1e6;
    hook.onBeforeRender = (renderer, scene) => { if (!this.attached) this.pendingAttach = { renderer, scene }; };
    this.hook = hook;
    this.root.add(hook);
  }

  // ---------------------------------------------------------------- components
  addMeshes(ref, meshes, parent, localMatrix, map) {
    const key = ref.id;
    if (!map.has(key)) map.set(key, []);
    for (const mesh of meshes) {
      fixed(mesh);
      if (localMatrix) mesh.matrix.copy(localMatrix);
      mesh.userData.pick = ref;
      parent.add(mesh);
      this.pickables.push(mesh);
      map.get(key).push(mesh);
    }
    return meshes;
  }
  addComponentMeshes(c, meshes, parent, local) {
    return this.addMeshes({ type: 'component', id: c.id }, meshes, parent, local, this.componentMeshes);
  }

  buildComponents() {
    const m = this.m, mats = this.mats;
    const motorAxis = { mot_j2: '-y', mot_j3: '-y', mot_j4: '-y' };
    this.screenMat = this.track(new THREE.MeshStandardMaterial({
      color: '#05080c', roughness: 0.2, metalness: 0, emissive: '#ffffff', emissiveIntensity: 0.9,
      emissiveMap: this.track(screenTexture()),
    }));
    for (const c of m.A.components.values()) {
      if (!c.mount || ['workpiece', 'arm', 'ambient', 'enclosure'].includes(c.kind)) continue;
      const parent = this.bodyGroups.get(c.mount.body);
      const local = mountMatrix(c.mount);
      const b = new Builder();
      const ctx = {
        mats, m, spec: m.spec, motorAxis, screenMat: this.screenMat,
        unique: (name, over) => mats.unique(name, over),
        toolColor: TOOL_COLORS[c.toolName],
      };
      let extra = null;
      if (c.kind === 'heater' || c.id === 'hot_block') {
        ctx.hotMat = mats.unique(c.kind === 'heater' ? (c.size[0] > 0.1 ? 'silicone' : 'steel') : 'alu');
        this.hotParts.push({ c, mat: ctx.hotMat });
      }
      if (c.kind === 'stepper') stepperComponent(b, c, ctx);
      else if (c.kind === 'part') rackModel(b, mats, m.spec, c.mount.p);
      else if (c.kind === 'turntable') this.turntableModel(b, c);
      else if (MODELS[c.kind]) extra = MODELS[c.kind](b, c, ctx);
      if (b.empty) b.add(G.rbox(...c.size, 0.001), mats.color(KIND_COLORS[c.kind] ?? '#888888', 0.2, 0.6));
      if (extra?.led) this.leds.push({ c, mat: extra.led });
      this.addComponentMeshes(c, b.build(), parent, local);
      // Brackets for switches that would otherwise float.
      if (c.kind === 'limit-switch') this.switchBracket(c, parent, local);
    }
  }

  switchBracket(c, parent, local) {
    const p = c.mount.p, b = new Builder(), mat = this.mats.get('aluDark');
    if (c.mount.body.name === 'world' && p[2] > 0.02) {
      b.add(G.box(0.006, 0.004, p[2] - 0.003), mat, M(0, -0.007, -(p[2] - 0.003) / 2 - 0.003 + 0.003));
      b.add(G.box(0.02, 0.016, 0.003), mat, M(0, -0.007, -p[2] + 0.0015));
    } else if (c.mount.body.name === 'base') {
      const r = Math.hypot(p[0], p[1]), L = r - 0.03;
      const ang = Math.atan2(p[1], p[0]);
      b.add(G.box(L, 0.006, 0.004), mat, M(-Math.cos(ang) * L / 2, -Math.sin(ang) * L / 2, -0.005, 0, 0, ang));
    } else {
      b.add(G.box(0.014, 0.004, 0.012), mat, M(0, -0.0052, 0));
    }
    this.addComponentMeshes(c, b.build(), parent, local);
  }

  turntableModel(b, c) {
    const mats = this.mats, z0 = c.mount.p[2]; // local z = world z - z0
    b.add(G.lathe([[0.014, 0], [0.062, 0], [0.062, 0.005], [0.059, 0.0065], [0.014, 0.0065]], 72), mats.get('aluDark'), M(0, 0, -z0));
    for (let k = 0; k < 4; k++) {
      const a = Math.PI / 4 + (k * Math.PI) / 2;
      b.add(G.cylZ(0.0042, 0.035, 16), mats.get('steel'), M(0.048 * Math.cos(a), 0.048 * Math.sin(a), 0.0235 - z0));
    }
    b.add(G.ring(0.03, 0.058, 0.006, 72), mats.get('aluSatin'), M(0, 0, 0.044 - z0));
    b.add(G.ring(0.043, 0.0445, 0.0062, 72), mats.get('slot'), M(0, 0, 0.044 - z0));
    b.add(G.cylZ(0.0505, 0.005, 72), mats.get('aluDark'), M(0, 0, 0.05 - z0));
    for (const z of [0.0473, 0.0527]) b.add(G.cylZ(0.0525, 0.0006, 72), mats.get('alu'), M(0, 0, z - z0));
  }

  buildArm() {
    const m = this.m, arm = m.arm;
    const models = armModels(m.truth, this.mats);
    const groups = [this.bodyGroups.get(arm.baseBody), ...arm.bodies.map((b) => this.bodyGroups.get(b))];
    ['base', 'link1', 'link2', 'link3', 'link4'].forEach((k, i) => this.addComponentMeshes(arm, models[k].build(), groups[i]));
  }

  // Gearboxes and the turntable belt: they are connections, so they pick
  // and highlight as the gearbox connection.
  buildDrives() {
    const m = this.m, mats = this.mats, t = m.truth;
    const add = (id, b, body) => {
      if (!m.A.connections.has(id)) return;
      this.addMeshes({ type: 'connection', id }, b.build(), this.bodyGroups.get(body), null, this.connectionMeshes);
    };
    const mot = m.motors;
    // j1: planetary on top of the motor, pinion into the ring gear.
    if (mot.j1) {
      const b = new Builder(), p = mot.j1.mount.p, top = p[2] + mot.j1.size[2] / 2;
      gearboxModel(b, mats, 0.018, [p[0], p[1], 0], 'z', top, t.baseHeight - 0.009);
      const pin = G.extrude(gearShapeSmall(), 0.0075, 0, 2);
      b.add(pin, mats.get('steel'), M(p[0], p[1], t.baseHeight - 0.0085));
      add('gb_j1', b, mot.j1.mount.body);
    }
    const joint = (n, outer, r) => {
      const mo = mot[n];
      if (!mo) return;
      const p = mo.mount.p, b = new Builder();
      gearboxModel(b, mats, r, p, 'y', p[1] - mo.size[2] / 2, outer);
      add(`gb_${n}`, b, mo.mount.body);
    };
    joint('j2', 0.034, 0.019);
    joint('j3', 0.032, 0.019);
    joint('j4', 0.032, 0.017);
    if (mot.table) {
      const p = mot.table.mount.p, c = m.table.center, b = new Builder();
      const top = p[2] + mot.table.size[2] / 2;
      b.add(G.cylZ(0.0075, 0.0055, 24), mats.get('alu'), M(p[0], p[1], top + 0.004));
      for (const z of [top + 0.001, top + 0.0072]) b.add(G.cylZ(0.0092, 0.0008, 24), mats.get('alu'), M(p[0], p[1], z));
      b.add(beltGeometry([p[0], p[1]], 0.0077, [c[0], c[1]], 0.0507, 0.0012, 0.0046), mats.get('rubber'), M(0, 0, 0.0477));
      add('gb_table', b, mot.table.mount.body);
    }
  }

  buildEnclosure() {
    const enc = this.m.enclosure;
    if (!enc) return;
    this.enc = new EnclosureView(enc, this.mats, (x) => this.track(x));
    this.bodyGroups.get(enc.mount.body).add(this.enc.group);
    for (const mesh of this.enc.pickMeshes) { mesh.castShadow = true; mesh.receiveShadow = true; }
    this.pickables.push(...this.enc.pickMeshes);
    this.componentMeshes.set(enc.id, [...this.enc.pickMeshes]);
  }

  buildContactShadows() {
    const s = this.stage, m = this.m;
    const psu = m.psu?.mount?.p;
    if (psu) s.blob(psu[0], psu[1], 0.3, 0.19);
    const host = m.host?.mount?.p;
    if (host) s.blob(host[0], host[1] + 0.02, 0.4, 0.3);
    s.blob(0, 0, 0.2, 0.2);
    const c = m.table?.center;
    if (c) s.blob(c[0], c[1], 0.16, 0.16);
    if (m.enclosure) {
      const { min, max } = m.enclosure.box;
      s.blob((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (max[0] - min[0]) * 1.12, (max[1] - min[1]) * 1.1, 0.0003).material = this.track(new THREE.MeshBasicMaterial({ map: s.blobTex, transparent: true, depthWrite: false, opacity: 0.35 }));
    }
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
    g.setIndex(new THREE.BufferAttribute(idx, 1).setUsage(THREE.DynamicDrawUsage));
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0.02), wp.half * 1.5);
    g.boundingBox = new THREE.Box3(new THREE.Vector3(-wp.half, -wp.half, -0.001), new THREE.Vector3(wp.half, wp.half, 0.1));
    const mat = this.track(new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.5, metalness: 0.0 }));
    const mesh = fixed(new THREE.Mesh(g, mat));
    mesh.userData.pick = { type: 'component', id: wp.id };
    mesh.receiveShadow = true;
    this.bodyGroups.get(wp.body).add(mesh);
    this.pickables.push(mesh);
    this.componentMeshes.set(wp.id, [mesh]);
    this.wpMesh = mesh;
    this.matColors = MATERIAL_BY_ID.map((mm) => new THREE.Color(mm.id === 0 ? '#222326' : mm.color));
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
    let ox0 = n, ox1 = -1, oy0 = n, oy1 = -1; // occupied cells seen in this update
    for (let j = y0; j <= y1; j++)
      for (let i = x0; i <= x1; i++) {
        const k = j * n + i, k3 = k * 3;
        if (h[k] > 0) { if (i < ox0) ox0 = i; if (i > ox1) ox1 = i; if (j < oy0) oy0 = j; if (j > oy1) oy1 = j; }
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
    // Only triangles over (and just around) material are drawn: an empty
    // plate draws nothing and a small part only its own patch of the grid.
    const full = x0 === 0 && y0 === 0 && x1 === n - 1 && y1 === n - 1;
    const o = full ? null : this.wpOcc;
    const occ = ox1 < 0 ? o : o ? [Math.min(o[0], ox0), Math.min(o[1], oy0), Math.max(o[2], ox1), Math.max(o[3], oy1)] : [ox0, oy0, ox1, oy1];
    this.wpOcc = occ;
    this.updateWorkpieceIndex(occ);
    d.any = false;
  }

  updateWorkpieceIndex(occ) {
    const g = this.wpMesh.geometry, n = this.wp.n;
    if (!occ) { this.wpMesh.visible = false; this.wpDrawn = null; return; }
    this.wpMesh.visible = true;
    const cur = this.wpDrawn;
    if (cur && occ[0] >= cur[0] && occ[1] >= cur[1] && occ[2] <= cur[2] && occ[3] <= cur[3]) return;
    const pad = 12;
    const r = [Math.max(0, occ[0] - pad), Math.max(0, occ[1] - pad), Math.min(n - 2, occ[2] + pad), Math.min(n - 2, occ[3] + pad)];
    const idx = g.index.array;
    let q = 0;
    for (let j = r[1]; j <= r[3]; j++)
      for (let i = r[0]; i <= r[2]; i++) {
        const a = j * n + i, b = a + 1, c = a + n, dd = c + 1;
        idx[q++] = a; idx[q++] = b; idx[q++] = dd;
        idx[q++] = a; idx[q++] = dd; idx[q++] = c;
      }
    g.setDrawRange(0, q);
    g.index.clearUpdateRanges();
    g.index.addUpdateRange(0, q);
    g.index.needsUpdate = true;
    this.wpDrawn = r;
  }

  // ---------------------------------------------------------------- couplings
  // Mechanical, thermal and process couplings: dashed lines (two points each).
  buildLineSet(conns) {
    const segs = conns.length;
    const pos = new Float32Array(segs * 6);
    const col = new Float32Array(segs * 6);
    const g = this.track(new THREE.BufferGeometry());
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3).setUsage(THREE.DynamicDrawUsage));
    const mat = this.track(new THREE.LineDashedMaterial({ vertexColors: true, dashSize: 0.006, gapSize: 0.004, transparent: true, opacity: 0.85, depthTest: true }));
    const lines = new THREE.LineSegments(g, mat);
    lines.frustumCulled = false;
    lines.userData.lineSet = true;
    lines.renderOrder = 3;
    this.root.add(lines);
    const baseColors = conns.map((c) => new THREE.Color(c.color ?? CONN_COLORS[c.kind] ?? '#aaaaaa'));
    return { conns, lines, geom: g, baseColors, segs };
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

  couplingPath(c) {
    const a = this.portPos(c.a);
    if (c.kind === 'convection') return [a, [a[0], a[1], a[2] + 0.04]];
    if (c.kind === 'deposition' || c.kind === 'cutting' || c.kind === 'probe-contact') {
      const wp = this.m.workpiece;
      const l = wp.toLocal(a);
      const T = wp.body.T, R = T.R, h = wp.heightAt(l[0], l[1]);
      const p = [R[0] * l[0] + R[1] * l[1] + R[2] * h + T.p[0], R[3] * l[0] + R[4] * l[1] + R[5] * h + T.p[1], R[6] * l[0] + R[7] * l[1] + R[8] * h + T.p[2]];
      return [a, p];
    }
    return [a, this.portPos(c.b)];
  }

  updateConnections() {
    const set = this.dashed;
    if (!this.showCouplings) { set.lines.visible = false; return; }
    set.lines.visible = true;
    const tool = this.m.master.tool;
    const pos = set.geom.attributes.position.array, col = set.geom.attributes.color.array;
    set.conns.forEach((c, idx) => {
      const pts = this.couplingPath(c);
      let visible = true;
      if (c.kind === 'deposition') visible = tool?.toolName === 'hotend';
      if (c.kind === 'cutting') visible = tool?.toolName === 'spindle';
      if (c.kind === 'probe-contact') visible = tool?.toolName === 'probe';
      if (c.kind === 'pogo-contact') visible = c.mated;
      const b = set.baseColors[idx];
      const f = c.kind === 'convection' ? 0.5 : 1;
      const k = idx * 6;
      for (let v = 0; v < 2; v++) {
        const o = k + v * 3, p = pts[v];
        if (!visible) { pos[o] = pos[o + 1] = pos[o + 2] = 0; } else { pos[o] = p[0]; pos[o + 1] = p[1]; pos[o + 2] = p[2]; }
        col[o] = b.r * f; col[o + 1] = b.g * f; col[o + 2] = b.b * f;
      }
    });
    set.geom.attributes.position.needsUpdate = true;
    set.geom.attributes.color.needsUpdate = true;
    set.lines.computeLineDistances();
  }

  // ---------------------------------------------------------------- per frame
  updateBodies() {
    for (const [body, g] of this.bodyGroups) setFromTf(g, body.T);
  }

  update() {
    const now = performance.now();
    const dt = Math.min(0.1, (now - this.lastT) / 1000);
    this.lastT = now;
    this.frame++;
    if (this.pendingAttach && !this.attached) { this.attach(this.pendingAttach.renderer, this.pendingAttach.scene); this.pendingAttach = null; }
    if (this.attached && this.frame % 30 === 0) this.stage.updateTheme();
    this.updateBodies();
    this.updateWorkpiece();
    const movedWires = this.cables.update();
    this.updateConnections();
    this.enc?.update(dt);
    this.updateHotParts();
    this.updateHighlight(movedWires);
  }

  updateHotParts() {
    for (const { c, mat } of this.hotParts) {
      const T = (c.T ?? 293) - 273.15;
      const glow = Math.max(0, Math.min(1, (T - 40) / 200));
      mat.emissive.setRGB(glow * 0.9, glow * 0.18, glow * 0.02);
    }
    for (const { c, mat } of this.leds) mat.emissiveIntensity = c.gate ? 2.2 : 0;
  }

  // ---------------------------------------------------------------- options
  get colorByCurrent() { return this._colorByCurrent; }
  set colorByCurrent(v) {
    this._colorByCurrent = !!v;
    if (this.cables) { this.cables.colorByCurrent = !!v; this.cables.updateColors(true); }
  }
  get showEnclosure() { return this.enc ? this.enc.group.visible : false; }
  set showEnclosure(v) { if (this.enc) this.enc.group.visible = !!v; }
  get enclosureOpacity() { return this.enc ? this.enc.opacity : 0; }
  set enclosureOpacity(v) { this.enc?.setOpacity(v); }

  // ---------------------------------------------------------------- picking
  static visibleChain(o) {
    for (; o; o = o.parent) if (!o.visible) return false;
    return true;
  }

  pick(raycaster) {
    const targets = this.pickables.filter((o) => MachineView.visibleChain(o));
    if (this.showCouplings) targets.push(this.dashed.lines);
    const hits = raycaster.intersectObjects(targets, false);
    const refOf = (h) => {
      let o = h.object;
      if (o.userData.lineSet) {
        const c = this.dashed.conns[Math.floor(h.index / 2)];
        return c ? { type: 'connection', id: c.id } : null;
      }
      while (o && !o.userData.pick) o = o.parent;
      return o ? o.userData.pick : null;
    };
    const valid = hits.filter((h) => !(h.object === this.dashed.lines && !this.dashedVisible(h.index)));
    const firstMesh = valid.find((h) => !h.object.userData.lineSet);
    const firstDash = valid.find((h) => h.object === this.dashed.lines);
    const thr = (raycaster.params.Line?.threshold ?? 0.003) * 0.7;
    const wire = this.cables.pick(raycaster.ray, thr);
    // Prefer a wire just behind the surface it runs along, and wires over
    // dashed couplings at about the same depth.
    const near = (h) => h && (!firstMesh || h.distance < firstMesh.distance + 0.006);
    if (near(wire) && (!firstDash || wire.distance < firstDash.distance + 0.03)) return { type: 'connection', id: wire.id };
    if (near(firstDash)) return refOf(firstDash);
    if (firstMesh) return refOf(firstMesh);
    // Nothing else under the pointer: clicking the acrylic selects the enclosure.
    if (this.enc && this.showEnclosure && this.enc.opacity > 0.005) {
      const ph = raycaster.intersectObjects(this.enc.panelMeshes, false);
      if (ph.length) return { type: 'component', id: this.enc.enc.id };
    }
    return null;
  }

  dashedVisible(vertexIndex) {
    const p = this.dashed.geom.attributes.position.array, k = Math.floor(vertexIndex / 2) * 6;
    return p[k] !== 0 || p[k + 1] !== 0 || p[k + 2] !== 0 || p[k + 3] !== 0 || p[k + 4] !== 0 || p[k + 5] !== 0;
  }

  // ---------------------------------------------------------------- selection
  hlMaterial(mat) {
    let h = this.hlMats.get(mat);
    if (!h) {
      h = mat.clone();
      if (h.emissive) { h.emissive.set('#0b5d80'); h.emissiveIntensity = 1; }
      this.hlMats.set(mat, this.track(h));
    }
    return h;
  }

  setSelection(ref) {
    this.clearHighlight();
    this.selection = ref;
    if (!ref) return;
    const meshes = ref.type === 'component' ? this.componentMeshes.get(ref.id) : this.connectionMeshes.get(ref.id);
    if (meshes?.length) {
      const hl = [];
      for (const mesh of meshes) {
        if (mesh === this.wpMesh) continue;
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 35),
          new THREE.LineBasicMaterial({ color: '#00e5ff', depthTest: false, transparent: true, opacity: 0.55 }));
        edges.renderOrder = 10;
        edges.raycast = () => {};
        mesh.add(edges);
        hl.push(edges);
        if (!this.isHot(mesh.material)) { mesh.userData.savedMat = mesh.material; mesh.material = this.hlMaterial(mesh.material); }
      }
      this.highlight = { type: 'meshes', objs: hl, meshes };
      return;
    }
    if (ref.type !== 'connection') return;
    const mat = new THREE.MeshBasicMaterial({ color: '#00e5ff', depthTest: false, transparent: true, opacity: 0.8 });
    const tube = new THREE.Mesh(new THREE.BufferGeometry(), mat);
    tube.renderOrder = 10;
    tube.frustumCulled = false;
    tube.raycast = () => {};
    this.root.add(tube);
    this.highlight = { type: 'connection', tube, id: ref.id };
    this.updateHighlight(true, true);
  }

  isHot(mat) { return this.hotParts.some((h) => h.mat === mat); }

  clearHighlight() {
    const h = this.highlight;
    if (!h) return;
    if (h.type === 'meshes') {
      for (const o of h.objs) { o.parent?.remove(o); o.geometry.dispose(); o.material.dispose(); }
      for (const mesh of h.meshes) if (mesh.userData.savedMat) { mesh.material = mesh.userData.savedMat; delete mesh.userData.savedMat; }
    } else {
      this.root.remove(h.tube); h.tube.geometry.dispose(); h.tube.material.dispose();
    }
    this.highlight = null;
  }

  updateHighlight(wiresMoved = false, force = false) {
    const h = this.highlight;
    if (!h || h.type !== 'connection') return;
    const c = this.m.A.connections.get(h.id);
    if (!c) return;
    if (c.kind === 'wire' || c.kind === 'usb-cable') {
      if (!force && !wiresMoved) return;
      const g = this.cables.highlightGeometry(h.id, h.tube.geometry);
      if (g && g !== h.tube.geometry) { h.tube.geometry.dispose(); h.tube.geometry = g; }
      return;
    }
    this.hlTick = (this.hlTick ?? 0) + 1;
    if (!force && this.hlTick % 2) return;
    const pts = this.couplingPath(c).map((p) => new THREE.Vector3(p[0], p[1], p[2]));
    if (pts[0].distanceTo(pts[1]) < 1e-6) pts[1] = pts[1].clone().add(new THREE.Vector3(0, 0, 0.004));
    const geom = new THREE.TubeGeometry(new THREE.LineCurve3(pts[0], pts[1]), 8, 0.0016, 6, false);
    h.tube.geometry.dispose();
    h.tube.geometry = geom;
  }

  // ---------------------------------------------------------------- toolpath
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
  get toolpathVisible() { return this.showToolpath ?? true; }

  dispose() {
    this.clearHighlight();
    this.stage.detach();
    this.root.parent?.remove(this.root);
    this.root.traverse((o) => { if (o.isMesh || o.isLine) o.geometry?.dispose(); });
    for (const d of this.disposables) d.dispose?.();
  }
}

// Pinion for the base drive (12 teeth).
function gearShapeSmall() {
  const sh = new THREE.Shape(), teeth = 12, n = teeth * 4, r0 = 0.0106, r1 = 0.0124;
  for (let k = 0; k <= n; k++) {
    const a = (k / n) * 2 * Math.PI, ph = k % 4, r = ph === 1 || ph === 2 ? r1 : r0;
    if (k === 0) sh.moveTo(r * Math.cos(a), r * Math.sin(a)); else sh.lineTo(r * Math.cos(a), r * Math.sin(a));
  }
  return sh;
}
