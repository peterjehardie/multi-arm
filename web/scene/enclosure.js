// Enclosure: 2020 aluminium extrusion frame, acrylic panels, a hinged door
// on the +x side (animated from enclosure.doorOpen), an exhaust fan on the
// back wall (spinning with enclosure.fanSpeed), cable glands, and the door
// switch. Only the frame, the door's own frame and handle, the fan and the
// glands are pickable; the acrylic never blocks picking of what is inside.

import * as THREE from 'three';
import { G, M, Builder, alongZ } from './kit.js';
import { fanParts } from './models.js';

const PROFILE = 0.02;

// 2020 V-slot cross-section (centred), extruded along +Z by `len`.
let profileShape = null;
function extrusion(len) {
  if (!profileShape) {
    const s = PROFILE / 2, w = 0.0031, d = 0.0045;
    const sh = new THREE.Shape();
    // Walk round the square, cutting a slot in the middle of every side.
    const pts = [
      [-s, -s], [-w, -s], [-w * 0.7, -s + d], [w * 0.7, -s + d], [w, -s],
      [s, -s], [s, -w], [s - d, -w * 0.7], [s - d, w * 0.7], [s, w],
      [s, s], [w, s], [w * 0.7, s - d], [-w * 0.7, s - d], [-w, s],
      [-s, s], [-s, w], [-s + d, w * 0.7], [-s + d, -w * 0.7], [-s, -w],
    ];
    pts.forEach(([x, y], k) => (k ? sh.lineTo(x, y) : sh.moveTo(x, y)));
    sh.closePath();
    const hole = new THREE.Path();
    hole.absarc(0, 0, 0.0021, 0, Math.PI * 2, true);
    sh.holes.push(hole);
    profileShape = sh;
  }
  return G.extrude(profileShape, len, 0.0003, 6);
}

export class EnclosureView {
  constructor(enc, mats, track) {
    this.enc = enc;
    this.group = new THREE.Group();
    this.group.name = 'enclosure';
    this.pickMeshes = [];   // frame, door frame, fan, glands
    this.panelMeshes = [];  // acrylic (pick only as a last resort)
    this.doorAngle = enc.doorOpen ? 1 : 0;
    this.opacity = 0.1;
    const { min, max } = enc.box;
    const P = PROFILE, h = P / 2;
    this.acrylic = track(new THREE.MeshStandardMaterial({
      color: '#dcebf2', metalness: 0, roughness: 0.04, transparent: true, opacity: this.opacity,
      depthWrite: false, side: THREE.DoubleSide, envMapIntensity: 0.9,
    }));
    // ---- frame
    const fb = new Builder();
    const alu = mats.get('aluSatin');
    const beam = (a, b) => {
      const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const len = Math.hypot(...d);
      fb.add(extrusion(len), alu, alongZ(a, d));
    };
    const x0 = min[0] + h, x1 = max[0] - h, y0 = min[1] + h, y1 = max[1] - h, z0 = h, z1 = max[2] - h;
    for (const x of [x0, x1]) for (const y of [y0, y1]) beam([x, y, 0], [x, y, max[2]]);
    for (const z of [z0, z1]) {
      for (const y of [y0, y1]) beam([x0 + h, y, z], [x1 - h, y, z]);
      for (const x of [x0, x1]) beam([x, y0 + h, z], [x, y1 - h, z]);
    }
    // Corner brackets.
    const dark = mats.get('aluDark');
    for (const x of [x0, x1]) for (const y of [y0, y1]) for (const z of [z0 + P, z1 - P])
      fb.add(G.box(0.022, 0.022, 0.003), dark, M(x + (x === x0 ? 0.004 : -0.004), y + (y === y0 ? 0.004 : -0.004), z));
    // Feet.
    for (const x of [x0, x1]) for (const y of [y0, y1]) fb.add(G.cylZ(0.012, 0.004, 24), mats.get('rubber'), M(x, y, 0.002));
    this.addPick(fb);

    // ---- fixed panels: back (-x), sides (±y), top.
    const t = 0.004;
    const panel = (sx, sy, sz, x, y, z) => {
      const m = new THREE.Mesh(track(G.box(sx, sy, sz)), this.acrylic);
      m.position.set(x, y, z);
      m.renderOrder = 6;
      this.group.add(m);
      this.panelMeshes.push(m);
      return m;
    };
    const cx = (min[0] + max[0]) / 2, cy = (min[1] + max[1]) / 2;
    const W = max[0] - min[0], D = max[1] - min[1], H = max[2];
    panel(t, D, H, min[0] - t / 2, cy, H / 2);
    panel(W, t, H, cx, min[1] - t / 2, H / 2);
    panel(W, t, H, cx, max[1] + t / 2, H / 2);
    panel(W + 2 * t, D + 2 * t, t, cx, cy, max[2] + t / 2);

    // ---- door on +x, hinged on the +y edge.
    const door = new THREE.Group();
    door.position.set(max[0] + t / 2 + 0.001, max[1], 0);
    this.door = door;
    this.group.add(door);
    const dW = D, dH = H;
    const pane = new THREE.Mesh(track(G.box(t, dW - 0.012, dH - 0.012)), this.acrylic);
    pane.position.set(0, -dW / 2, dH / 2);
    pane.renderOrder = 6;
    door.add(pane);
    this.panelMeshes.push(pane);
    const db = new Builder();
    const edge = mats.get('aluDark');
    const e = 0.012;
    db.add(G.box(t + 0.003, dW, e), edge, M(0, -dW / 2, e / 2));
    db.add(G.box(t + 0.003, dW, e), edge, M(0, -dW / 2, dH - e / 2));
    db.add(G.box(t + 0.003, e, dH), edge, M(0, -e / 2, dH / 2));
    db.add(G.box(t + 0.003, e, dH), edge, M(0, -dW + e / 2, dH / 2));
    // Handle near the free (-y) edge, hinges on the +y edge.
    db.add(G.cylZ(0.0045, 0.12, 20), mats.get('steel'), M(0.024, -dW + 0.045, dH * 0.5));
    for (const z of [dH * 0.5 - 0.05, dH * 0.5 + 0.05]) db.add(G.cylX(0.0045, 0.024, 16), mats.get('steel'), M(0.012, -dW + 0.045, z));
    for (const z of [0.07, dH - 0.07]) db.add(G.cylZ(0.006, 0.05, 20), mats.get('steel'), M(-0.002, 0.004, z));
    // Actuator for the door switch.
    const sw = enc.ports.get('DOOR_COM').worldPos();
    db.add(G.box(0.008, 0.012, 0.018), mats.get('redPlastic'), M(-0.006, sw[1] - max[1], sw[2] - 0.005));
    for (const mesh of db.build()) { door.add(mesh); this.tag(mesh); }

    // Door switch on a bracket from the front post.
    const sb = new Builder();
    sb.add(G.rbox(0.01, 0.02, 0.012, 0.001), mats.get('blackPlastic'), M(sw[0] - 0.012, sw[1], sw[2] - 0.005));
    sb.add(G.box(0.004, Math.abs(sw[1] - y0) + 0.01, 0.016), dark, M(sw[0] - 0.02, (sw[1] + y0) / 2, sw[2] - 0.005));
    this.addPick(sb);

    // ---- exhaust fan on the back wall (outside), with grille.
    const fp = enc.ports.get('FAN+').worldPos();
    const fanC = [min[0] - t - 0.013, fp[1], fp[2] - 0.035];
    this.fanCentre = fanC;
    const fanSize = 0.08;
    const { frame, hub, bladeGeoms } = fanParts(fanSize, 0.025, 7);
    const toBack = M(fanC[0], fanC[1], fanC[2], 0, -Math.PI / 2, 0);
    const fanB = new Builder();
    fanB.add(frame.applyMatrix4(toBack), mats.get('blackPlastic'));
    for (let k = 1; k <= 3; k++) fanB.add(G.torusZ(fanSize * 0.15 * k, 0.0009, 48, 6).applyMatrix4(M(fanC[0] - 0.0135, fanC[1], fanC[2], 0, Math.PI / 2, 0)), mats.get('steel'));
    fanB.add(G.box(0.0015, fanSize * 0.92, 0.0015), mats.get('steel'), M(fanC[0] - 0.0135, fanC[1], fanC[2]));
    fanB.add(G.box(0.0015, 0.0015, fanSize * 0.92), mats.get('steel'), M(fanC[0] - 0.0135, fanC[1], fanC[2]));
    this.addPick(fanB);
    const rotor = new THREE.Group();
    rotor.position.set(fanC[0], fanC[1], fanC[2]);
    rotor.rotation.y = -Math.PI / 2;
    const rb = new Builder();
    rb.add(hub, mats.get('blackPlastic'));
    for (const g of bladeGeoms) rb.add(g, mats.get('greyPlastic'));
    for (const mesh of rb.build()) { rotor.add(mesh); this.tag(mesh); }
    this.rotor = rotor;
    this.group.add(rotor);
    this.fanAngle = 0;

    // ---- cable glands through the back wall.
    const gb = new Builder();
    for (const g of enc.glands ?? []) {
      gb.add(G.hexZ(0.0105, 0.005).rotateY(Math.PI / 2), mats.get('blackPlastic'), M(g[0] + 0.003, g[1], g[2]));
      gb.add(G.cylX(0.0095, 0.006, 24, 0.006), mats.get('blackPlastic'), M(g[0] - t - 0.003, g[1], g[2]));
      gb.add(G.hexZ(0.0105, 0.004).rotateY(Math.PI / 2), mats.get('blackPlastic'), M(g[0] - t - 0.0075, g[1], g[2]));
    }
    this.addPick(gb);
    this.setOpacity(this.opacity);
  }

  tag(mesh) {
    mesh.userData.pick = { type: 'component', id: this.enc.id };
    this.pickMeshes.push(mesh);
  }
  addPick(b) {
    for (const mesh of b.build()) { this.group.add(mesh); this.tag(mesh); }
  }

  setOpacity(v) {
    this.opacity = Math.max(0, Math.min(1, v));
    this.acrylic.opacity = this.opacity;
    this.acrylic.depthWrite = this.opacity > 0.95;
    this.acrylic.transparent = this.opacity < 1;
    this.acrylic.needsUpdate = true;
    for (const m of this.panelMeshes) m.visible = this.opacity > 0.005;
  }

  update(dt) {
    const target = this.enc.doorOpen ? 1 : 0;
    const k = 1 - Math.exp(-dt / 0.25);
    this.doorAngle += (target - this.doorAngle) * k;
    if (Math.abs(target - this.doorAngle) < 1e-3) this.doorAngle = target;
    // Ease in-out so the door swings rather than slides.
    const s = this.doorAngle * this.doorAngle * (3 - 2 * this.doorAngle);
    this.door.rotation.z = s * (100 * Math.PI) / 180;
    this.fanAngle += dt * (this.enc.fanSpeed ?? 0) * 2 * Math.PI * 6;
    this.rotor.rotation.z = this.fanAngle;
  }
}
