// The stage the machine stands on: image-based lighting from a procedural
// room (RoomEnvironment through PMREM, no network), one soft shadow-casting
// key light, tone mapping, and a plywood workbench with an anti-static mat
// under the electronics bay.

import * as THREE from 'three';
import { RoomEnvironment } from '../vendor/RoomEnvironment.js';
import { G } from './kit.js';

const envCache = new WeakMap(); // renderer -> PMREM texture

function environmentFor(renderer) {
  let tex = envCache.get(renderer);
  if (!tex) {
    const pmrem = new THREE.PMREMGenerator(renderer);
    const room = new RoomEnvironment();
    tex = pmrem.fromScene(room, 0.03).texture;
    room.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); });
    pmrem.dispose();
    envCache.set(renderer, tex);
  }
  return tex;
}

// Plywood top: long grain with a few soft growth rings, drawn once on a canvas.
function plywoodTexture() {
  const c = document.createElement('canvas');
  c.width = 1024; c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = '#d9bf98';
  g.fillRect(0, 0, c.width, c.height);
  let seed = 11;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let k = 0; k < 260; k++) {
    const y = rnd() * c.height, amp = 2 + rnd() * 10, f = 0.002 + rnd() * 0.006, ph = rnd() * 6.28;
    const dark = rnd() < 0.5;
    g.strokeStyle = dark ? `rgba(150,105,60,${0.05 + rnd() * 0.12})` : `rgba(255,240,215,${0.05 + rnd() * 0.1})`;
    g.lineWidth = 0.6 + rnd() * 2.4;
    g.beginPath();
    for (let x = 0; x <= c.width; x += 8) {
      const yy = y + amp * Math.sin(x * f + ph) + 3 * Math.sin(x * f * 3.1 + ph * 2);
      if (x === 0) g.moveTo(x, yy); else g.lineTo(x, yy);
    }
    g.stroke();
  }
  for (let k = 0; k < 6; k++) {
    const x = rnd() * c.width, y = rnd() * c.height;
    const grd = g.createRadialGradient(x, y, 1, x, y, 16 + rnd() * 24);
    grd.addColorStop(0, 'rgba(120,80,40,0.25)'); grd.addColorStop(1, 'rgba(120,80,40,0)');
    g.fillStyle = grd; g.beginPath(); g.ellipse(x, y, 60, 12, 0, 0, 6.3); g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 4;
  return t;
}

// Soft dark blob used as a cheap contact shadow / ambient occlusion under
// objects that stand on the bench.
function blobTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(64, 64, 8, 64, 64, 64);
  grd.addColorStop(0, 'rgba(0,0,0,0.55)');
  grd.addColorStop(0.55, 'rgba(0,0,0,0.25)');
  grd.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = grd; g.fillRect(0, 0, 128, 128);
  const t = new THREE.CanvasTexture(c);
  return t;
}

export class Stage {
  constructor(track) {
    this.track = track;
    this.group = new THREE.Group();
    this.group.name = 'stage';

    // Key light: the only shadow caster.
    const key = new THREE.DirectionalLight(0xfff4e6, 2.2);
    key.position.set(0.55, -0.75, 1.5);
    key.target.position.set(-0.1, 0, 0.1);
    key.castShadow = true;
    const cam = key.shadow.camera;
    cam.left = -0.85; cam.right = 0.85; cam.top = 0.7; cam.bottom = -0.7; cam.near = 0.5; cam.far = 3.2;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.bias = -0.0002;
    key.shadow.normalBias = 0.0012;
    key.shadow.radius = 4;
    this.key = key;
    this.group.add(key, key.target);
    // Cool rim light from behind, no shadows.
    const rim = new THREE.DirectionalLight(0xcfe0ff, 0.6);
    rim.position.set(-1.0, 0.8, 0.9);
    this.group.add(rim);
    this.hemi = new THREE.HemisphereLight(0xeaf0ff, 0x3a3228, 0.25);
    this.group.add(this.hemi);

    // Bench: plywood slab with a darker edge, anti-static mat under the bay.
    const wood = track(plywoodTexture());
    wood.repeat.set(1.6, 1.1);
    this.benchMat = track(new THREE.MeshStandardMaterial({ map: wood, roughness: 0.72, metalness: 0 }));
    const edgeMat = track(new THREE.MeshStandardMaterial({ color: '#b89a72', roughness: 0.8 }));
    const W = 1.9, D = 1.34, T = 0.036;
    const top = new THREE.Mesh(track(new THREE.PlaneGeometry(W, D)), this.benchMat);
    top.position.set(-0.17, 0.04, 0);
    top.receiveShadow = true;
    const slab = new THREE.Mesh(track(G.rbox(W, D, T, 0.006, 3)), edgeMat);
    slab.position.set(-0.17, 0.04, -T / 2 - 0.0002);
    slab.receiveShadow = true;
    this.matMat = track(new THREE.MeshStandardMaterial({ color: '#4d6a73', roughness: 0.92, metalness: 0 }));
    const mat = new THREE.Mesh(track(G.rbox(0.5, 0.5, 0.0016, 0.006, 2)), this.matMat);
    mat.position.set(-0.335, -0.02, -0.0002);
    mat.receiveShadow = true;
    this.edgeMat = edgeMat;
    this.bench = new THREE.Group();
    this.bench.add(slab, top, mat);
    this.group.add(this.bench);

    this.blobTex = track(blobTexture());
    this.blobMat = track(new THREE.MeshBasicMaterial({ map: this.blobTex, transparent: true, depthWrite: false, opacity: 0.8 }));
    this.dark = null;
  }

  // Contact shadow under something standing on the bench.
  blob(x, y, sx, sy, z = 0.0004) {
    const m = new THREE.Mesh(this.track(new THREE.PlaneGeometry(sx, sy)), this.blobMat);
    m.position.set(x, y, z);
    m.renderOrder = -1;
    this.group.add(m);
    return m;
  }

  // Configure the renderer and scene. Lights the host page added are
  // switched off while the stage is attached (restored by detach()).
  attach(renderer, scene, root) {
    if (this.renderer === renderer && this.scene === scene) return;
    this.detach();
    this.renderer = renderer; this.scene = scene;
    this.saved = {
      toneMapping: renderer.toneMapping, exposure: renderer.toneMappingExposure,
      shadow: renderer.shadowMap.enabled, shadowType: renderer.shadowMap.type,
      env: scene.environment, envI: scene.environmentIntensity, hidden: [],
    };
    renderer.toneMapping = THREE.AgXToneMapping;
    renderer.toneMappingExposure = 1.25;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    scene.environment = environmentFor(renderer);
    scene.environmentIntensity = 0.75;
    for (const o of scene.children) {
      if (o === root || !o.visible) continue;
      const isBench = o.isMesh && o.geometry?.type === 'PlaneGeometry';
      if (o.isLight || o.type === 'GridHelper' || isBench) { o.visible = false; this.saved.hidden.push(o); }
    }
    scene.traverse((o) => { if (o.material) o.material.needsUpdate = true; });
    this.updateTheme(scene);
  }

  detach() {
    const s = this.saved, r = this.renderer, sc = this.scene;
    if (!s || !r) return;
    r.toneMapping = s.toneMapping; r.toneMappingExposure = s.exposure;
    r.shadowMap.enabled = s.shadow; r.shadowMap.type = s.shadowType;
    sc.environment = s.env; sc.environmentIntensity = s.envI;
    for (const o of s.hidden) o.visible = true;
    this.saved = null; this.renderer = null; this.scene = null;
  }

  // Follow the page theme through the scene background the host sets.
  updateTheme(scene = this.scene) {
    const bg = scene?.background;
    if (!bg || !bg.isColor) return;
    const hsl = {};
    bg.getHSL(hsl, THREE.SRGBColorSpace);
    const dark = hsl.l < 0.45;
    if (dark === this.dark) return;
    this.dark = dark;
    this.benchMat.color.set(dark ? '#8f8579' : '#ffffff');
    this.edgeMat.color.set(dark ? '#6d5c49' : '#b89a72');
    this.matMat.color.set(dark ? '#34474e' : '#4d6a73');
    this.hemi.intensity = dark ? 0.2 : 0.3;
    this.key.intensity = dark ? 2.0 : 2.3;
  }
}
