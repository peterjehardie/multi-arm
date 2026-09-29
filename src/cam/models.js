// Demo part: a fluted, dome-topped control knob, built as a triangle mesh
// by revolving a profile. Sized for the demo machine (about 20 mm across).
// Dimensions in millimetres.

export function knobMesh({ R = 10, wallH = 4.5, H = 8, flutes = 10, fluteDepth = 0.06, seg = 120, dome = 24 } = {}) {
  // Profile (r, z) from the axis at the bottom, out, up the wall, over the
  // spherical dome, back to the axis at the top.
  const z0 = (R * R + wallH * wallH - H * H) / (2 * (wallH - H)); // sphere centre on the axis
  const rs = H - z0;
  const prof = [[0, 0], [R, 0], [R, wallH]];
  const a0 = Math.atan2(wallH - z0, R);
  for (let k = 1; k <= dome; k++) {
    const a = a0 + ((Math.PI / 2 - a0) * k) / dome;
    prof.push([k === dome ? 0 : rs * Math.cos(a), z0 + rs * Math.sin(a)]);
  }
  const rim = (th) => 1 + fluteDepth * Math.cos(flutes * th); // grip flutes
  const pt = ([r, z], th) => [r * rim(th) * Math.cos(th), r * rim(th) * Math.sin(th), z];
  const out = [];
  for (let s = 0; s + 1 < prof.length; s++) {
    for (let j = 0; j < seg; j++) {
      const t0 = (2 * Math.PI * j) / seg, t1 = (2 * Math.PI * (j + 1)) / seg;
      const a = pt(prof[s], t0), b = pt(prof[s], t1), c = pt(prof[s + 1], t1), d = pt(prof[s + 1], t0);
      // Outward-facing winding (counter-clockwise seen from outside).
      if (prof[s][0] > 1e-9) out.push(...a, ...b, ...c);
      if (prof[s + 1][0] > 1e-9) out.push(...a, ...c, ...d);
    }
  }
  // The bottom segment runs from the axis outward, so its winding faces up;
  // flip it so the base faces down.
  const tris = Float32Array.from(out);
  const nBase = seg; // first profile segment produced one triangle per step
  for (let t = 0; t < nBase; t++) {
    const p = t * 9;
    for (let k = 0; k < 3; k++) { const tmp = tris[p + 3 + k]; tris[p + 3 + k] = tris[p + 6 + k]; tris[p + 6 + k] = tmp; }
  }
  return tris;
}
