// Seeded pseudo-random numbers. Every random effect in the simulation
// (manufacturing tolerances, contact bounce, sensor noise) draws from a seeded
// stream so that a run can be repeated exactly.

export function makeRng(seed = 1) {
  let s = seed >>> 0;
  const next = () => {
    // mulberry32
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  let spare = null;
  const gauss = () => {
    if (spare !== null) { const v = spare; spare = null; return v; }
    let u = 0, v = 0;
    while (u === 0) u = next();
    v = next();
    const r = Math.sqrt(-2 * Math.log(u));
    spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  };
  return {
    next,
    uniform: (a, b) => a + (b - a) * next(),
    gauss: (mean = 0, sd = 1) => mean + sd * gauss(),
    fork: (salt) => makeRng((seed * 2654435761 + salt * 40503) >>> 0),
  };
}
