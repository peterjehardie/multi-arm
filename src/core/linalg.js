// Small, allocation-light linear algebra for 3-vectors, 3x3 matrices and
// small dense systems. Vectors are plain arrays [x, y, z]; 3x3 matrices are
// row-major arrays of 9 numbers.

export const v3 = (x = 0, y = 0, z = 0) => [x, y, z];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const normalize = (a) => {
  const n = norm(a);
  return n > 0 ? scale(a, 1 / n) : [0, 0, 0];
};

export const I3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const m3mulv = (m, v) => [
  m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
  m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
  m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
];
export const m3mul = (a, b) => {
  const r = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return r;
};
export const m3T = (m) => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
export const m3add = (a, b) => a.map((x, i) => x + b[i]);
export const m3scale = (a, s) => a.map((x) => x * s);

// Rotation matrix about a unit axis by angle (Rodrigues).
export function rotAxis(axis, ang) {
  const [x, y, z] = axis;
  const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}
export const rotZ = (a) => rotAxis([0, 0, 1], a);

// Rigid transform {R, p}: maps local point x to R x + p.
export const tf = (R = I3(), p = [0, 0, 0]) => ({ R, p });
export const tfMul = (a, b) => ({ R: m3mul(a.R, b.R), p: add(m3mulv(a.R, b.p), a.p) });
export const tfApply = (t, x) => add(m3mulv(t.R, x), t.p);
export const tfInv = (t) => {
  const Rt = m3T(t.R);
  return { R: Rt, p: scale(m3mulv(Rt, t.p), -1) };
};

// Skew-symmetric matrix of v (so that skew(v) x = v cross x).
export const skew = (v) => [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];

// Inertia of a point mass m at offset r about the origin (parallel axis term).
export function parallelAxis(m, r) {
  const [x, y, z] = r;
  return [
    m * (y * y + z * z), -m * x * y, -m * x * z,
    -m * x * y, m * (x * x + z * z), -m * y * z,
    -m * x * z, -m * y * z, m * (x * x + y * y),
  ];
}

// Dense LU factorisation with partial pivoting of an n x n matrix stored as
// Float64Array (row major). Returns an object that can solve A x = b many
// times without refactoring.
export function luFactor(A, n) {
  const a = Float64Array.from(A);
  const piv = new Int32Array(n);
  for (let i = 0; i < n; i++) piv[i] = i;
  for (let k = 0; k < n; k++) {
    let p = k, max = Math.abs(a[k * n + k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(a[i * n + k]);
      if (v > max) { max = v; p = i; }
    }
    if (max < 1e-300) throw new Error(`luFactor: singular matrix at column ${k}`);
    if (p !== k) {
      for (let j = 0; j < n; j++) {
        const t = a[k * n + j]; a[k * n + j] = a[p * n + j]; a[p * n + j] = t;
      }
      const t = piv[k]; piv[k] = piv[p]; piv[p] = t;
    }
    const d = a[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const f = (a[i * n + k] /= d);
      if (f !== 0) for (let j = k + 1; j < n; j++) a[i * n + j] -= f * a[k * n + j];
    }
  }
  const y = new Float64Array(n);
  return {
    solve(b, x = new Float64Array(n)) {
      for (let i = 0; i < n; i++) {
        let s = b[piv[i]];
        for (let j = 0; j < i; j++) s -= a[i * n + j] * y[j];
        y[i] = s;
      }
      for (let i = n - 1; i >= 0; i--) {
        let s = y[i];
        for (let j = i + 1; j < n; j++) s -= a[i * n + j] * x[j];
        x[i] = s / a[i * n + i];
      }
      return x;
    },
  };
}

// Cholesky factorisation of a symmetric positive definite n x n matrix
// (array of arrays or flat). Returns lower-triangular L as flat Float64Array.
export function cholesky(M, n) {
  const L = new Float64Array(n * n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = M[i * n + j];
      for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      if (i === j) {
        if (s <= 0) throw new Error('cholesky: matrix not positive definite');
        L[i * n + i] = Math.sqrt(s);
      } else L[i * n + j] = s / L[j * n + j];
    }
  }
  return L;
}
export function cholSolve(L, n, b, x = new Float64Array(n)) {
  for (let i = 0; i < n; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[i * n + k] * x[k];
    x[i] = s / L[i * n + i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = x[i];
    for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k];
    x[i] = s / L[i * n + i];
  }
  return x;
}

export const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
export const DEG = Math.PI / 180;
