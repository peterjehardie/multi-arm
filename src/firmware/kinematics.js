// Arm kinematics as the firmware knows it: closed-form, nominal geometry.
//
// Joint 1 turns the whole arm about the vertical axis. Joints 2-4 all pitch
// in the vertical plane that joint 1 selects, so the rest is planar geometry:
//   shoulder at height H, links L2 and L3, and a wrist-to-tip length Lw
//   (flange plus the current tool's tip offset).
// Angles: q2 is the upper arm above horizontal, q3 and q4 are relative bends,
// phi = q2 + q3 + q4 is the tool axis angle (-90 deg = pointing straight down).

export class ArmKinematics {
  constructor({ baseHeight, shoulderHeight, L2, L3, L4 }) {
    this.H = baseHeight + shoulderHeight;
    this.L2 = L2; this.L3 = L3; this.L4 = L4;
    this.tip = 0; // tool tip offset beyond the mating face
  }
  get Lw() { return this.L4 + this.tip; }

  forward(q) {
    const [q1, q2, q3, q4] = q;
    const a2 = q2, a3 = q2 + q3, phi = q2 + q3 + q4;
    const r = this.L2 * Math.cos(a2) + this.L3 * Math.cos(a3) + this.Lw * Math.cos(phi);
    const z = this.H + this.L2 * Math.sin(a2) + this.L3 * Math.sin(a3) + this.Lw * Math.sin(phi);
    return { p: [r * Math.cos(q1), r * Math.sin(q1), z], phi };
  }

  // Tip position p (world) with tool pitch phi. Elbow-up solution.
  // Returns null if out of reach.
  inverse(p, phi = -Math.PI / 2, q1Hint = 0) {
    let r = Math.hypot(p[0], p[1]);
    let q1 = r > 1e-6 ? Math.atan2(p[1], p[0]) : q1Hint;
    const wr = r - this.Lw * Math.cos(phi);
    const wz = p[2] - this.Lw * Math.sin(phi) - this.H;
    const D = (wr * wr + wz * wz - this.L2 ** 2 - this.L3 ** 2) / (2 * this.L2 * this.L3);
    if (D > 1 || D < -1) return null;
    const q3 = -Math.acos(D);
    const q2 = Math.atan2(wz, wr) - Math.atan2(this.L3 * Math.sin(q3), this.L2 + this.L3 * Math.cos(q3));
    const q4 = phi - q2 - q3;
    return [q1, q2, q3, q4];
  }
}
