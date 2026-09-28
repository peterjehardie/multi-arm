// Process interfaces: where a tool meets the workpiece.
//
// These are connections like any other. The "route" is the gap between the
// tool tip and the part surface. When they meet, material and force move
// across: plastic onto the part, chips off it, and the cutting force pushes
// back on the tool (and so on the arm and its gearboxes) while the equal and
// opposite force pushes on the part (and so on the turntable).

import { Connection } from '../core/graph.js';
import { sub, scale, norm, cross, m3mulv } from '../core/linalg.js';
import { MATERIAL_BY_ID, MATERIALS } from './workpiece.js';

function applyReaction(workpiece, x, F) {
  workpiece.body.applyForce?.(x, F);
}

export class DepositionContact extends Connection {
  constructor(id, nozzlePort, surfacePort, opts = {}) {
    super(id, nozzlePort, surfacePort, { ...opts, kind: 'deposition' });
    this.extruder = nozzlePort.owner;
    this.work = surfacePort.owner;
    this.prev = null;
    this.contactForce = 0;
  }
  update(dt) {
    const ex = this.extruder, wp = this.work;
    const tip = this.a.worldPos();
    const local = wp.toLocal(tip);
    const prev = this.prev ?? local;
    if (ex.volumeOut > 0) {
      wp.deposit(prev, local, ex.volumeOut, ex.nozzleR * 1.1, MATERIALS.pla);
      ex.volumeOut = 0;
    }
    // Nozzle pressed into the part: stiff contact pushes it back up.
    const pen = wp.heightAt(local[0], local[1]) - local[2];
    this.contactForce = 0;
    if (pen > 0.05e-3 && local[2] > -0.01) {
      const Fz = 2e4 * (pen - 0.05e-3);
      const F = wp.toWorldDir([0, 0, Fz]);
      this.contactForce = Fz;
      this.a.owner.mount.body.applyForce?.(tip, F);
      applyReaction(wp, tip, scale(F, -1));
    }
    this.prev = local;
  }
  inspect() { return { contact_N: this.contactForce }; }
}

// Flat end mill in a spindle. Cutting power = specific cutting energy x
// material removal rate. Torque on the spindle = power / speed. Force on the
// cutter edge = torque / radius, acting against the feed direction. A cutter
// that is not turning cannot cut: it rams the part instead.
export class CuttingContact extends Connection {
  constructor(id, bitPort, surfacePort, { radius = 1.5875e-3, flutes = 2, ...opts } = {}) {
    super(id, bitPort, surfacePort, { ...opts, kind: 'cutting' });
    this.spindle = bitPort.owner;
    this.work = surfacePort.owner;
    this.R = radius; this.flutes = flutes;
    this.prev = null;
    this.P = 0; this.Pf = 0; this.F = [0, 0, 0]; this.mrr = 0; this.chipLoad = 0;
    this.crash = 0;
  }
  update(dt) {
    const wp = this.work, sp = this.spindle;
    const tip = this.a.worldPos();
    const local = wp.toLocal(tip);
    const prev = this.prev ?? local;
    const vLocal = scale(sub(local, prev), 1 / dt);
    this.prev = local;
    const w = sp.omega;
    let F = [0, 0, 0];
    if (w > 30) {
      const { removed, byMat } = wp.cut(local, this.R);
      let E = 0;
      for (let m = 0; m < byMat.length; m++) E += byMat[m] * MATERIAL_BY_ID[m].cutEnergy;
      this.mrr = removed / dt;
      this.P = E / dt;
      this.Pf += (this.P - this.Pf) * (1 - Math.exp(-dt / 0.003));
      const tau = this.Pf / w;
      this.a.tauLoad = tau;
      const Ft = tau / this.R;
      const vh = [vLocal[0], vLocal[1], 0];
      const s = norm(vh);
      if (s > 1e-5 && Ft > 0) {
        const d = scale(vh, 1 / s);
        const side = cross([0, 0, 1], d);
        // Resultant: mostly against the feed, partly sideways (climb vs conventional).
        F = wp.toWorldDir([-0.8 * Ft * d[0] + 0.45 * Ft * side[0], -0.8 * Ft * d[1] + 0.45 * Ft * side[1], 0.15 * Ft]);
      }
      this.chipLoad = s / ((w / (2 * Math.PI)) * this.flutes);
      this.crash = 0;
    } else {
      this.P = 0; this.Pf *= 0.9; this.mrr = 0;
      const pen = wp.heightAt(local[0], local[1]) - local[2];
      if (pen > 0) {
        this.crash = pen;
        F = wp.toWorldDir([-vLocal[0] * 50, -vLocal[1] * 50, 5e4 * pen]);
      } else this.crash = 0;
    }
    this.F = F;
    if (F[0] || F[1] || F[2]) {
      this.a.owner.mount.body.applyForce?.(tip, F);
      applyReaction(wp, tip, scale(F, -1));
    }
  }
  inspect() {
    return {
      power_W: this.Pf, MRR_mm3s: this.mrr * 1e9, force_N: norm(this.F),
      chipload_mm: this.chipLoad * 1e3, crash_mm: this.crash * 1e3,
    };
  }
}
