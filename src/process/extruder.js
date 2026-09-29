// Direct-drive extruder and hot end.
//
// Chain of physical steps from motor to plastic on the part:
//   1. The motor pinion turns the extruder's big gear (ratio N). The drive
//      gear on that shaft grips the filament: filament feed = gear angle x r.
//   2. Between the drive gear and the melt zone the solid filament is a
//      spring (Young's modulus x area / free length). Pushing compresses it;
//      the compression force is the melt pressure times the filament area.
//   3. Molten plastic flows out of the nozzle at a rate set by that pressure
//      and by the melt's viscosity. Plastic melt is shear-thinning (a
//      power-law fluid) and far stiffer when colder (Arrhenius). Too cold and
//      it will not flow at all.
//   4. Melting needs heat: the plastic takes heat from the heater block at
//      rate  mass flow x (cp dT + latent heat). High flow cools the melt,
//      which raises viscosity and pressure, which is how a hot end reaches
//      its flow limit.
//   5. If the force exceeds what the drive gear teeth can grip, the gear
//      slips on the filament (the "clicking" extruder).
// Pressure lagging behind the motor (step 2 + 3) is why slicers use
// "pressure advance", and why a nozzle keeps oozing after the motor stops.

import { Component, Connection } from '../core/graph.js';
import { AMBIENT } from '../core/units.js';
import { thermalPort } from '../thermal/thermal.js';

export const PLA_MELT = {
  density: 1240, cp: 1800, latent: 20e3,
  Fref: 18, Qref: 5e-9, Tref: 273.15 + 210, n: 0.38, Ea: 80e3, Tsolid: 273.15 + 165,
};

export class Extruder extends Component {
  static PARAMS = ['r', 'A', 'nozzleR', 'k', 'grip', 'Cmelt', 'meltG', 'm'];
  static STATE = ['xSlip', 'xMelt', 'F', 'Q', 'Tmelt', 'volumeOut', 'totalOut'];
  constructor(id, { melt = PLA_MELT, gearRadius = 3.65e-3, filamentD = 1.75e-3, nozzleD = 0.4e-3,
    freeLength = 0.03, gripForce = 60, meltG = 0.35, ...opts } = {}) {
    super(id, { ...opts, kind: 'extruder', size: [0.03, 0.04, 0.05] });
    this.m = melt;
    this.r = gearRadius;
    this.A = (Math.PI * filamentD * filamentD) / 4;
    this.nozzleR = nozzleD / 2;
    this.k = (3.5e9 * this.A) / freeLength;
    this.grip = gripForce;
    this.xSlip = 0;       // filament length lost to gear slip [m]
    this.xMelt = 0;       // filament length consumed into the melt [m]
    this.F = 0; this.Q = 0;
    this.Tmelt = AMBIENT; this.Cmelt = 0.25; this.meltG = meltG;
    this.volumeOut = 0;   // accumulated extruded volume not yet deposited [m^3]
    this.totalOut = 0;
    this.drive = this.addPort('drive', 'rot', 'shaft');
    this.drive.theta = 0; this.drive.omega = 0; this.drive.loadTau = 0;
    this.melt = thermalPort(this, 'melt');
    this.nozzle = this.addPort('nozzle', 'mat', 'nozzle', { at: opts.nozzleAt ?? [0, 0, -0.05] });
  }

  viscosityFactor(T) {
    const f = Math.exp((this.m.Ea / 8.314) * (1 / T - 1 / this.m.Tref));
    return T < this.m.Tsolid ? f * 1e4 : f;
  }

  // Fast step: filament force and melt flow.
  update(dt) {
    const xFeed = this.drive.theta * this.r - this.xSlip;
    let F = this.k * (xFeed - this.xMelt);
    if (F > this.grip) { this.xSlip += (F - this.grip) / this.k; F = this.grip; }
    if (F < -this.grip) { this.xSlip += (F + this.grip) / this.k; F = -this.grip; }
    const a = this.viscosityFactor(this.Tmelt);
    const Q = Math.sign(F) * this.m.Qref * Math.pow(Math.abs(F) / (this.m.Fref * a), 1 / this.m.n);
    this.F = F; this.Q = Q;
    this.xMelt += (Q / this.A) * dt;
    if (Q > 0) { this.volumeOut += Q * dt; this.totalOut += Q * dt; }
    this.drive.loadTau = F * this.r;
  }

  // Thermal step: melt zone temperature.
  thermalStep(dt) {
    const mdot = Math.max(0, this.Q) * this.m.density;
    const qIn = this.melt.q;
    this.melt.q = 0;
    const qOut = mdot * (this.m.cp * (this.Tmelt - AMBIENT) + this.m.latent);
    this.Tmelt += (dt * (qIn - qOut)) / this.Cmelt;
    this.melt.T = this.Tmelt;
  }

  inspect() {
    return {
      force_N: this.F, flow_mm3s: this.Q * 1e9, melt_C: this.Tmelt - 273.15,
      slip_mm: this.xSlip * 1e3, extruded_mm3: this.totalOut * 1e9,
    };
  }
}

// Motor pinion meshing with a gear: a rigid kinematic link. Output angle is
// input / N. The load torque at the output comes back to the motor divided by
// N, with mesh losses in whichever direction power flows.
export class GearMesh extends Connection {
  static PARAMS = ['N', 'eff'];
  static STATE = [];
  constructor(id, input, output, { ratio, efficiency = 0.9, ...opts }) {
    super(id, input, output, { ...opts, kind: 'gear-mesh' });
    this.N = ratio; this.eff = efficiency;
  }
  exchange() {
    this.b.theta = this.a.theta / this.N;
    this.b.omega = this.a.omega / this.N;
    const t = this.b.loadTau / this.N;
    const motoring = t * this.a.omega >= 0;
    this.a.tau -= motoring ? t / this.eff : t * this.eff;
  }
}
