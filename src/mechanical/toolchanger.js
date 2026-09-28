// Tool changer: a latch on the arm's wrist flange and a mating plate on each
// tool.
//
// Mechanically: a small servo turns a cam that pulls the tool plate against
// the flange (locked above ~70 deg of horn travel, released below ~30 deg).
// The tool only locks if its plate is sitting on the flange: within a
// millimetre or so and a few degrees. A locked tool moves with the wrist and
// its mass is added to the arm's last link.
//
// Electrically: spring-loaded pogo pins on the flange press on pads on the
// tool plate. Each pin is a MatingContact connection. Its resistance is a few
// tens of milliohms when mated and infinite otherwise, so a heater on a
// parked tool is simply an open circuit. Every circuit through the changer is
// re-traced after each lock or release.

import { Component, Connection } from '../core/graph.js';
import { tf, tfMul, tfInv, norm, sub, m3mulv, dot } from '../core/linalg.js';

export class ChangerMaster extends Component {
  constructor(id, { pins = 8, flange, ...opts }) {
    super(id, { ...opts, kind: 'changer-master', size: [0.012, 0.05, 0.05] });
    this.flange = flange; // [x,y,z] offset of the mating face on the link
    for (let i = 1; i <= pins; i++)
      this.addPort(`P${i}`, 'elec', 'passthru', { required: false, at: [0.006, 0.02 - i * 0.005, 0.02] });
    this.passive = false;
    this.latch = this.addPort('latch', 'rot', 'sensor');
    this.locked = false;
    this.tool = null;
  }
  inspect() { return { locked: this.locked, tool: this.tool?.id ?? 'none', latch_deg: ((this.latch.theta ?? 0) * 180) / Math.PI }; }
}

export class ToolPlate extends Component {
  constructor(id, { pins = 8, toolName, holder, ...opts }) {
    super(id, { ...opts, kind: 'tool-plate', size: [0.008, 0.05, 0.05] });
    this.toolName = toolName;
    this.holder = holder; // world transform of the mating face when parked
    for (let i = 1; i <= pins; i++)
      this.addPort(`P${i}`, 'elec', 'passthru', { required: false, at: [-0.004, 0.02 - i * 0.005, 0.02] });
    this.state = 'parked';
  }
  inspect() { return { state: this.state }; }
}

export class MatingContact extends Connection {
  constructor(id, a, b, { Ron = 0.025, ratedA = 3, ...opts } = {}) {
    super(id, a, b, { ...opts, kind: 'pogo-contact' });
    this.Ron = Ron; this.ratedA = ratedA;
    this.mated = false; this.i = 0; this.length = 0.002;
    this.overload = 0;
  }
  resistance() { return this.mated ? this.Ron : Infinity; }
  capacitance() { return 1e-12; }
  inductance() { return 2e-9; }
  record(i) { this.i = i; }
  heat(dt) { if (Math.abs(this.i) > this.ratedA) this.overload += dt; }
  inspect() { return { mated: this.mated, R_ohm: this.resistance(), I_A: this.i, overload_s: this.overload }; }
}

export class ToolChanger {
  constructor({ master, plates, arm, linkBody, onChange, log }) {
    this.master = master; this.plates = plates; this.arm = arm;
    this.linkBody = linkBody; this.onChange = onChange; this.log = log;
    for (const p of plates) p.mount.body.T = p.holder;
  }
  flangeT() { return tfMul(this.linkBody.T, tf(undefined, this.master.flange)); }

  update(t) {
    const ang = this.master.latch.theta ?? 0;
    const wantLock = this.master.locked ? ang > 0.52 : ang > 1.22;
    const F = this.flangeT();
    if (wantLock && !this.master.locked) {
      this.master.locked = true;
      // Which tool plate is sitting on the flange?
      for (const p of this.plates) {
        const T = p.mount.body.T;
        const d = norm(sub(T.p, F.p));
        const ax = dot(m3mulv(T.R, [1, 0, 0]), m3mulv(F.R, [1, 0, 0]));
        const ay = dot(m3mulv(T.R, [0, 1, 0]), m3mulv(F.R, [0, 1, 0]));
        if (d < 1.5e-3 && ax > 0.9986 && ay > 0.9986) { this.attach(p, t); break; }
      }
      if (!this.master.tool) this.log?.(t, 'changer locked with no tool seated');
    } else if (!wantLock && this.master.locked) {
      this.master.locked = false;
      if (this.master.tool) this.detach(this.master.tool, t);
    }
    if (this.master.tool) this.master.tool.mount.body.T = F;
  }

  attach(plate, t) {
    this.master.tool = plate;
    plate.state = 'attached';
    for (const c of this.contactsOf(plate)) c.mated = true;
    this.log?.(t, `tool ${plate.toolName} locked on`);
    this.onChange?.();
  }
  detach(plate, t) {
    this.master.tool = null;
    for (const c of this.contactsOf(plate)) c.mated = false;
    const d = norm(sub(plate.mount.body.T.p, plate.holder.p));
    if (d < 3e-3) { plate.state = 'parked'; plate.mount.body.T = plate.holder; }
    else { plate.state = 'dropped'; this.log?.(t, `tool ${plate.toolName} released ${(d * 1e3).toFixed(1)} mm from its holder: dropped`); }
    this.log?.(t, `tool ${plate.toolName} released`);
    this.onChange?.();
  }
  contactsOf(plate) {
    const out = [];
    for (const p of plate.ports.values()) for (const c of p.connections) if (c instanceof MatingContact) out.push(c);
    return out;
  }
}
