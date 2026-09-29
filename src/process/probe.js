// Touch probe tool: a stylus on a spring-loaded, kinematic seat. Pressing
// the stylus tip against anything lifts the seat and opens or closes a
// contact (here: the contact closes on touch). Wired through the tool
// changer's pogo pins to a controller input, it lets the firmware measure
// where surfaces really are: the part, the bed, the plate. That is how a
// machine finds its own errors (bed height, link lengths, part position).
//
// Physics: the ProbeContact connection measures how far the stylus tip has
// been pushed below the surface under it (the workpiece height map, or the
// bare plate). Beyond the pre-travel (the few hundredths of a millimetre the
// seat needs to lift) the contact closes.

import { Component, Connection } from '../core/graph.js';
import { Contact } from '../electrical/circuit.js';

export class ProbeSwitch extends Component {
  static PARAMS = ['pretravel', 'hysteresis'];
  static STATE = ['triggered', 'contact.closed'];
  constructor(id, { pretravel = 0.04e-3, hysteresis = 0.01e-3, stylusAt, ...opts }) {
    super(id, { ...opts, kind: 'probe', size: opts.size ?? [0.05, 0.02, 0.02] });
    this.pretravel = pretravel; this.hysteresis = hysteresis;
    this.triggered = false;
    this.contact = new Contact(0.05, `${id} contact`);
    this.addPort('COM', 'elec', 'sensor');
    this.addPort('NO', 'elec', 'sensor');
    this.addThrough('COM', 'NO', this.contact);
    this.stylus = this.addPort('stylus', 'mat', 'stylus', { required: false, at: stylusAt ?? [0.05, 0, 0] });
  }
  update() {
    const d = this.stylus.contact;
    const on = this.triggered ? d > this.pretravel - this.hysteresis : d > this.pretravel;
    this.triggered = on;
    this.contact.closed = on;
  }
  inspect() { return { triggered: this.triggered, deflection_mm: Math.max(0, this.stylus.contact) * 1e3 }; }
}

// Stylus tip against the workpiece / plate.
export class ProbeContact extends Connection {
  static PARAMS = [];
  static STATE = ['depth'];
  constructor(id, stylusPort, surfacePort, opts = {}) {
    super(id, stylusPort, surfacePort, { ...opts, kind: 'probe-contact' });
    this.work = surfacePort.owner;
    this.depth = 0;
  }
  update() {
    const tip = this.a.worldPos();
    const local = this.work.toLocal(tip);
    const onPlate = Math.hypot(local[0], local[1]) < this.work.plateRadius;
    const surface = onPlate ? this.work.heightAt(local[0], local[1]) : -Infinity;
    this.depth = surface - local[2];
    this.a.contact = Number.isFinite(this.depth) ? this.depth : -1;
  }
  inspect() { return { depth_mm: this.depth * 1e3 }; }
}
