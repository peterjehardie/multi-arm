// The physical-anchor graph.
//
// Principle: nothing in the simulation interacts except through something
// that exists physically. Concretely:
//   * A Component is a physical object (motor, driver board, heater block).
//     It is mounted on a Body at a position, so it has a place in the world.
//   * A Component exposes Ports: terminals, shafts, thermal faces. A port is a
//     physical point on the component.
//   * A Connection joins exactly two ports and is itself a physical object:
//     a wire, a gearbox, a thermal contact face, a cam that presses a switch.
//     Every connection has a route (waypoints fixed to bodies), so it has a
//     length and can be drawn.
//   * Components never hold references to other components. They only read
//     and write their own ports. The only way energy or information moves is
//     through a Connection.
//
// Assembly.validate() checks these rules and reports every violation (for
// example a floating input pin, just like a real board).

import { tf, tfMul, tfApply, I3 } from './linalg.js';
import { AMBIENT } from './units.js';

export const DOMAINS = {
  elec: 'electrical',    // across: voltage [V], through: current [A]
  rot: 'rotational',     // across: angle/angular velocity, through: torque [N m]
  therm: 'thermal',      // across: temperature [K], through: heat flow [W]
  mat: 'material',       // material transfer: filament, deposited or removed volume
};

// The values each kind of port carries. Every port of a domain has exactly
// these fields (a native port would give each domain one struct).
//   rot:   theta [rad], omega [rad/s], tau [N m] torque applied to the owner
//          this step (accumulated by connections), loadTau [N m] a load the
//          owner reports for a kinematic link (gear mesh output)
//   therm: T [K], q [W] heat flowing into the owner this step
//   mat:   omega [rad/s] of a cutter, tauLoad [N m] cutting torque on it,
//          contact [m] how far a stylus is pushed into the surface
//   elec:  none; voltages live in the supply network, currents in loops
export const PORT_FIELDS = {
  elec: [],
  rot: ['theta', 'omega', 'tau', 'loadTau'],
  therm: ['T', 'q'],
  mat: ['omega', 'tauLoad', 'contact'],
};
const PORT_INIT = { theta: 0, omega: 0, tau: 0, loadTau: 0, T: AMBIENT, q: 0, tauLoad: 0, contact: -1 };

export class Body {
  // A rigid body with a world pose. Owners (arm model, turntable) update T.
  constructor(name, T = tf()) {
    this.name = name;
    this.T = T;
  }
}

export class Port {
  constructor(owner, name, domain, role, opts = {}) {
    if (!DOMAINS[domain]) throw new Error(`Unknown domain ${domain}`);
    this.owner = owner;
    this.name = name;
    this.domain = domain;
    this.role = role;
    // Ports on the same component that share `node` are the same piece of
    // copper (e.g. a driver board's logic GND pin and its motor GND pin).
    this.node = opts.node ?? name;
    this.required = opts.required ?? true;
    this.at = opts.at ?? [0, 0, 0]; // terminal location in the component frame
    this.label = opts.label ?? name;
    this.connections = [];
    for (const f of PORT_FIELDS[domain]) this[f] = PORT_INIT[f];
  }
  get id() { return `${this.owner.id}.${this.name}`; }
  get connection() { return this.connections[0] ?? null; }
  worldPos() { return this.owner.worldPoint(this.at); }
}

export class Component {
  constructor(id, { mount, label, kind, size } = {}) {
    this.id = id;
    this.label = label ?? id;
    this.kind = kind ?? this.constructor.name;
    this.mount = mount ?? null; // { body: Body, p: [x,y,z], R?: 3x3 }
    this.size = size ?? [0.02, 0.02, 0.02]; // bounding box for drawing, metres
    this.mass = 0; // kg; set by builders. Arm inertia is summed from mounted components.
    this.ports = new Map();
    // Internal series elements joining two of this component's ports, used by
    // circuit tracing (a heater's resistance between its two terminals).
    this.through = new Map();
  }
  addPort(name, domain, role, opts) {
    const p = new Port(this, name, domain, role, opts);
    this.ports.set(name, p);
    return p;
  }
  port(name) {
    const p = this.ports.get(name);
    if (!p) throw new Error(`${this.id} has no port ${name}`);
    return p;
  }
  // Declare that current entering portA flows through `element` and leaves
  // at portB. element must implement resistance() and may implement
  // inductance(), emf() and record(i).
  addThrough(a, b, element) {
    this.through.set(a, { other: b, element, sign: +1 });
    this.through.set(b, { other: a, element, sign: -1 });
  }
  worldTransform() {
    if (!this.mount) return null;
    const { body, p = [0, 0, 0], R = I3() } = this.mount;
    return tfMul(body.T, tf(R, p));
  }
  worldPoint(local) {
    const T = this.worldTransform();
    return T ? tfApply(T, local) : null;
  }
  // Snapshot of internal state for inspectors / logging.
  inspect() { return {}; }
  // Declarations (overridden per class as static fields):
  //   static PARAMS: fixed parameters (what a data sheet or drawing gives)
  //   static STATE:  values that change as the simulation runs. Paths may
  //                  reach into owned objects ('loopA.i'); 'x[]' is an array;
  //                  'mode:cv|cc|off' is a named state stored as its index.
  static PARAMS = [];
  static STATE = [];
}

export class Connection {
  constructor(id, a, b, { route = [], kind, label } = {}) {
    if (a.domain !== b.domain)
      throw new Error(`Connection ${id}: domain mismatch ${a.id}(${a.domain}) vs ${b.id}(${b.domain})`);
    this.id = id;
    this.a = a;
    this.b = b;
    this.kind = kind ?? this.constructor.name;
    this.label = label ?? id;
    // Waypoints between the two terminals: [{ body, p }]. The terminals
    // themselves are the first and last points of the path.
    this.route = route;
    a.connections.push(this);
    b.connections.push(this);
  }
  other(port) {
    if (port === this.a) return this.b;
    if (port === this.b) return this.a;
    throw new Error(`${port.id} is not an end of ${this.id}`);
  }
  // Current world-space polyline of the physical path.
  path() {
    const pts = [];
    const pa = this.a.worldPos();
    if (pa) pts.push(pa);
    for (const w of this.route) pts.push(tfApply(w.body.T, w.p));
    const pb = this.b.worldPos();
    if (pb) pts.push(pb);
    return pts;
  }
  pathLength() {
    const pts = this.path();
    let L = 0;
    for (let i = 1; i < pts.length; i++)
      L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2]);
    return L;
  }
  inspect() { return {}; }
  static PARAMS = [];
  static STATE = [];
}

export class Assembly {
  constructor() {
    this.components = new Map();
    this.connections = new Map();
    this.bodies = new Map();
  }
  body(name, T) {
    if (!this.bodies.has(name)) this.bodies.set(name, new Body(name, T));
    return this.bodies.get(name);
  }
  add(c) {
    if (this.components.has(c.id)) throw new Error(`Duplicate component id ${c.id}`);
    this.components.set(c.id, c);
    return c;
  }
  connect(conn) {
    if (this.connections.has(conn.id)) throw new Error(`Duplicate connection id ${conn.id}`);
    this.connections.set(conn.id, conn);
    return conn;
  }
  get(id) {
    const c = this.components.get(id);
    if (!c) throw new Error(`No component ${id}`);
    return c;
  }
  // Resolve "component.port" to a Port.
  p(ref) {
    const i = ref.lastIndexOf('.');
    return this.get(ref.slice(0, i)).port(ref.slice(i + 1));
  }

  validate() {
    const problems = [];
    for (const c of this.components.values()) {
      if (!c.mount) problems.push(`${c.id}: not mounted anywhere (no physical location)`);
      for (const p of c.ports.values()) {
        if (p.required && p.connections.length === 0)
          problems.push(`${p.id}: required ${DOMAINS[p.domain]} port is not connected (floating)`);
      }
    }
    for (const w of this.connections.values()) {
      if (!w.a.owner.mount || !w.b.owner.mount)
        problems.push(`${w.id}: an end has no physical location`);
    }
    return problems;
  }
}
