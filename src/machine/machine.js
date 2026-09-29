// The running machine: builds the assembly, finds the circuits, and runs the
// physics in a fixed order each step.
//
// Per 25 us step (one chopper period):
//   1. power network: solve all supply node voltages
//   2. logic and regulators: buck, controller board, servo
//   3. drivers and MOSFET modules drive their load circuits (coil currents)
//   4. extruder filament force
//   5. mechanical connections exchange torque (gearboxes, gear mesh)
//   6. rotors, arm, turntable, spindle integrate
// Every 100 us: supply network. Every 500 us: kinematics and arm mass matrix,
// tool changer, limit switches.
// Every 1 ms:   process contacts (deposit / cut), ADC filters, recorder.
// Every 10 ms:  thermal network, wire and motor heating.

import { Simulator } from '../core/sim.js';
import { tfApply, tf, tfMul, sub, norm } from '../core/linalg.js';
import { DCNetwork } from '../electrical/dcnetwork.js';
import { traceLoop, Wire } from '../electrical/circuit.js';
import { buildDigitalNets } from '../electrical/digital.js';
import { ThermalContact } from '../thermal/thermal.js';
import { Gearbox, SensorCoupling } from '../mechanical/transmission.js';
import { ToolChanger, MatingContact } from '../mechanical/toolchanger.js';
import { GearMesh } from '../process/extruder.js';
import { Firmware } from '../firmware/firmware.js';
import { buildMachine, PINS } from './build.js';
import { SPEC } from './spec.js';

const DEG = Math.PI / 180;

export class Machine {
  constructor({ spec = SPEC, dt = 25e-6, preheated = false, startHomed = false } = {}) {
    this.spec = spec;
    this.sim = new Simulator({ dt });
    const m = buildMachine(this.sim, spec);
    Object.assign(this, m);
    this.problems = this.A.validate();

    // Cut every wire to length from its route at the build pose.
    this.arm.fk();
    this.table.updateBody();
    this.wires = [...this.A.connections.values()].filter((c) => c instanceof Wire);
    for (const w of this.wires) w.measure();

    // Power network.
    // The supply network's voltages move on millisecond time scales (bulk
    // capacitors behind wire resistance), so it is solved every 4th step.
    this.netEvery = 4;
    this.net = new DCNetwork(dt * this.netEvery);
    for (const c of this.A.components.values()) c.dcStamp?.(this.net);
    for (const w of this.wires) if (w.a.role === 'supply' && w.b.role === 'supply') this.net.addWire(w);
    this.net.finalize();

    // Signal nets and load circuits.
    this.nets = buildDigitalNets(this.A, this.sim, this.net);
    this.rewire();

    // Mechanics.
    this.gearList = [...this.A.connections.values()].filter((c) => c instanceof Gearbox);
    this.meshes = [...this.A.connections.values()].filter((c) => c instanceof GearMesh);
    this.sensorLinks = [...this.A.connections.values()].filter((c) => c instanceof SensorCoupling);
    this.thermalLinks = [...this.A.connections.values()].filter((c) => c instanceof ThermalContact);
    this.contacts = [...this.A.connections.values()].filter((c) => c instanceof MatingContact);
    this.stepperList = [...Object.values(this.motors), this.emot];
    this.driverList = Object.values(this.drivers);
    this.fetList = Object.values(this.fets);
    this.thermalBodies = [...this.A.components.values()].filter((c) => c.thermalStep);
    this.switchList = Object.values(this.switches);
    for (const gb of this.gearList) gb.align();
    for (const s of this.sensorLinks) s.exchange();

    this.changer = new ToolChanger({
      master: this.master, plates: this.plates, arm: this.arm, linkBody: this.arm.bodies[3],
      log: this.note, onChange: () => this.onToolChange(),
    });
    this.arm.rebuildInertia();

    // External forces from process contacts are held for one process period.
    this.arm.extTauHeld = new Float64Array(this.arm.n);
    for (const p of this.plates) {
      p.mount.body.applyForce = (x, F) => {
        if (this.master.tool === p) {
          const saved = this.arm.extTau; this.arm.extTau = this.arm.extTauHeld;
          this.arm.addForceAt(x, F); this.arm.extTau = saved;
        }
      };
    }
    this.table.extTauHeld = 0;
    this.tableBodyRef = this.workpiece.body;
    this.workpiece.body.applyForce = (x, F) => {
      const c = this.table.center;
      const r = sub(x, c);
      this.table.extTauHeld += r[0] * F[1] - r[1] * F[0];
    };

    if (preheated) this.preheat();

    // Firmware on the controller board.
    this.firmware = new Firmware(this.board.mcu, { ...firmwareConfig(spec), startHomed });
    this.host.link(this.board);

    this.recorder = new Recorder(this);
    this.setupStages();
  }

  // Re-trace every load circuit (after a tool locks or releases).
  rewire() {
    for (const c of this.A.components.values()) c.resolveLoops?.(traceLoop);
  }
  onToolChange() {
    this.rewire();
    const tool = this.master.tool;
    const payload = [];
    if (tool) {
      for (const c of this.A.components.values())
        if (c.mount?.body === tool.mount.body && c.mass > 0)
          payload.push({ m: c.mass, p: [this.flange[0] + c.mount.p[0], c.mount.p[1], c.mount.p[2]], size: c.size });
    }
    this.arm.payload = payload;
    this.arm.rebuildInertia();
  }

  // Start with the hot end and bed at temperature (skips a long heat-up).
  preheat() {
    const K = (c) => c + 273.15;
    for (const [c, T] of [[this.block, 210], [this.hotHeater, 212], [this.hotTherm, 210], [this.bedPlate, 60], [this.bedHeater, 61], [this.bedTherm, 60]]) {
      c.T = K(T);
      for (const p of c.ports.values()) if (p.domain === 'therm') p.T = K(T);
    }
    this.extruder.Tmelt = K(205); this.extruder.melt.T = K(205);
  }

  setupStages() {
    const s = this.sim, net = this.net, arm = this.arm, table = this.table;
    let netCount = 0;
    s.stage('electrical+mechanical', 1, (dt, t) => {
      if (netCount++ % this.netEvery === 0) net.step();
      this.buck.update();
      this.board.update(dt, t);
      this.servo.update(dt);
      for (const d of this.driverList) d.update(dt, t);
      for (const f of this.fetList) f.update(dt, t);
      this.extruder.update(dt);
      for (const g of this.gearList) g.exchange();
      for (const g of this.meshes) g.exchange();
      for (let i = 0; i < arm.n; i++) arm.jointPorts[i].tau += arm.extTauHeld[i];
      table.extTau += table.extTauHeld;
      for (const mtr of this.stepperList) mtr.integrate(dt);
      this.spindle.integrate(dt);
      arm.integrate(dt);
      table.integrate(dt);
    });
    s.stage('kinematics', 20, (dt, t) => {
      arm.fk();
      arm.dynamics();
      table.updateBody();
      this.changer.update(t);
      for (const l of this.sensorLinks) l.exchange();
      for (const sw of this.switchList) sw.update();
    });
    s.stage('process', 40, (dt, t) => {
      arm.extTauHeld.fill(0);
      table.extTauHeld = 0;
      this.deposition.update(dt);
      this.cutting.update(dt);
      this.board.adcUpdate(dt);
      this.recorder.sample(t);
    });
    s.stage('thermal', 400, (dt) => {
      for (const l of this.thermalLinks) l.exchange();
      for (const b of this.thermalBodies) b.thermalStep(dt);
      for (const m of this.stepperList) m.thermal(dt);
      for (const d of this.driverList) d.thermal(dt);
      for (const w of this.wires) w.heat(dt);
      for (const c of this.contacts) c.heat(dt);
      for (const d of this.driverList) { d.loopA?.refresh(); d.loopB?.refresh(); }
      for (const f of this.fetList) f.loop?.refresh();
      net.checkDrift();
    });
    s.stage('slow', 40000, () => {
      const { J } = this.workpiece.massProperties();
      table.Jpart = J;
    });
  }

  run(seconds) { this.sim.advance(seconds); }
  get t() { return this.sim.t; }

  // True tool tip (real geometry, real joint angles incl. deflection).
  toolTip() {
    const tool = this.master.tool;
    if (!tool) return tfApply(this.arm.frames[3], this.flange);
    const T = tool.mount.body.T;
    const off = tool.toolName === 'hotend' ? this.spec.tools.hotend.tipOffset : this.spec.tools.spindle.tipOffset;
    return tfApply(T, [off, 0, 0]);
  }
  // Where the firmware believes the tip is (nominal geometry, its step counts).
  firmwareTip() {
    const fw = this.firmware, j = fw.jointAngles();
    return fw.kin.forward([j.j1, j.j2, j.j3, j.j4]).p;
  }
  // Only meaningful when firmware and machine agree which tool is on.
  tipError() {
    const fw = this.firmware, tool = this.master.tool?.toolName ?? null;
    if (!fw.homed || fw.tool !== tool) return NaN;
    const expect = tool ? this.spec.tools[tool].tipOffset : 0;
    if (Math.abs(fw.kin.tip - expect) > 1e-9) return NaN;
    return norm(sub(this.toolTip(), this.firmwareTip()));
  }

  // Error where it matters for the part: the true tool tip seen from the
  // plate's true frame, against where the firmware believes the tip is on the
  // plate (its own kinematics and its own idea of the plate angle). Includes
  // lost plate steps, which the arm-only tip error cannot see.
  partError() {
    const fw = this.firmware, tool = this.master.tool?.toolName ?? null;
    if (!Number.isFinite(this.tipError())) return NaN;
    const trueLocal = this.workpiece.toLocal(this.toolTip());
    const fwLocal = fw.worldToWork(this.firmwareTip(), fw.jointAngles().table ?? 0);
    void tool;
    return norm(sub(trueLocal, fwLocal));
  }

  status() {
    const K = (T) => T - 273.15;
    const fw = this.firmware;
    return {
      t: this.t,
      V_bus: this.net.across(this.psu.port('V+'), this.psu.port('V-')),
      I_psu: this.psu.I,
      hotend_C: K(this.hotTherm.T), block_C: K(this.block.T), bed_C: K(this.bedPlate.T),
      fw_hotend: fw.heaters.hotend.temp, fw_bed: fw.heaters.bed.temp,
      q_deg: Array.from(this.arm.q, (x) => x / DEG),
      table_deg: this.table.theta / DEG,
      tool: this.master.tool?.toolName ?? 'none',
      tipError_mm: this.tipError() * 1e3,
      partError_mm: this.partError() * 1e3,
      spindle_rpm: this.spindle.omega * 60 / (2 * Math.PI),
      homed: fw.homed,
      queue: fw.queue.length + (this.host.lines.length - this.host.sent),
    };
  }
}

// Firmware configuration derived from the nominal design (never the truth).
export function firmwareConfig(spec) {
  const spr = (n) => (spec.microsteps * 200 * spec.drives[n].ratio) / (2 * Math.PI);
  const axes = {};
  for (const n of ['j1', 'j2', 'j3', 'j4', 'table']) {
    axes[n] = {
      step: PINS[n].step, dir: PINS[n].dir, spr: spr(n),
      home: { pin: PINS[n].home, trip: spec.homing[n].trip, dir: spec.homing[n].dir },
    };
  }
  // Extruder: "angle" unit is metres of filament: steps per metre.
  axes.e = { step: PINS.e.step, dir: PINS.e.dir, spr: (spec.microsteps * 200 * spec.drives.e.ratio) / (2 * Math.PI * 3.65e-3) };
  return {
    geometry: spec.arm,
    axes,
    enPin: PINS.en,
    holdAtBoot: true,
    table: { center: spec.table.center },
    limits: spec.limits,
    maxJointSpeed: { ...spec.maxJointSpeed, e: 1e9 },
    maxJointAccel: spec.maxJointAccel,
    maxJointJump: spec.maxJointJump,
    homingOrder: spec.homing.order,
    restPose: spec.restPose,
    accel: 1.5,                 // m/s^2 along the tool path (hobby-printer range)
    junctionDeviation: 0.05e-3,
    rapid: 2400,
    changeFeed: 2400,
    rack: spec.rack,
    toolOrder: ['hotend', 'spindle'],
    tipOffset: { hotend: spec.tools.hotend.tipOffset, spindle: spec.tools.spindle.tipOffset },
    heaters: {
      hotend: { pin: PINS.hotend, adc: 'TH0', tool: 'hotend', pid: { kp: 0.08, ki: 0.006, kd: 0.3 }, band: 12, iMax: 120, max: 285, pwmHz: 10, runawayTime: 40 },
      bed: { pin: PINS.bed, adc: 'TH1', tool: null, pid: { kp: 0.35, ki: 0.004, kd: 1.5 }, band: 6, iMax: 200, max: 120, pwmHz: 5, runawayTime: 120 },
    },
    spindle: { pin: PINS.spindle, maxRpm: 9000, pwmHz: 1000 },
    servo: { pin: PINS.servo, lockUs: 1850, unlockUs: 1050 },
  };
}

// Ring-buffer recorder of named signals at 1 kHz for plots and reports.
export class Recorder {
  constructor(machine, seconds = 20) {
    this.m = machine;
    this.n = Math.round(seconds * 1000);
    this.t = new Float64Array(this.n);
    this.idx = 0; this.count = 0;
    this.channels = new Map();
    const m = machine;
    const add = (name, unit, fn) => this.add(name, unit, fn);
    add('24 V bus at PSU', 'V', () => m.net.across(m.psu.port('V+'), m.psu.port('V-')));
    add('PSU current', 'A', () => m.psu.I);
    add('Driver j2 VM', 'V', () => m.net.across(m.drivers.j2.port('VM'), m.drivers.j2.port('GND')));
    add('Ground offset board-j2', 'mV', () => 1e3 * (m.net.voltage(m.board.port('GND1')) - m.net.voltage(m.drivers.j2.port('GNDL'))));
    for (const n of ['j1', 'j2', 'j3', 'j4']) add(`${n} coil A`, 'A', () => m.drivers[n].loopA?.i ?? 0);
    add('e coil A', 'A', () => m.drivers.e.loopA?.i ?? 0);
    for (let i = 0; i < 4; i++) add(`q${i + 1}`, 'deg', () => m.arm.q[i] / DEG);
    for (const n of ['j2', 'j3']) add(`${n} gearbox wind-up`, 'arcmin', () => m.gearboxes[n].deflection * 60 / DEG);
    add('table angle', 'deg', () => m.table.theta / DEG);
    add('tip error', 'mm', () => m.tipError() * 1e3);
    add('part-frame error', 'mm', () => m.partError() * 1e3);
    add('table angle error', 'deg', () => (m.firmware.homed ? (m.table.theta - m.firmware.jointAngles().table) / DEG : NaN));
    add('hot end (true)', 'C', () => m.hotTherm.T - 273.15);
    add('hot end (firmware)', 'C', () => m.firmware.heaters.hotend.temp);
    add('hot end duty', '%', () => 100 * m.firmware.heaters.hotend.duty);
    add('bed (true)', 'C', () => m.bedPlate.T - 273.15);
    add('extrusion force', 'N', () => m.extruder.F);
    add('melt flow', 'mm3/s', () => m.extruder.Q * 1e9);
    add('spindle speed', 'rpm', () => m.spindle.omega * 60 / (2 * Math.PI));
    add('spindle current', 'A', () => m.spindle.armature.i);
    add('cutting power', 'W', () => m.cutting.Pf);
  }
  add(name, unit, fn) {
    this.channels.set(name, { unit, fn, data: new Float32Array(this.n) });
  }
  sample(t) {
    const i = this.idx;
    this.t[i] = t;
    for (const ch of this.channels.values()) ch.data[i] = ch.fn();
    this.idx = (i + 1) % this.n;
    this.count = Math.min(this.count + 1, this.n);
  }
  // Last `seconds` of a channel as [t[], v[]].
  series(name, seconds = 5) {
    const ch = this.channels.get(name);
    const k = Math.min(this.count, Math.round(seconds * 1000));
    const ts = new Float64Array(k), vs = new Float32Array(k);
    for (let j = 0; j < k; j++) {
      const i = (this.idx - k + j + this.n) % this.n;
      ts[j] = this.t[i]; vs[j] = ch.data[i];
    }
    return [ts, vs];
  }
}
