// Assembly of the complete machine: every component placed on a body and
// every wire, gearbox, thermal contact and sensor linkage between them.
// This file is the machine's wiring diagram and mechanical drawing in code.

import { Assembly } from '../core/graph.js';
import { tf, I3 } from '../core/linalg.js';
import { makeRng } from '../core/rng.js';
import { AMBIENT } from '../core/units.js';
import { Wire } from '../electrical/circuit.js';
import { PowerSupply, BuckConverter, TerminalBlock } from '../electrical/power.js';
import { StepperMotor, StepperDriver } from '../electrical/stepper.js';
import { Heater, Thermistor, MosfetModule, BrushedDCMotor, Servo, LimitSwitch } from '../electrical/devices.js';
import { ThermalMass, Ambient, ThermalContact } from '../thermal/thermal.js';
import { ArmModel } from '../mechanical/arm.js';
import { Gearbox, SensorCoupling, Turntable } from '../mechanical/transmission.js';
import { ChangerMaster, ToolPlate, MatingContact } from '../mechanical/toolchanger.js';
import { Extruder, GearMesh } from '../process/extruder.js';
import { Workpiece } from '../process/workpiece.js';
import { DepositionContact, CuttingContact } from '../process/contacts.js';
import { ControllerBoard, HostPC, UsbCable } from '../mcu/board.js';
import { Component } from '../core/graph.js';
import { SPEC, MOTORS, SPINDLE_MOTOR } from './spec.js';

// Pin assignment on the controller board (what a firmware config file lists).
export const PINS = {
  j1: { step: 'GP0', dir: 'GP1', home: 'GP16' },
  j2: { step: 'GP2', dir: 'GP3', home: 'GP17' },
  j3: { step: 'GP4', dir: 'GP5', home: 'GP18' },
  j4: { step: 'GP6', dir: 'GP7', home: 'GP19' },
  table: { step: 'GP8', dir: 'GP9', home: 'GP20' },
  e: { step: 'GP10', dir: 'GP11' },
  en: 'GP12',
  hotend: 'GP13', bed: 'GP14', spindle: 'GP15', servo: 'GP21',
};

// Slip ring: stator brushes on the fixed frame, rotor rings turning with the plate.
class SlipRing extends Component {
  constructor(id, { circuits = 6, rotorBody, ...opts }) {
    super(id, { ...opts, kind: 'slip-ring', size: [0.022, 0.022, 0.03] });
    this.rotorBody = rotorBody;
    for (let i = 1; i <= circuits; i++) {
      this.addPort(`S${i}`, 'elec', 'passthru', { required: false, at: [0.011, 0, -0.012 + i * 0.003] });
      this.addPort(`R${i}`, 'elec', 'passthru', { required: false, at: [-0.011, 0, -0.012 + i * 0.003] });
      this.addThrough(`S${i}`, `R${i}`, { label: `${id} ring ${i}`, i: 0, resistance: () => 0.02, record(i) { this.i = i; } });
    }
  }
}

// Simple mechanical part with mass and no ports (brackets, the bed plate).
class Part extends Component {
  constructor(id, opts) { super(id, { ...opts, kind: opts.kind ?? 'part' }); this.mass = opts.mass ?? 0; }
}

export function buildMachine(sim, spec = SPEC) {
  const A = new Assembly();
  const rng = makeRng(spec.seed);
  const tol = rng.fork(1);
  const world = A.body('world', tf());
  const base = A.body('base', tf());
  const tableBody = A.body('table', tf());
  const log = [];
  const note = (t, m) => { log.push([t, m]); if (log.length > 500) log.shift(); };

  // ------------------------------------------------------------ geometry
  // True dimensions = nominal + manufacturing error.
  const g = spec.arm;
  const err = () => tol.gauss(0, spec.tolerances.lengthSD);
  const truth = {
    baseHeight: g.baseHeight + err(), shoulderHeight: g.shoulderHeight + err(),
    L2: g.L2 + err(), L3: g.L3 + err(), L4: g.L4 + err(),
  };
  const Y = [0, -1, 0];
  const armSpec = {
    joints: [
      { name: 'j1', body: 'link1', offset: [0, 0, truth.baseHeight], axis: [0, 0, 1], limits: spec.limits.j1,
        structure: [{ m: 0.35, p: [0, 0, 0.04], size: [0.1, 0.1, 0.08] }] },
      { name: 'j2', body: 'link2', offset: [0, 0, truth.shoulderHeight], axis: Y, limits: spec.limits.j2,
        structure: [{ m: 0.30, p: [truth.L2 / 2, 0, 0], size: [truth.L2, 0.04, 0.03] }] },
      { name: 'j3', body: 'link3', offset: [truth.L2, 0, 0], axis: Y, limits: spec.limits.j3,
        structure: [{ m: 0.22, p: [truth.L3 / 2, 0, 0], size: [truth.L3, 0.035, 0.03] }] },
      { name: 'j4', body: 'link4', offset: [truth.L3, 0, 0], axis: Y, limits: spec.limits.j4,
        structure: [{ m: 0.08, p: [truth.L4 / 2, 0, 0], size: [truth.L4, 0.04, 0.04] }] },
    ],
  };
  const arm = A.add(new ArmModel('arm', armSpec, { assembly: A, baseBody: base, q0: spec.restPose }));
  const [link1, link2, link3, link4] = arm.bodies;
  const flange = [truth.L4, 0, 0];

  // ------------------------------------------------------------ electronics bay (on the bench)
  const at = (x, y, z = 0) => ({ body: world, p: [x, y, z] });
  const psu = A.add(new PowerSupply('psu', { mount: at(-0.40, 0.12), label: '24 V 15 A PSU' }));
  const tb = A.add(new TerminalBlock('tb24', [['P', 6], ['N', 6]], { mount: at(-0.26, 0.15), label: '24 V distribution' }));
  const buck = A.add(new BuckConverter('buck', { mount: at(-0.26, 0.10), label: '24->5 V buck' }));
  const tb5 = A.add(new TerminalBlock('tb5', [['V', 3], ['G', 3]], { mount: at(-0.20, 0.10), label: '5 V distribution' }));
  const board = A.add(new ControllerBoard('board', { sim, rng: rng.fork(2), mount: at(-0.26, -0.01), label: 'Controller board' }));
  const host = A.add(new HostPC('host', { sim, mount: at(-0.70, -0.30), label: 'Host PC' }));
  for (const [c, m] of [[psu, 0.6], [board, 0.04], [buck, 0.01]]) c.mass = m;

  const driverNames = ['j1', 'j2', 'j3', 'j4', 'table', 'e'];
  const drivers = {};
  driverNames.forEach((n, k) => {
    const d = spec.drives[n];
    drivers[n] = A.add(new StepperDriver(`drv_${n}`, {
      mount: at(-0.13, 0.14 - k * 0.045, 0.01), label: `TMC2209 ${n}`,
      microsteps: spec.microsteps, Irun: d.Irun,
    }));
  });
  const fets = {
    hotend: A.add(new MosfetModule('fet_hotend', { mount: at(-0.40, -0.04), label: 'MOSFET hot end' })),
    bed: A.add(new MosfetModule('fet_bed', { mount: at(-0.40, -0.09), label: 'MOSFET bed' })),
    spindle: A.add(new MosfetModule('fet_spindle', { mount: at(-0.40, -0.14), label: 'MOSFET spindle', flyback: true })),
  };
  for (const f of Object.values(fets)) f.port('SGND').required = false;

  // ------------------------------------------------------------ wires
  let wn = 0;
  const wire = (a, b, opts = {}) => A.connect(new Wire(opts.id ?? `w${++wn}`, A.p(a), A.p(b), opts));
  const RED = '#c0392b', BLK = '#1c1c1c', YEL = '#f1c40f', ORG = '#e67e22', WHT = '#ecf0f1',
    GRN = '#27ae60', BLU = '#2e86de', PUR = '#8e44ad', GRY = '#7f8c8d';

  // Mains-side DC: PSU -> distribution block.
  wire('psu.V+', 'tb24.P1', { awg: 16, color: RED, id: 'psu+' });
  wire('psu.V-', 'tb24.N1', { awg: 16, color: BLK, id: 'psu-' });
  wire('tb24.P2', 'buck.IN+', { awg: 22, color: RED });
  wire('tb24.N2', 'buck.IN-', { awg: 22, color: BLK });
  wire('buck.OUT+', 'tb5.V1', { awg: 22, color: RED });
  wire('buck.OUT-', 'tb5.G1', { awg: 22, color: BLK });
  wire('tb5.V2', 'board.5V', { awg: 24, color: RED });
  wire('tb5.G2', 'board.GND1', { awg: 24, color: BLK, id: 'board-gnd' });

  // Driver power (daisy-free: each from the block), logic supply and logic ground.
  const pBars = ['P3', 'P4', 'P5', 'P6'], nBars = ['N3', 'N4', 'N5', 'N6'];
  driverNames.forEach((n, k) => {
    const d = `drv_${n}`;
    // Two drivers share each distribution screw (as on a real block).
    wire(`tb24.${pBars[k >> 1]}`, `${d}.VM`, { awg: 20, color: RED, id: `${n}-VM` });
    wire(`tb24.${nBars[k >> 1]}`, `${d}.GND`, { awg: 20, color: BLK, id: `${n}-GND` });
    wire(`board.3V3_${k + 2}`, `${d}.VIO`, { awg: 26, color: '#e74c3c', id: `${n}-VIO` });
    wire(`board.GND${k + 2}`, `${d}.GNDL`, { awg: 26, color: BLK, id: `${n}-GNDL` });
    wire(`board.${PINS[n].step}`, `${d}.STEP`, { awg: 26, color: YEL, id: `${n}-STEP` });
    wire(`board.${PINS[n].dir}`, `${d}.DIR`, { awg: 26, color: ORG, id: `${n}-DIR` });
  });
  // Enable: one signal daisy-chained through a small splice to all drivers.
  const splice = A.add(new TerminalBlock('en_splice', [['E', 7, 'passthru']], { mount: at(-0.17, -0.12, 0.01), label: 'EN splice' }));
  wire(`board.${PINS.en}`, 'en_splice.E1', { awg: 26, color: WHT, id: 'EN' });
  driverNames.forEach((n, k) => wire(`en_splice.E${k + 2}`, `drv_${n}.EN`, { awg: 26, color: WHT, id: `${n}-EN` }));
  splice.passive = true;

  // MOSFET modules: power and gate signal.
  for (const [n, f] of Object.entries(fets)) {
    const bar = n === 'bed' ? ['P4', 'N4'] : ['P5', 'N5'];
    wire(`tb24.${bar[0]}`, `${f.id}.VIN+`, { awg: n === 'bed' ? 16 : 18, color: RED, id: `${n}-VIN+` });
    wire(`tb24.${bar[1]}`, `${f.id}.VIN-`, { awg: n === 'bed' ? 16 : 18, color: BLK, id: `${n}-VIN-` });
    wire(`board.${PINS[n]}`, `${f.id}.SIG`, { awg: 26, color: PUR, id: `${n}-SIG` });
  }
  // USB from the host PC.
  A.connect(new UsbCable('usb', host.port('USB'), board.port('USB'), { route: [] }));

  // ------------------------------------------------------------ harness route up the arm
  const harness = [
    { body: world, p: [-0.10, 0.0, 0.01] }, { body: base, p: [-0.045, 0.0, 0.03] },
    { body: base, p: [0, 0, 0.07] }, { body: link1, p: [0, 0.035, 0.07] },
    { body: link2, p: [0.02, 0.03, 0.025] }, { body: link2, p: [truth.L2 - 0.02, 0.03, 0.025] },
    { body: link3, p: [0.02, 0.03, 0.025] }, { body: link3, p: [truth.L3 - 0.02, 0.03, 0.025] },
    { body: link4, p: [0.01, 0.03, 0.02] },
  ];
  const upTo = (body) => {
    const i = harness.findIndex((w) => w.body === body);
    return harness.slice(0, i < 0 ? harness.length : i + 1);
  };

  // ------------------------------------------------------------ joint drives
  const motors = {}, gearboxes = {}, switches = {};
  const motorMounts = {
    j1: { body: base, p: [0.0, -0.05, 0.035] },
    j2: { body: link1, p: [0, 0.065, truth.shoulderHeight] },
    j3: { body: link2, p: [truth.L2, 0.06, 0] },
    j4: { body: link3, p: [truth.L3, 0.05, 0] },
    table: { body: world, p: [spec.table.center[0], -0.13, 0.025] },
  };
  const phaseColors = [BLK, GRN, RED, BLU];
  for (const n of ['j1', 'j2', 'j3', 'j4', 'table']) {
    const d = spec.drives[n], ms = MOTORS[d.motor];
    const qInit = n === 'table' ? 0 : spec.restPose[['j1', 'j2', 'j3', 'j4'].indexOf(n)];
    const m = A.add(new StepperMotor(`mot_${n}`, ms, { mount: motorMounts[n], label: `${d.motor} ${n}`, theta0: qInit * d.ratio }));
    m.mass = ms.mass + 0.2; // motor + gearbox housing
    motors[n] = m;
    const route = n === 'table' ? [{ body: world, p: [-0.05, -0.12, 0.01] }, { body: world, p: [0.2, -0.13, 0.01] }]
      : upTo(motorMounts[n].body);
    ['A1', 'A2', 'B1', 'B2'].forEach((ph, k) =>
      wire(`drv_${n}.${ph}`, `mot_${n}.${ph}`, { awg: 22, color: phaseColors[k], route, id: `${n}-${ph}` }));
  }
  const table = A.add(new Turntable('turntable', {
    body: tableBody, center: spec.table.center, J: 0.0026, mount: { body: world, p: [spec.table.center[0], 0, 0.03] },
    label: 'Turntable',
  }));
  const jointPort = (n) => (n === 'table' ? table.port('axis') : arm.port(n));
  for (const n of ['j1', 'j2', 'j3', 'j4', 'table']) {
    const d = spec.drives[n];
    const gb = A.connect(new Gearbox(`gb_${n}`, motors[n].port('shaft'), jointPort(n), {
      ratio: d.ratio, stiffness: d.stiffness, backlash: d.backlash, efficiency: d.efficiency,
      coulomb: d.coulomb, damping: d.damping, viscous: 0.05,
    }));
    gearboxes[n] = gb;
  }

  // Limit switches: body carrying the switch, and the cam on the next body.
  const swMount = {
    j1: { body: base, p: [0.05, 0.03, 0.07] }, j2: { body: link1, p: [0.03, 0.04, 0.085] },
    j3: { body: link2, p: [truth.L2 - 0.03, 0.04, 0.02] }, j4: { body: link3, p: [truth.L3 - 0.03, 0.04, 0.02] },
    table: { body: world, p: [spec.table.center[0] - 0.1, -0.02, 0.05] },
  };
  for (const [k, n] of ['j1', 'j2', 'j3', 'j4', 'table'].entries()) {
    const h = spec.homing[n];
    const trueTrip = h.trip + tol.gauss(0, spec.tolerances.switchSD);
    const s = A.add(new LimitSwitch(`sw_${n}`, {
      tripAngle: trueTrip, direction: h.dir, rng: rng.fork(10 + k), sim, mount: swMount[n], label: `limit ${n}`,
    }));
    switches[n] = s;
    A.connect(new SensorCoupling(`cam_${n}`, jointPort(n), s.port('cam'), { kind: 'cam' }));
    const route = n === 'table' ? [{ body: world, p: [-0.05, -0.10, 0.01] }] : upTo(swMount[n].body);
    wire(`board.${PINS[n].home}`, `sw_${n}.COM`, { awg: 26, color: GRY, route, id: `${n}-SW` });
    wire(`board.GND${9 + ['j1', 'j2', 'j3', 'j4', 'table'].indexOf(n)}`, `sw_${n}.NO`, { awg: 26, color: BLK, route, id: `${n}-SWG` });
  }

  // ------------------------------------------------------------ build plate
  const workpiece = A.add(new Workpiece('workpiece', { body: tableBody, size: spec.table.workSize, cell: spec.table.cell }));
  const bedPlate = A.add(new ThermalMass('bed_plate', { C: 470, mount: { body: tableBody, p: [0, 0, -0.003] }, faces: ['heater', 'air', 'probe'], label: 'Aluminium bed plate' }));
  bedPlate.size = [0.18, 0.18, 0.006]; bedPlate.round = spec.table.radius;
  const bedHeater = A.add(new Heater('bed_heater', { R0: 4.6, C: 20, mount: { body: tableBody, p: [0, 0, -0.007] }, label: 'Silicone bed heater 125 W' }));
  bedHeater.size = [0.17, 0.17, 0.0015];
  const bedTherm = A.add(new Thermistor('bed_therm', { mount: { body: tableBody, p: [0.02, 0.0, -0.007] }, C: 0.02 }));
  const slip = A.add(new SlipRing('slip', { rotorBody: tableBody, mount: { body: world, p: [spec.table.center[0], 0, 0.012] }, label: 'Slip ring' }));
  const bedRoute = [{ body: world, p: [-0.30, -0.10, 0.01] }, { body: world, p: [0.15, -0.02, 0.01] }];
  const rotorRoute = [{ body: tableBody, p: [0, 0.01, -0.02] }];
  wire('fet_bed.OUT+', 'slip.S1', { awg: 16, color: RED, route: bedRoute, id: 'bed+' });
  wire('fet_bed.OUT-', 'slip.S2', { awg: 16, color: BLK, route: bedRoute, id: 'bed-' });
  wire('slip.R1', 'bed_heater.a', { awg: 16, color: RED, route: rotorRoute });
  wire('slip.R2', 'bed_heater.b', { awg: 16, color: BLK, route: rotorRoute });
  wire('board.TH1', 'slip.S3', { awg: 26, color: WHT, route: bedRoute, id: 'bedT' });
  wire('board.TH1G', 'slip.S4', { awg: 26, color: WHT, route: bedRoute, id: 'bedTG' });
  wire('slip.R3', 'bed_therm.a', { awg: 26, color: WHT, route: rotorRoute });
  wire('slip.R4', 'bed_therm.b', { awg: 26, color: WHT, route: rotorRoute });

  // ------------------------------------------------------------ wrist: tool changer + latch servo
  const master = A.add(new ChangerMaster('changer', { pins: 10, flange, mount: { body: link4, p: flange }, label: 'Tool changer (arm side)' }));
  master.mass = 0.07;
  const servo = A.add(new Servo('servo', { mount: { body: link4, p: [truth.L4 * 0.5, 0.03, 0.0] }, label: 'Latch servo' }));
  servo.mass = 0.013;
  A.connect(new SensorCoupling('latch_link', servo.port('horn'), master.port('latch'), { kind: 'linkage' }));
  const wrist = upTo(link4);
  wire('tb5.V3', 'servo.V+', { awg: 24, color: RED, route: wrist, id: 'servo-5V' });
  wire('tb5.G3', 'servo.GND', { awg: 24, color: BLK, route: wrist, id: 'servo-GND' });
  wire(`board.${PINS.servo}`, 'servo.SIG', { awg: 26, color: ORG, route: wrist, id: 'servo-SIG' });
  // Harness to the pogo pins.
  wire('fet_hotend.OUT+', 'changer.P1', { awg: 20, color: RED, route: wrist, id: 'heater+' });
  wire('fet_hotend.OUT-', 'changer.P2', { awg: 20, color: BLK, route: wrist, id: 'heater-' });
  wire('board.TH0', 'changer.P3', { awg: 26, color: WHT, route: wrist, id: 'hotT' });
  wire('board.TH0G', 'changer.P4', { awg: 26, color: WHT, route: wrist, id: 'hotTG' });
  ['A1', 'A2', 'B1', 'B2'].forEach((ph, k) =>
    wire(`drv_e.${ph}`, `changer.P${5 + k}`, { awg: 22, color: phaseColors[k], route: wrist, id: `e-${ph}` }));
  wire('fet_spindle.OUT+', 'changer.P9', { awg: 18, color: RED, route: wrist, id: 'spindle+' });
  wire('fet_spindle.OUT-', 'changer.P10', { awg: 18, color: BLK, route: wrist, id: 'spindle-' });

  // ------------------------------------------------------------ tools
  const holderT = (H) => {
    const q1 = Math.atan2(H[1], H[0]), s = Math.sin(q1), c = Math.cos(q1);
    // Mating-face frame when the arm reaches H with the tool pointing down:
    // x along the tool (down), y = the wrist link's y axis, z = x cross y.
    return tf([0, -s, c, 0, c, s, -1, 0, 0], H);
  };
  const air = A.add(new Ambient('air', { mount: at(0, 0.5, 0.3), faces: 16, label: 'Room air' }));
  let airFace = 0;
  const toAir = (port, G, id) => A.connect(new ThermalContact(id, port, air.port(`air${airFace++}`), { G, kind: 'convection' }));

  // Hot end tool.
  const hotBody = A.body('tool_hotend', holderT(spec.rack.hotend));
  const hotPlate = A.add(new ToolPlate('plate_hotend', { pins: 10, toolName: 'hotend', holder: holderT(spec.rack.hotend), mount: { body: hotBody, p: [0, 0, 0] }, label: 'Hot end tool plate' }));
  hotPlate.mass = 0.03;
  const tip = spec.tools.hotend.tipOffset;
  const extruder = A.add(new Extruder('extruder', { mount: { body: hotBody, p: [0.022, 0, 0.022] }, nozzleAt: [tip - 0.022, 0, -0.022], label: 'Direct-drive extruder' }));
  extruder.mass = 0.09;
  const emot = A.add(new StepperMotor('mot_e', MOTORS['36STH20'], { mount: { body: hotBody, p: [0.02, 0, 0.05] }, label: 'Extruder motor', extraInertia: 3e-7 }));
  emot.mass = MOTORS['36STH20'].mass;
  const block = A.add(new ThermalMass('hot_block', { C: 11, mount: { body: hotBody, p: [tip - 0.012, 0, 0] }, faces: ['heater', 'probe', 'melt', 'air'], label: 'Heater block + nozzle' }));
  block.mass = 0.07; block.size = [0.012, 0.02, 0.016];
  const hotHeater = A.add(new Heater('hot_heater', { R0: 14.4, C: 0.6, mount: { body: hotBody, p: [tip - 0.012, 0, 0.004] }, label: 'Heater cartridge 40 W' }));
  const hotTherm = A.add(new Thermistor('hot_therm', { mount: { body: hotBody, p: [tip - 0.012, 0, -0.004] } }));
  A.connect(new GearMesh('extruder_gears', emot.port('shaft'), extruder.port('drive'), { ratio: spec.drives.e.ratio }));
  A.connect(new ThermalContact('tc_cartridge', hotHeater.port('surface'), block.port('heater'), { G: 0.8 }));
  A.connect(new ThermalContact('tc_hot_probe', block.port('probe'), hotTherm.port('bead'), { G: 0.004 }));
  A.connect(new ThermalContact('tc_melt', block.port('melt'), extruder.port('melt'), { G: extruder.meltG }));
  toAir(block.port('air'), 0.055, 'conv_block');
  const pins = (pl, pairs) => {
    for (const [pin, target] of pairs)
      wire(`${pl}.${pin}`, target, { awg: 24, color: GRY, length: 0.06, id: `${pl}-${pin}` });
  };
  pins('plate_hotend', [['P1', 'hot_heater.a'], ['P2', 'hot_heater.b'], ['P3', 'hot_therm.a'], ['P4', 'hot_therm.b'],
    ['P5', 'mot_e.A1'], ['P6', 'mot_e.A2'], ['P7', 'mot_e.B1'], ['P8', 'mot_e.B2']]);

  // Spindle tool.
  const spBody = A.body('tool_spindle', holderT(spec.rack.spindle));
  const spPlate = A.add(new ToolPlate('plate_spindle', { pins: 10, toolName: 'spindle', holder: holderT(spec.rack.spindle), mount: { body: spBody, p: [0, 0, 0] }, label: 'Spindle tool plate' }));
  spPlate.mass = 0.03;
  const spTip = spec.tools.spindle.tipOffset;
  const spindle = A.add(new BrushedDCMotor('spindle', SPINDLE_MOTOR, { mount: { body: spBody, p: [0.045, 0, 0] }, bitAt: [spTip - 0.045, 0, 0], label: '775 spindle + 1/8" ball-nose end mill' }));
  spindle.mass = 0.45; spindle.size = [0.09, 0.045, 0.045];
  pins('plate_spindle', [['P9', 'spindle.M+'], ['P10', 'spindle.M-']]);

  // Pogo contacts between the flange and each tool plate.
  for (const pl of [hotPlate, spPlate])
    for (let i = 1; i <= 10; i++)
      A.connect(new MatingContact(`pogo_${pl.toolName}_${i}`, master.port(`P${i}`), pl.port(`P${i}`), { ratedA: 5 }));

  // Bed thermal network.
  A.connect(new ThermalContact('tc_bed_pad', bedHeater.port('surface'), bedPlate.port('heater'), { G: 30 }));
  A.connect(new ThermalContact('tc_bed_probe', bedPlate.port('probe'), bedTherm.port('bead'), { G: 0.006 }));
  toAir(bedPlate.port('air'), 0.55, 'conv_bed');

  // Process contacts.
  const deposition = A.connect(new DepositionContact('deposition', extruder.port('nozzle'), workpiece.port('surface')));
  const cutting = A.connect(new CuttingContact('cutting', spindle.port('bit'), workpiece.port('surface'), {
    radius: spec.tools.spindle.cutterRadius, flutes: spec.tools.spindle.flutes, shape: spec.tools.spindle.cutter,
  }));

  // Brackets / rack as massless visual parts.
  A.add(new Part('rack', { mount: at(0.06, 0, 0.1), size: [0.04, 0.6, 0.02], label: 'Tool rack' }));

  return {
    A, sim, rng, log, note, truth, arm, table, psu, buck, board, host, drivers, motors, gearboxes, switches, fets,
    workpiece, bedPlate, bedHeater, bedTherm, block, hotHeater, hotTherm, extruder, emot, spindle, servo,
    master, plates: [hotPlate, spPlate], deposition, cutting, air, slip, flange, spec,
  };
}
