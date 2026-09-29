// Enclosure: a box of acrylic panels on an aluminium frame around the arm
// and the build plate, with a front door and an extraction fan.
//
// Why it is worth having on a multi-tool machine:
//   * a warm, still chamber for printing (less warping, no draughts),
//   * a door switch the firmware checks before it lets the spindle (or a
//     future laser) run: an interlock,
//   * fumes and dust leave through one fan and filter instead of the room.
//
// Physics:
//   Chamber air plus the inner layer of the panels form one thermal mass.
//   Hot parts inside (bed, hot end) lose heat to it through their air faces.
//   The chamber loses heat to the room through the panels (U x A) and with
//   the exhaust air stream (mass flow x cp). An open door adds a large air
//   exchange. The fan is a small 24 V motor whose speed follows its current;
//   the door switch is a contact that is closed while the door is shut.

import { Component } from '../core/graph.js';
import { AMBIENT } from '../core/units.js';
import { Contact } from '../electrical/circuit.js';
import { thermalPort, ThermalContact } from './thermal.js';

const RHO_AIR = 1.2, CP_AIR = 1005;

export class Enclosure extends Component {
  static PARAMS = ['box', 'Cair', 'Uwall', 'area', 'fanFlowMax', 'fanR', 'fanIrated', 'Gdoor', 'glands', 'doorSide'];
  static STATE = ['T', 'doorOpen', 'fanSpeed', 'door.closed', 'fanMotor.i'];
  constructor(id, { box, faces = 8, glands = [], ...opts }) {
    const size = [box.max[0] - box.min[0], box.max[1] - box.min[1], box.max[2] - box.min[2]];
    const centre = [0, 1, 2].map((k) => (box.min[k] + box.max[k]) / 2);
    super(id, { ...opts, kind: 'enclosure', size, mount: { body: opts.body, p: centre } });
    this.box = box;
    this.glands = glands;
    this.doorSide = opts.doorSide ?? '+x';
    this.area = 2 * (size[0] * size[1] + size[0] * size[2] + size[1] * size[2]);
    const V = size[0] * size[1] * size[2];
    const panelC = this.area * 0.005 * 1190 * 1470; // 5 mm acrylic
    this.Cair = RHO_AIR * V * CP_AIR + 0.3 * panelC; // air + the inner third of the panels
    this.Uwall = opts.Uwall ?? 3.5;                   // W/(m^2 K), still air both sides of acrylic
    this.Gdoor = opts.Gdoor ?? 12;                    // extra exchange with the door open [W/K]
    this.fanFlowMax = opts.fanFlowMax ?? 25 / 3600;   // 25 m^3/h at full speed
    this.fanR = opts.fanR ?? 160;                     // 24 V, 0.15 A fan
    this.fanIrated = 24 / this.fanR;
    this.T = AMBIENT;
    this.doorOpen = false;
    this.fanSpeed = 0; // 0..1
    const self = this;
    this.fanMotor = {
      label: `${id} exhaust fan`, i: 0,
      resistance: () => self.fanR,
      record(i) { this.i = i; },
    };
    this.door = new Contact(0.02, `${id} door switch`);
    this.door.closed = true;
    const wx = size[0] / 2, wy = size[1] / 2;
    this.addPort('FAN+', 'elec', 'load', { at: [-wx, wy * 0.6, size[2] * 0.3] });
    this.addPort('FAN-', 'elec', 'load', { at: [-wx, wy * 0.6, size[2] * 0.3 - 0.01] });
    this.addThrough('FAN+', 'FAN-', this.fanMotor);
    this.addPort('DOOR_COM', 'elec', 'sensor', { at: [wx, -wy * 0.8, size[2] * 0.2] });
    this.addPort('DOOR_NO', 'elec', 'sensor', { at: [wx, -wy * 0.8, size[2] * 0.2 - 0.01] });
    this.addThrough('DOOR_COM', 'DOOR_NO', this.door);
    this.faces = [];
    for (let i = 0; i < faces; i++) this.faces.push(thermalPort(this, `air${i}`, { required: false }));
    this.wall = thermalPort(this, 'wall');
    this.syncPorts();
  }
  syncPorts() { for (const f of this.faces) f.T = this.T; this.wall.T = this.T; }
  // A person opens or shuts the door (a user action on the physical machine).
  setDoor(open) { this.doorOpen = !!open; this.door.closed = !this.doorOpen; }
  airflow() { return this.fanFlowMax * this.fanSpeed; }
  // Heat path to the room: panels, exhaust air stream, open door.
  wallConductance() {
    return this.Uwall * this.area + RHO_AIR * CP_AIR * this.airflow() + (this.doorOpen ? this.Gdoor : 0);
  }
  thermalStep(dt) {
    let q = this.wall.q;
    this.wall.q = 0;
    for (const f of this.faces) { q += f.q; f.q = 0; }
    this.T += (dt * q) / this.Cair;
    // Fan speed follows its current (first order, about 1.5 s to spin up).
    const target = Math.min(1.2, Math.abs(this.fanMotor.i) / this.fanIrated);
    this.fanSpeed += (target - this.fanSpeed) * (1 - Math.exp(-dt / 1.5));
    this.syncPorts();
  }
  inspect() {
    return {
      chamber_C: this.T - 273.15, door: this.doorOpen ? 'open' : 'closed',
      fan_percent: 100 * this.fanSpeed, exhaust_m3h: this.airflow() * 3600, loss_WperK: this.wallConductance(),
    };
  }
}

// The boundary between chamber and room: its conductance is whatever the
// enclosure's panels, fan and door currently allow.
export class EnclosureWall extends ThermalContact {
  static PARAMS = [];
  static STATE = ['q'];
  conductance() { return this.a.owner.wallConductance(); }
}
