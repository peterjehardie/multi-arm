// Controller board (a 3.3 V, 32-bit microcontroller on a carrier board) and
// the host PC that streams G-code to it over a USB cable.
//
// The firmware never touches the machine directly. Everything it can do goes
// through this board's silicon: set a pin high or low, read a pin, start a
// hardware PWM, read an ADC channel, arm a timer interrupt, read bytes that
// arrived over USB. The board then drives real ports, and wires carry the
// result onward.

import { Component, Connection } from '../core/graph.js';

const CLOCK_HZ = 125e6;       // core clock: timer compare resolution 8 ns
const ISR_LATENCY = 0.4e-6;   // interrupt entry + first instruction
const V33 = 3.3;

export class ControllerBoard extends Component {
  static PARAMS = [];
  static STATE = ['running', 'adcState.TH0.v', 'adcState.TH1.v', 'load.current'];
  constructor(id, { sim, rng, gpio = 26, ...opts }) {
    super(id, { ...opts, kind: 'controller', size: [0.085, 0.056, 0.015] });
    this.sim = sim;
    this.rng = rng;
    this.addPort('5V', 'elec', 'supply', { at: [-0.04, 0.02, 0.008] });
    for (let i = 1; i <= 16; i++)
      this.addPort(`GND${i}`, 'elec', 'supply', { node: 'gnd', required: false, at: [-0.04 + i * 0.005, -0.026, 0.008] });
    for (let i = 1; i <= 8; i++)
      this.addPort(`3V3_${i}`, 'elec', 'supply', { node: '3v3', required: false, at: [-0.04 + i * 0.008, 0.026, 0.008] });
    for (let i = 0; i < gpio; i++)
      this.addPort(`GP${i}`, 'elec', 'gpio', { required: false, at: [0.042, 0.024 - i * 0.002, 0.008] });
    // Thermistor inputs: 4.7 k pull-up to 3V3 and a 10 uF filter capacitor on the board.
    for (const n of ['TH0', 'TH1']) {
      this.addPort(n, 'elec', 'analog', { required: false });
      this.addPort(`${n}G`, 'elec', 'sensor', { node: 'gnd', required: false });
    }
    this.addPort('USB', 'elec', 'usb', { required: false, at: [-0.042, 0, 0.006] });
    this.adcState = { TH0: { v: V33, loop: null }, TH1: { v: V33, loop: null } };
    this.pins = new Map(); // name -> { mode, level, net, cbs, loop, inLevel }
    this.load = { current: 0.045 };
    this.ldo = {
      changed: false, on: false,
      norton: () => (this.ldo.on ? { G: 1 / 0.05, I: V33 / 0.05 } : { G: 0, I: 0 }),
    };
    this.mcu = new MCU(this);
    this.running = false;
  }

  dcStamp(net) {
    this.net = net;
    net.addLoad(this.port('5V'), this.port('GND1'), this.load);
    net.addCapacitor(this.port('3V3_1'), this.port('GND1'), 22e-6);
    net.addCapacitor(this.port('5V'), this.port('GND1'), 10e-6);
    net.addSource(this.port('3V3_1'), this.port('GND1'), this.ldo);
  }
  resolveLoops(traceLoop) {
    const toGnd = (q) => q.owner === this && q.node === 'gnd';
    for (const n of ['TH0', 'TH1']) this.adcState[n].loop = traceLoop(this.port(n), this.port(`${n}G`));
    for (const p of this.ports.values())
      if (p.role === 'gpio') this.pin(p.name).loop = traceLoop(p, toGnd);
  }
  attachNet(portName, net) { this.pin(portName).net = net; }
  pin(name) {
    if (!this.pins.has(name)) this.pins.set(name, { mode: 'in', level: false, net: null, cbs: [], loop: null, inLevel: true });
    return this.pins.get(name);
  }

  // Logic interface for the digital nets.
  logicHigh() { return this.net ? this.net.across(this.port('3V3_1'), this.port('GND1')) : V33; }
  logicGround() { return this.port('GND1'); }
  outputResistance() { return 30; }
  inputCapacitance() { return 5e-12; }
  inputThresholds() { return { rise: 2.0, fall: 0.8 }; }
  digitalIn(name, level, t) { this.setInput(name, level, t); }

  setInput(name, level, t) {
    const p = this.pin(name);
    if (p.inLevel === level) return;
    p.inLevel = level;
    if (p.mode === 'out') return;
    for (const cb of p.cbs) this.sim.at(t + ISR_LATENCY, () => cb(level, t), 'gpio-isr');
  }

  // Electrical step: power-good, pull-up inputs, ADC filter.
  update(dt, t) {
    const v5 = this.net.across(this.port('5V'), this.port('GND1'));
    const on = v5 > 3.7;
    if (on !== this.ldo.on) { this.ldo.on = on; this.ldo.changed = true; }
    if (on && !this.running) { this.running = true; this.mcu.boot(t); }
    if (!on && this.running) { this.running = false; this.mcu.brownout(t); }
    // Inputs with the internal ~50 k pull-up and an external switch to ground.
    for (const [name, p] of this.pins) {
      if (p.mode !== 'in_pullup' || !p.loop) continue;
      const R = p.loop.resistance();
      const v = Number.isFinite(R) ? (V33 * R) / (50e3 + R) : V33;
      const lvl = p.inLevel ? v > 0.8 : v > 2.0;
      if (lvl !== p.inLevel) this.setInput(name, lvl, t);
    }
  }
  adcUpdate(dt) {
    for (const s of Object.values(this.adcState)) {
      const R = s.loop ? s.loop.resistance() : Infinity;
      const Rpu = 4700;
      const vTarget = Number.isFinite(R) ? (V33 * R) / (Rpu + R) : V33;
      const Rth = Number.isFinite(R) ? (Rpu * R) / (Rpu + R) : Rpu;
      const tau = Rth * 10e-6;
      s.v += (vTarget - s.v) * (1 - Math.exp(-dt / tau));
      if (s.loop) s.loop.setCurrent((V33 - s.v) / Rpu);
    }
  }
  inspect() {
    const out = { powered: this.running };
    for (const [n, p] of this.pins) if (p.mode !== 'in' || p.cbs.length) out[n] = `${p.mode}:${p.mode === 'out' ? +p.level : +p.inLevel}`;
    return out;
  }
}

// The microcontroller's peripherals, as seen by firmware.
class MCU {
  constructor(board) {
    this.board = board;
    this.sim = board.sim;
    this.pwms = new Map();
    this.usbRxQueue = [];
    this.usbRxCb = null;
    this.usbTx = null;
    this.onBoot = null;
    this.bootCount = 0;
  }
  now() { return Math.floor(this.sim.t * 1e6) / 1e6; } // 1 MHz system timer
  boot(t) { this.bootCount++; this.onBoot?.(t); }
  brownout() {
    for (const [name] of this.pwms) this.pwmStop(name);
    for (const p of this.board.pins.values()) { p.mode = 'in'; if (p.net) p.net.release(this.sim.t); }
    this.onBrownout?.();
  }
  quantize(t) { return Math.ceil(t * CLOCK_HZ) / CLOCK_HZ; }

  pinMode(name, mode) {
    const p = this.board.pin(name);
    p.mode = mode;
    if (mode !== 'out' && p.net) p.net.release(this.sim.t);
  }
  write(name, level) {
    const p = this.board.pin(name);
    if (p.mode !== 'out') return;
    p.level = level;
    if (p.net) p.net.drive(level, this.sim.t);
  }
  read(name) {
    const p = this.board.pin(name);
    return p.mode === 'out' ? p.level : p.inLevel;
  }
  onChange(name, cb) { this.board.pin(name).cbs.push(cb); }

  // 12-bit ADC, ratiometric to 3V3, with ~1.5 LSB of noise.
  adc(name) {
    const s = this.board.adcState[name];
    const code = Math.round((s.v / V33) * 4095 + this.board.rng.gauss(0, 1.5));
    return Math.max(0, Math.min(4095, code));
  }

  // Hardware timer compare: runs cb at absolute time t (plus ISR latency).
  timerAt(t, cb) { return this.sim.at(this.quantize(t) + ISR_LATENCY, cb, 'timer'); }

  // Hardware PWM on a pin. Duty changes take effect at the next period.
  pwm(name, freq, duty) {
    let s = this.pwms.get(name);
    if (!s) {
      s = { freq, duty, next: duty, running: false };
      this.pwms.set(name, s);
    }
    s.freq = freq; s.next = Math.max(0, Math.min(1, duty));
    if (!s.running) {
      s.running = true;
      this.pinMode(name, 'out');
      const period = (t) => {
        if (!s.running) return;
        s.duty = s.next;
        const T = 1 / s.freq;
        if (s.duty <= 0) this.write(name, false);
        else if (s.duty >= 1) this.write(name, true);
        else {
          this.write(name, true);
          this.sim.at(this.quantize(t + s.duty * T), () => { if (s.running) this.write(name, false); }, 'pwm');
        }
        s.handle = this.sim.at(this.quantize(t + T), period, 'pwm');
      };
      period(this.sim.t);
    }
  }
  pwmStop(name) {
    const s = this.pwms.get(name);
    if (!s) return;
    s.running = false;
    s.handle?.cancel();
    this.pwms.delete(name);
    this.write(name, false);
  }

  // USB CDC serial.
  onUsbLine(cb) { this.usbRxCb = cb; }
  usbWrite(text) { this.usbTx?.(text); }
}

// ---------------------------------------------------------------------------
// USB cable between host PC and controller board. Full-speed USB moves data
// in 1 ms frames, so a line of G-code arrives at the next frame boundary
// after it is sent, plus its serialisation time at 12 Mbit/s.
export class UsbCable extends Connection {
  static PARAMS = ['length'];
  static STATE = ['bytes'];
  constructor(id, a, b, opts = {}) {
    super(id, a, b, { ...opts, kind: 'usb-cable' });
    this.color = '#222222';
    this.length = 1.5;
    this.bytes = 0;
  }
  // Transfers are serialised: a line cannot overtake the one before it.
  deliver(sim, text, to, dir = 'down') {
    const frame = 1e-3;
    const key = dir === 'down' ? 'busyDown' : 'busyUp';
    const tFrame = Math.ceil((Math.max(sim.t, this[key] ?? 0) + 1e-9) / frame) * frame;
    const tArr = Math.max(tFrame, this[key] ?? 0) + ((text.length + 8) * 8) / 12e6;
    this[key] = tArr;
    this.bytes += text.length;
    sim.at(tArr, () => to(text), 'usb');
  }
}

export class HostPC extends Component {
  static PARAMS = ['window'];
  static STATE = ['sent', 'inFlight'];
  constructor(id, { sim, ...opts }) {
    super(id, { ...opts, kind: 'host', size: [0.3, 0.2, 0.02] });
    this.sim = sim;
    this.addPort('USB', 'elec', 'usb', { at: [0.15, 0, 0.01] });
    this.lines = [];
    this.sent = 0;
    this.inFlight = 0;
    this.window = 4;       // lines allowed in flight before waiting for 'ok'
    this.log = [];
    this.onMessage = null;
    this.listeners = [];   // other programs on the PC reading the serial port
  }
  link(board) {
    this.cable = this.port('USB').connection;
    this.board = board;
    board.mcu.usbTx = (text) => this.cable.deliver(this.sim, text, (m) => this.receive(m), 'up');
  }
  load(gcodeText) {
    for (const raw of gcodeText.split(/\r?\n/)) {
      const line = raw.replace(/;.*$/, '').trim();
      if (line) this.lines.push(line);
    }
    this.pump();
  }
  send(line) { this.lines.push(line); this.pump(); }
  pump() {
    // The USB serial port only exists once the board's MCU has booted and
    // announced itself; until then there is nothing to send to.
    if (!this.connected) return;
    while (this.inFlight < this.window && this.sent < this.lines.length) {
      const line = this.lines[this.sent++];
      this.inFlight++;
      this.cable.deliver(this.sim, line, (m) => this.board.mcu.usbRxCb?.(m));
    }
  }
  receive(msg) {
    if (msg === 'start') { this.connected = true; this.inFlight = 0; this.pump(); }
    if (msg.startsWith('ok')) { this.inFlight = Math.max(0, this.inFlight - 1); this.pump(); }
    else {
      this.log.push([this.sim.t, msg]); if (this.log.length > 500) this.log.shift();
      this.onMessage?.(msg);
      for (const f of this.listeners) f(msg, this.sim.t);
    }
  }
  get done() { return this.sent >= this.lines.length && this.inFlight === 0; }
  inspect() { return { sent: this.sent, queued: this.lines.length - this.sent, inFlight: this.inFlight }; }
}
