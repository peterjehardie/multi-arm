// Machine firmware.
//
// It runs on the simulated microcontroller and can only use the MCU's
// peripherals: GPIO pins, timer interrupts, hardware PWM, ADC, USB serial.
// Structure (as in real printer / CNC firmware):
//   * G-code arrives over USB, is parsed and queued.
//   * A 1 kHz "servo tick" interrupt advances the current move along its
//     velocity profile, converts the tool position to joint angles (inverse
//     kinematics), and schedules the step pulses each driver needs in the
//     next millisecond on hardware timers.
//   * A 10 Hz loop reads thermistors through the ADC and runs heater PID
//     loops that set PWM duty on the heater MOSFET pins.
//   * Homing drives each joint slowly into its limit switch and defines the
//     joint's zero from where the switch trips.
// The firmware only knows the nominal geometry. Whatever the real machine
// does differently (tolerances, backlash, deflection) shows up as error at
// the tool tip, just as on a real build.

import { ArmKinematics } from './kinematics.js';

const TICK = 1e-3;
const STEP_PULSE = 2e-6;
const DEG = Math.PI / 180;
const ARM_AXES = ['j1', 'j2', 'j3', 'j4'];
const wrap = (a) => a - 2 * Math.PI * Math.round(a / (2 * Math.PI));

export class Firmware {
  constructor(mcu, cfg) {
    this.mcu = mcu;
    this.cfg = cfg;
    this.kin = new ArmKinematics(cfg.geometry);
    this.axes = {};
    for (const [name, a] of Object.entries(cfg.axes))
      this.axes[name] = { name, ...a, pos: 0, issued: 0, zero: 0, dirLevel: false, triggered: false, trigIssued: 0 };
    this.queue = [];
    this.current = null;     // running command state machine
    this.homed = false;
    this.enabled = false;
    this.mode = { absXYZ: true, absE: true };
    // Last commanded position in work coordinates (m, rad).
    this.pos = { x: 0, y: 0, z: 0.05, c: 0, a: 0, e: 0 };
    this.heaters = {};
    for (const [n, h] of Object.entries(cfg.heaters))
      this.heaters[n] = { ...h, target: 0, temp: NaN, integ: 0, prevErr: 0, duty: 0, fault: null, runaway: { t: 0, T: 0 } };
    this.tool = null;        // which tool firmware believes is locked on
    this.spindle = { target: 0, duty: 0 };
    this.log = [];
    this.stats = { moves: 0, lines: 0, maxStepsPerTick: 0 };
    mcu.onBoot = (t) => this.boot(t);
    mcu.onBrownout = () => { this.running = false; };
  }

  msg(text) {
    this.log.push([this.mcu.now(), text]);
    if (this.log.length > 2000) this.log.shift();
    this.mcu.usbWrite(text);
  }

  // ------------------------------------------------------------------ boot
  boot(t) {
    const m = this.mcu, c = this.cfg;
    this.running = true;
    for (const a of Object.values(this.axes)) {
      m.pinMode(a.step, 'out'); m.write(a.step, false);
      m.pinMode(a.dir, 'out'); m.write(a.dir, false);
      if (a.home) {
        m.pinMode(a.home.pin, 'in_pullup');
        m.onChange(a.home.pin, (level) => { if (!level) this.onSwitch(a); });
      }
    }
    m.pinMode(c.enPin, 'out');
    // EN is active low. An arm with back-drivable gearboxes sags when the
    // motors are off, so by default the drivers are enabled at boot to hold it.
    this.enable(!!c.holdAtBoot);
    for (const h of Object.values(this.heaters)) { m.pinMode(h.pin, 'out'); m.write(h.pin, false); }
    m.pinMode(c.spindle.pin, 'out'); m.write(c.spindle.pin, false);
    m.pwm(c.servo.pin, 50, c.servo.unlockUs * 50e-6);
    m.onUsbLine((line) => this.onLine(line));
    // Timer compare values advance by exactly one period, so ticks do not drift.
    let tickAt = Math.ceil(t / TICK + 1) * TICK;
    const tick = (tt) => {
      if (!this.running) return;
      this.tick(tt);
      tickAt += TICK;
      m.timerAt(tickAt, tick);
    };
    m.timerAt(tickAt, tick);
    const slow = (tt) => { if (!this.running) return; this.slowLoop(tt); m.timerAt(tt + 0.1, slow); };
    m.timerAt(t + 0.1, slow);
    this.msg('start');
  }

  // --------------------------------------------------------------- G-code
  onLine(line) {
    this.stats.lines++;
    this.queue.push(line.trim());
    this.mcu.usbWrite('ok');
  }
  static parse(line) {
    const out = { cmd: '', args: {} };
    const toks = line.toUpperCase().match(/[A-Z_][^A-Z_\s]*/g) ?? [];
    if (line.startsWith('_')) {
      const [cmd, ...rest] = line.split(/\s+/);
      return { cmd, args: rest };
    }
    for (const t of toks) {
      const k = t[0], v = parseFloat(t.slice(1));
      if (!out.cmd && (k === 'G' || k === 'M' || k === 'T')) out.cmd = k + (Number.isNaN(v) ? '' : v);
      else out.args[k] = v;
    }
    return out;
  }

  // ------------------------------------------------------ 1 kHz servo tick
  tick(t) {
    let guard = 0;
    while (!this.current && this.queue.length && guard++ < 20) this.start(this.queue.shift(), t);
    const cur = this.current;
    if (cur) {
      const done = cur.update(t);
      if (done) this.current = null;
    }
  }

  start(line, t) {
    const { cmd, args } = Firmware.parse(line);
    const need = (ok) => { if (!ok) this.msg(`error: not homed, ignoring ${line}`); return ok; };
    if (cmd !== 'G0' && cmd !== 'G1') { this.vCarry = 0; this.tCarry = null; }
    switch (cmd) {
      case 'G0': case 'G1': if (need(this.homed)) this.startLinear(args, cmd === 'G0', t); break;
      case 'G4': {
        const dur = (args.P ?? 0) / 1000 + (args.S ?? 0);
        const tEnd = t + dur;
        this.current = { update: (tt) => tt >= tEnd };
        break;
      }
      case 'G28': this.startHoming(t); break;
      case 'G90': this.mode.absXYZ = true; break;
      case 'G91': this.mode.absXYZ = false; break;
      case 'G92': if (args.E !== undefined) this.pos.e = args.E / 1000; break;
      case 'M82': this.mode.absE = true; break;
      case 'M83': this.mode.absE = false; break;
      case 'M17': this.enable(true); break;
      case 'M18': case 'M84': this.enable(false); this.homed = false; break;
      case 'M104': case 'M109': this.setHeater('hotend', args.S ?? 0, cmd === 'M109', t); break;
      case 'M140': case 'M190': this.setHeater('bed', args.S ?? 0, cmd === 'M190', t); break;
      case 'M3': this.setSpindle(args.S ?? this.cfg.spindle.maxRpm, t); break;
      case 'M5': this.setSpindle(0, t); break;
      case 'M6': if (need(this.homed)) this.toolChange(args.T ?? 0, t); break;
      case 'M114': this.msg(this.positionReport()); break;
      case 'M400': break;
      case 'M106': case 'M107': break;
      // Polar mode: the plate turns so every point is worked on the side
      // facing the arm. The arm then moves only in its own vertical plane.
      case 'M620': if (need(this.homed)) this.setPolar(true, t); break;
      case 'M621': this.polar = false; break;
      case '_TIP': this.kin.tip = parseFloat(args[0]); this.syncPosFromJoints(); break;
      case '_MOVEW': this.startWorldMove(args.map(Number), t); break;
      case '_SERVO': this.mcu.pwm(this.cfg.servo.pin, 50, (args[0] === 'lock' ? this.cfg.servo.lockUs : this.cfg.servo.unlockUs) * 50e-6); break;
      case '_TOOL': this.tool = args[0] === 'none' ? null : args[0]; this.msg(`tool: ${args[0]}`); break;
      default: if (cmd || Object.keys(args).length) this.msg(`unknown: ${line}`);
    }
  }

  enable(on) {
    this.enabled = on;
    this.mcu.write(this.cfg.enPin, !on);
  }

  // ------------------------------------------------------------- motion
  jointAngles() {
    const o = {};
    for (const a of Object.values(this.axes)) o[a.name] = (a.pos - a.zero) / a.spr;
    return o;
  }
  workToWorld(p, c) {
    const [cx, cy, cz] = this.cfg.table.center;
    const cs = Math.cos(c), sn = Math.sin(c);
    return [cx + cs * p[0] - sn * p[1], cy + sn * p[0] + cs * p[1], cz + p[2]];
  }
  worldToWork(w, c) {
    const [cx, cy, cz] = this.cfg.table.center;
    const dx = w[0] - cx, dy = w[1] - cy, cs = Math.cos(c), sn = Math.sin(c);
    return [cs * dx + sn * dy, -sn * dx + cs * dy, w[2] - cz];
  }
  syncPosFromJoints() {
    const j = this.jointAngles();
    const fk = this.kin.forward([j.j1, j.j2, j.j3, j.j4]);
    const w = this.worldToWork(fk.p, j.table ?? 0);
    Object.assign(this.pos, { x: w[0], y: w[1], z: w[2], c: j.table ?? 0, a: fk.phi + Math.PI / 2 });
  }
  positionReport() {
    const p = this.pos, j = this.jointAngles();
    const f = (v) => (v * 1000).toFixed(3);
    return `X:${f(p.x)} Y:${f(p.y)} Z:${f(p.z)} C:${(p.c / DEG).toFixed(2)} A:${(p.a / DEG).toFixed(2)} E:${f(p.e)}` +
      ` | J1:${(j.j1 / DEG).toFixed(2)} J2:${(j.j2 / DEG).toFixed(2)} J3:${(j.j3 / DEG).toFixed(2)} J4:${(j.j4 / DEG).toFixed(2)}`;
  }

  // Joint targets for a work-coordinate tool pose.
  solve(p, c, a, e) {
    const w = this.workToWorld(p, c);
    const q = this.kin.inverse(w, -Math.PI / 2 + a, this.jointAngles().j1);
    if (!q) return null;
    return { j1: q[0], j2: q[1], j3: q[2], j4: q[3], table: c, e };
  }
  withinLimits(q) {
    for (const n of ARM_AXES) {
      const [lo, hi] = this.cfg.limits[n];
      if (!(q[n] >= lo && q[n] <= hi)) return false;
    }
    return true;
  }

  // Target pose of a G0/G1 given the pose it starts from.
  targetOf(args, p0) {
    const mm = (v) => v / 1000;
    const p1 = { ...p0 };
    for (const [k, key] of [['X', 'x'], ['Y', 'y'], ['Z', 'z']])
      if (args[k] !== undefined) p1[key] = this.mode.absXYZ ? mm(args[k]) : p0[key] + mm(args[k]);
    if (args.C !== undefined) p1.c = args.C * DEG;
    if (args.A !== undefined) p1.a = args.A * DEG;
    if (args.E !== undefined) p1.e = this.mode.absE ? mm(args.E) : p0.e + mm(args.E);
    return p1;
  }

  // Look-ahead of one move: how fast may the tool pass the corner between
  // this move and the next queued one? (Junction deviation, as in common
  // printer firmware, capped so the next move can still stop in its length.)
  junctionSpeed(p0, p1, v1, rapid) {
    const next = this.queue[0];
    if (!next || !/^G[01](\s|$)/i.test(next)) return 0;
    const { cmd, args } = Firmware.parse(next);
    if (args.C !== undefined || args.A !== undefined) return 0;
    const p2 = this.targetOf(args, p1);
    const d1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z], d2 = [p2.x - p1.x, p2.y - p1.y, p2.z - p1.z];
    const L1 = Math.hypot(...d1), L2 = Math.hypot(...d2);
    if (L1 < 1e-6 || L2 < 1e-6) return 0;
    const F2 = cmd === 'G0' ? this.cfg.rapid : (args.F ?? this.feed ?? 1200);
    const cosT = -(d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2]) / (L1 * L2);
    const a = this.cfg.accel;
    let vj = Math.min(v1, F2 / 60 / 1000, Math.sqrt(2 * a * L2));
    if (cosT > 0.999) return 0; // reversal
    if (cosT > -0.999999) {
      const sh = Math.sqrt(0.5 * (1 - cosT));
      vj = Math.min(vj, Math.sqrt((a * this.cfg.junctionDeviation * sh) / (1 - sh)));
    }
    void rapid;
    return vj;
  }

  startLinear(args, rapid, t) {
    const p0 = { ...this.pos };
    const p1 = this.targetOf(args, p0);
    if (args.F !== undefined) this.feed = args.F;
    const F = rapid ? this.cfg.rapid : (this.feed ?? 1200);
    const polar = this.polar && args.C === undefined;
    const cAt = (s) => {
      if (!polar) return p0.c + s * (p1.c - p0.c);
      const x = p0.x + s * (p1.x - p0.x), y = p0.y + s * (p1.y - p0.y);
      if (Math.hypot(x, y) < 1e-4) return p0.c;
      return p0.c + wrap(this.polarAngle(x, y) - p0.c);
    };
    if (polar) p1.c = cAt(1);
    const eval_ = (s) => this.solve(
      [p0.x + s * (p1.x - p0.x), p0.y + s * (p1.y - p0.y), p0.z + s * (p1.z - p0.z)],
      cAt(s), p0.a + s * (p1.a - p0.a), p0.e + s * (p1.e - p0.e));
    const L = Math.hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    // Commanded rotations count toward the move length (at a 100 mm radius);
    // a plate angle derived by polar mode does not, the tool path does.
    const Lrot = Math.max(polar ? 0 : Math.abs(p1.c - p0.c), Math.abs(p1.a - p0.a)) * 0.1;
    const Le = Math.abs(p1.e - p0.e);
    const len = Math.max(L, Lrot, L > 1e-7 || Lrot > 1e-7 ? 0 : Le);
    // Check the whole path is reachable and inside joint limits.
    for (let k = 0; k <= 8; k++) {
      const q = eval_(k / 8);
      if (!q || !this.withinLimits(q)) {
        this.msg(`error: move out of reach, skipped (${p1.x * 1e3},${p1.y * 1e3},${p1.z * 1e3})`);
        this.vCarry = 0;
        return;
      }
    }
    this.pos = p1;
    const vmax = F / 60 / 1000;
    const vEnd = L > 1e-6 && args.C === undefined && args.A === undefined ? this.junctionSpeed(p0, p1, vmax, rapid) : 0;
    this.runMove(eval_, len, vmax, this.cfg.accel, t, null, vEnd);
  }

  // Plate angle that brings work point (x, y) onto the line from the plate
  // centre toward the arm base.
  polarAngle(x, y) {
    const [cx, cy] = this.cfg.table.center;
    return Math.atan2(-cy, -cx) - Math.atan2(y, x);
  }
  setPolar(on, t) {
    this.polar = on;
    const target = this.pos.c + wrap(this.polarAngle(this.pos.x, this.pos.y) - this.pos.c);
    if (Math.hypot(this.pos.x, this.pos.y) > 1e-4) this.startLinear({ C: target / DEG }, true, t);
  }

  startWorldMove([x, y, z, F], t) {
    const j = this.jointAngles();
    const fk = this.kin.forward([j.j1, j.j2, j.j3, j.j4]);
    const w0 = fk.p, w1 = [x, y, z];
    const phi0 = fk.phi, phi1 = -Math.PI / 2;
    const c = j.table, e = j.e ?? 0;
    const eval_ = (s) => {
      const q = this.kin.inverse([w0[0] + s * (w1[0] - w0[0]), w0[1] + s * (w1[1] - w0[1]), w0[2] + s * (w1[2] - w0[2])],
        phi0 + s * (phi1 - phi0), j.j1);
      return q && { j1: q[0], j2: q[1], j3: q[2], j4: q[3], table: c, e };
    };
    for (let k = 0; k <= 8; k++) if (!eval_(k / 8)) { this.msg('error: tool change move unreachable'); return; }
    const len = Math.hypot(w1[0] - w0[0], w1[1] - w0[1], w1[2] - w0[2]) + Math.abs(phi1 - phi0) * 0.05;
    this.runMove(eval_, len, (F ?? 3000) / 60 / 1000, this.cfg.accel, t, () => this.syncPosFromJoints());
  }

  // Trapezoidal velocity profile along a path parameter s in [0,1], starting
  // at the speed the previous move handed over and ending at vEnd (m/s).
  runMove(eval_, len, vmax, amax, t, onDone, vEnd = 0) {
    if (!this.enabled) this.enable(true);
    const v0m = this.vCarry ?? 0;
    const tCarry = this.tCarry;
    this.vCarry = 0; this.tCarry = null;
    if (len < 1e-9) { onDone?.(); return; }
    // Limit speed so no joint exceeds its maximum rate.
    const dqds = {};
    for (let k = 1; k <= 8; k++) {
      const q = eval_(k / 8), qp = eval_((k - 1) / 8);
      for (const n of Object.keys(q)) dqds[n] = Math.max(dqds[n] ?? 0, Math.abs(q[n] - qp[n]) * 8);
    }
    let v = vmax / len;
    const a = amax / len; // everything below in units of s (path fraction)
    for (const [n, d] of Object.entries(dqds)) {
      const wmax = this.cfg.maxJointSpeed[n];
      if (wmax && d > 1e-9) v = Math.min(v, wmax / d);
    }
    const v0 = Math.min(v0m / len, v);
    let v1 = Math.min(vEnd / len, v, Math.sqrt(v0 * v0 + 2 * a));
    const vp = Math.max(v0, v1, Math.min(v, Math.sqrt(a + 0.5 * (v0 * v0 + v1 * v1))));
    const d1 = (vp * vp - v0 * v0) / (2 * a), d3 = (vp * vp - v1 * v1) / (2 * a);
    const d2 = Math.max(0, 1 - d1 - d3);
    const t1 = (vp - v0) / a, t2 = d2 / vp, t3 = (vp - v1) / a;
    const T = t1 + t2 + t3;
    const sAt = (tt) => {
      if (tt <= 0) return 0;
      if (tt >= T) return 1;
      if (tt < t1) return v0 * tt + 0.5 * a * tt * tt;
      if (tt < t1 + t2) return d1 + vp * (tt - t1);
      const r = tt - t1 - t2;
      return Math.min(1, d1 + d2 + vp * r - 0.5 * a * r * r);
    };
    // A blended move starts exactly where the previous one ended in time.
    const t0 = v0 > 0 && tCarry != null && tCarry <= t + 1e-9 && tCarry > t - 3 * TICK ? tCarry : t;
    this.stats.moves++;
    this.current = {
      update: (tt) => {
        const s = sAt(tt + TICK - t0);
        const q = eval_(s);
        if (q) this.stepTo(q, tt);
        if (tt + TICK - t0 >= T) {
          if (v1 > 0) { this.vCarry = v1 * len; this.tCarry = t0 + T; }
          onDone?.();
          return true;
        }
        return false;
      },
    };
  }

  // Schedule step pulses so each axis reaches its target at the end of this tick.
  stepTo(q, t) {
    for (const [name, angle] of Object.entries(q)) {
      const ax = this.axes[name];
      if (!ax || angle === undefined) continue;
      const target = Math.round(angle * ax.spr) + ax.zero;
      this.emitSteps(ax, target - ax.pos, t);
    }
  }
  emitSteps(ax, n, t) {
    if (n === 0) return;
    const m = this.mcu;
    const dir = n > 0;
    let t1 = t;
    if (dir !== ax.dirLevel) { m.write(ax.dir, dir); ax.dirLevel = dir; t1 += 1e-6; }
    const count = Math.abs(n);
    this.stats.maxStepsPerTick = Math.max(this.stats.maxStepsPerTick, count);
    const span = TICK - (t1 - t) - 2 * STEP_PULSE;
    const inc = dir ? 1 : -1;
    for (let k = 0; k < count; k++) {
      const ts = t1 + ((k + 0.5) * span) / count;
      m.timerAt(ts, () => { m.write(ax.step, true); ax.issued += inc; });
      m.timerAt(ts + STEP_PULSE, () => m.write(ax.step, false));
    }
    ax.pos += n;
  }

  // ------------------------------------------------------------ homing
  onSwitch(ax) {
    if (ax.seeking && !ax.triggered) { ax.triggered = true; ax.trigIssued = ax.issued; }
  }
  startHoming(t) {
    this.enable(true);
    this.homed = false;
    const order = [...this.cfg.homingOrder];
    let phase = null;
    const next = () => {
      const name = order.shift();
      if (!name) return false;
      const ax = this.axes[name];
      phase = { ax, stage: 'seek', v: ax.home.dir * 12 * DEG, travel: 0, cmd: (ax.pos - ax.zero) / ax.spr };
      if (!this.mcu.read(ax.home.pin)) { phase.stage = 'backoff'; phase.travel = 0; }
      ax.triggered = false; ax.seeking = phase.stage === 'seek';
      return true;
    };
    next();
    this.current = {
      update: (tt) => {
        if (!phase) return true;
        const { ax } = phase;
        if (phase.stage === 'seek' || phase.stage === 'slow') {
          if (ax.triggered) {
            if (phase.stage === 'seek') {
              phase.stage = 'backoff'; phase.travel = 0; ax.seeking = false;
            } else {
              // Define zero: at the trip point the joint is at the nominal trip angle.
              ax.zero = ax.trigIssued - Math.round(ax.home.trip * ax.spr);
              ax.seeking = false;
              this.msg(`homed ${ax.name}`);
              if (!next()) {
                this.homed = true;
                this.gotoRest(tt); // replaces this.current with the move to the rest pose
                return false;
              }
            }
          } else {
            const v = phase.stage === 'seek' ? phase.v : phase.v / 6;
            phase.travel += Math.abs(v) * TICK;
            if (phase.travel > 360 * DEG) { this.msg(`error: homing ${ax.name} failed, switch not found`); return true; }
            phase.cmd += v * TICK;
            this.stepTo({ [ax.name]: phase.cmd }, tt);
          }
        } else if (phase.stage === 'backoff') {
          const v = -phase.v / 2;
          phase.travel += Math.abs(v) * TICK;
          phase.cmd += v * TICK;
          this.stepTo({ [ax.name]: phase.cmd }, tt);
          if (phase.travel > 4 * DEG && this.mcu.read(ax.home.pin)) {
            phase.stage = 'slow'; ax.triggered = false; ax.seeking = true; phase.travel = 0;
          }
        }
        return false;
      },
    };
  }
  gotoRest(t) {
    const j0 = this.jointAngles();
    const rest = this.cfg.restPose;
    const target = { j1: rest[0], j2: rest[1], j3: rest[2], j4: rest[3], table: 0 };
    const eval_ = (s) => {
      const o = {};
      for (const n of Object.keys(target)) o[n] = j0[n] + s * (target[n] - j0[n]);
      return o;
    };
    this.runMove(eval_, 0.1, 0.03, 0.2, t, () => { this.syncPosFromJoints(); this.msg('homing done'); });
  }

  // ------------------------------------------------------------ tool change
  toolChange(n, t) {
    const want = this.cfg.toolOrder[n];
    if (!want) { this.msg(`error: no tool T${n}`); return; }
    if (this.tool === want) return;
    const seq = [];
    const up = 0.045, f = this.cfg.changeFeed;
    const r = this.cfg.rack;
    seq.push('_TIP 0');
    if (this.tool) {
      const h = r[this.tool];
      if (this.tool === 'hotend') this.setHeater('hotend', 0, false, t);
      if (this.tool === 'spindle') this.setSpindle(0, t);
      seq.push(`_MOVEW ${h[0]} ${h[1]} ${h[2] + up} ${f}`, `_MOVEW ${h[0]} ${h[1]} ${h[2]} ${f / 4}`,
        '_SERVO unlock', 'G4 P600', `_TOOL none`, `_MOVEW ${h[0]} ${h[1]} ${h[2] + up} ${f / 4}`);
    }
    const h = r[want];
    seq.push(`_MOVEW ${h[0]} ${h[1]} ${h[2] + up} ${f}`, `_MOVEW ${h[0]} ${h[1]} ${h[2]} ${f / 4}`,
      '_SERVO lock', 'G4 P600', `_TOOL ${want}`, `_MOVEW ${h[0]} ${h[1]} ${h[2] + up} ${f / 4}`,
      `_TIP ${this.cfg.tipOffset[want]}`);
    this.queue.unshift(...seq);
  }

  // ------------------------------------------------------------ heaters
  setHeater(name, tempC, wait, t) {
    const h = this.heaters[name];
    h.target = tempC > 0 ? tempC : 0;
    h.integ = 0; h.fault = null;
    h.runaway = { t, T: h.temp };
    if (wait && h.target > 0) {
      this.current = { update: () => h.fault !== null || Math.abs(h.temp - h.target) < 2 || h.target === 0 };
    }
  }
  readTemp(h) {
    const code = this.mcu.adc(h.adc);
    if (code >= 4090) return -273; // open circuit: reads as infinitely cold
    const R = (4700 * code) / (4095 - code);
    return 1 / (1 / 298.15 + Math.log(R / 100e3) / 3950) - 273.15;
  }
  slowLoop(t) {
    for (const [name, h] of Object.entries(this.heaters)) {
      const present = !h.tool || this.tool === h.tool;
      h.temp = this.readTemp(h);
      let duty = 0;
      if (present && h.target > 0 && !h.fault) {
        if (h.temp < 0) h.fault = 'MINTEMP (thermistor open?)';
        else if (h.temp > h.max) h.fault = 'MAXTEMP';
        if (!h.fault) {
          const err = h.target - h.temp;
          const { kp, ki, kd } = h.pid;
          if (Math.abs(err) < h.band) h.integ = Math.max(-h.iMax, Math.min(h.iMax, h.integ + err * 0.1));
          const d = (err - h.prevErr) / 0.1;
          h.prevErr = err;
          duty = err > h.band ? 1 : Math.max(0, Math.min(1, kp * err + ki * h.integ + kd * d));
          // Thermal runaway: heating hard but temperature not rising.
          if (!Number.isFinite(h.runaway.T)) h.runaway = { t, T: h.temp };
          if (err > 10 && duty > 0.9) {
            if (h.temp > h.runaway.T + 2) h.runaway = { t, T: h.temp };
            else if (t - h.runaway.t > h.runawayTime) h.fault = 'THERMAL RUNAWAY';
          } else h.runaway = { t, T: h.temp };
        }
        if (h.fault) { this.msg(`!! ${name} heater fault: ${h.fault}`); duty = 0; }
      }
      h.duty = duty;
      this.mcu.pwm(h.pin, h.pwmHz, duty);
    }
    // Spindle soft start.
    const sp = this.spindle, goal = sp.target / this.cfg.spindle.maxRpm;
    sp.duty += Math.max(-0.2, Math.min(0.08, goal - sp.duty));
    if (sp.duty < 0.02 && goal === 0) sp.duty = 0;
    if (this.tool !== 'spindle') sp.duty = 0;
    this.mcu.pwm(this.cfg.spindle.pin, this.cfg.spindle.pwmHz, sp.duty);
  }
  setSpindle(rpm, t) {
    this.spindle.target = Math.max(0, Math.min(this.cfg.spindle.maxRpm, rpm));
    if (rpm > 0 && this.tool !== 'spindle') this.msg('warning: spindle commanded but no spindle tool attached');
    const tEnd = t + (rpm > 0 ? 1.5 : 0.8);
    this.current = { update: (tt) => tt >= tEnd };
  }
}
