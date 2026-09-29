// Firmware motion planner, run on its own against a stub microcontroller:
// does it respect each joint's limits so the steppers can follow?
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Firmware } from '../src/firmware/firmware.js';
import { firmwareConfig } from '../src/machine/machine.js';
import { SPEC } from '../src/machine/spec.js';

function stubFirmware() {
  const mcu = { now: () => 0, pinMode() {}, write() {}, read: () => true, onChange() {}, adc: () => 2000, timerAt() {}, pwm() {}, pwmStop() {}, onUsbLine() {}, usbWrite() {} };
  const fw = new Firmware(mcu, firmwareConfig(SPEC));
  Object.assign(fw, { homed: true, enabled: true, tool: 'hotend' });
  fw.kin.tip = SPEC.tools.hotend.tipOffset;
  fw.pos = { x: 0.03, y: 0, z: 0.005, c: 0, a: 0, e: 0 };
  return fw;
}

test('polar printing off-centre: plate speed and per-tick speed changes stay within what a stepper follows', () => {
  const fw = stubFirmware();
  // A 16 mm circle centred 20 mm off the plate centre, in short segments.
  const job = ['G90', 'M83', 'M620'];
  for (let k = 0; k <= 120; k++) {
    const a = (2 * Math.PI * k) / 120;
    job.push(`G1 X${(20 + 8 * Math.cos(a)).toFixed(3)} Y${(8 * Math.sin(a)).toFixed(3)} E0.02 F1500`);
  }
  const ax = fw.axes.table, D = 180 / Math.PI;
  let k = 0, prev = null, prevV = 0, maxV = 0, maxA = 0, t = 0;
  for (; t < 60; t += 1e-3) {
    while (fw.queue.length < 16 && k < job.length) fw.queue.push(job[k++]);
    fw.tick(t);
    const q = ax.pos / ax.spr;
    if (prev !== null) {
      const v = (q - prev) / 1e-3;
      maxV = Math.max(maxV, Math.abs(v));
      if (Math.abs(v) > 0 || Math.abs(prevV) > 0) maxA = Math.max(maxA, Math.abs(v - prevV) / 1e-3);
      prevV = v;
    }
    prev = q;
    if (k >= job.length && !fw.queue.length && !fw.current) break;
  }
  assert.ok(t < 59, 'job finished');
  const vLim = SPEC.maxJointSpeed.table * D, stepQuantum = (1 / ax.spr) / 1e-6 * D; // one step per tick, as deg/s^2
  assert.ok(maxV * D <= vLim + 2 * stepQuantum * 1e-3, `plate speed ${(maxV * D).toFixed(1)} deg/s vs limit ${vLim}`);
  assert.ok(maxA * D <= 4 * stepQuantum, `largest per-tick speed change ${(maxA * D).toFixed(0)} deg/s^2 (step quantum ${stepQuantum.toFixed(0)})`);
});
