// Simulation clock and scheduler.
//
// Two kinds of time coexist, as in a real mixed-signal system:
//   * Discrete events at exact times: a GPIO pin changing level, a timer
//     interrupt firing, a digital edge arriving at the far end of a wire.
//   * Continuous physics (currents, velocities, temperatures) integrated in
//     fixed steps. The base step is one stepper-driver chopper period
//     (25 us at 40 kHz). A chopper only updates its coil voltage once per
//     period, so the physics really is sampled at this rate in the hardware.
//     Slower processes (heat, material deposition) run in stages that are
//     whole multiples of the base step.
//
// Events are executed in time order. Continuous stages see the state of all
// digital signals as it stands at the start of each step.

import { EventQueue } from './events.js';

export class Simulator {
  constructor({ dt = 25e-6 } = {}) {
    this.dt = dt;
    this.t = 0;
    this.stepCount = 0;
    this.queue = new EventQueue();
    this.stages = []; // { name, every, fn }
    this.eventCount = 0;
  }

  // Schedule fn(t) to run at absolute time t. Returns a handle with cancel().
  at(t, fn, tag) {
    if (t < this.t) t = this.t;
    const ev = this.queue.push(t, fn, tag);
    return { cancel: () => { ev.cancelled = true; }, t };
  }
  after(delay, fn, tag) { return this.at(this.t + delay, fn, tag); }

  // Add a continuous stage executed every `every` base steps (in insertion order).
  stage(name, every, fn) {
    this.stages.push({ name, every: Math.max(1, Math.round(every)), fn });
  }

  // Advance the simulation by `duration` seconds.
  advance(duration) {
    const tEnd = this.t + duration - 1e-12;
    while (this.t < tEnd) this.step();
  }

  step() {
    const tNext = this.t + this.dt;
    // 1. Execute all events that fall inside this step, at their exact times.
    const q = this.queue;
    while (q.size && q.peekTime() < tNext) {
      const ev = q.pop();
      if (ev.cancelled) continue;
      const tSave = this.t;
      this.t = ev.t > tSave ? ev.t : tSave;
      ev.fn(this.t);
      this.eventCount++;
      this.t = tSave;
    }
    // 2. Continuous physics over [t, t+dt].
    const n = this.stepCount;
    for (const s of this.stages) {
      if (n % s.every === 0) s.fn(this.dt * s.every, this.t);
    }
    this.stepCount++;
    this.t = this.stepCount * this.dt;
  }
}
