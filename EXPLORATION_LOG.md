> Non-binding exploration log. Records what was tried and why it was dropped. Not read without a request.

# Exploration log

## Session 1 (2026-09-28)

**Language and runtime.** Plain JavaScript ES modules, no build step, running in both Node (headless, tests) and the browser (viewer). Python with numpy was considered for the physics but dropped: numpy was not installed, and a browser-runnable sim is easier for learners to jump into. JavaScript's V8 engine is also much faster than CPython for the scalar inner loops used here.

**Arm layout.** A 4-joint arm (yaw, shoulder, elbow, wrist pitch) plus the turntable gives exactly 5 controllable axes: tool position (3) and tool axis direction (2). A 6-joint arm was considered and dropped for the demo. It would be redundant with the turntable, and a shorter joint chain is stiffer for machining.

**Electrical solver.** A single general circuit solver (modified nodal analysis) over the whole electrical system at microsecond steps was considered. It was dropped because it was too slow in JavaScript (about 100 nodes at 1 µs). It was replaced by three parts:
- a nodal solver for the DC supply network only, whose matrix factorisation is cached;
- series loops traced through real wires for each load (coils, heaters, spindle), integrated exactly;
- an averaged chopper model for the stepper drivers, which applies the mean effect of each 25 µs switching period instead of every switching edge.

**Event timing vs step grid.** At first, all events were quantised to the 25 µs step. That is fine for stepper drivers, because the chopper period is 25 µs anyway. It was too coarse for PWM on the MOSFET modules: a 1 kHz spindle PWM would get only 12.5 % duty resolution. The MOSFET module now integrates its load loop piecewise between exact gate transition times.

**Bugs found by running the whole machine (kept for the record):**
- USB lines arrived out of order: shorter lines overtook longer ones in the same 1 ms frame. Transfers are now serialised.
- The microstep counter advanced 64 counts per step instead of 16, so every motor moved 4× too far.
- Homing moves slower than half a step per 1 ms tick were rounded away to zero. Homing now accumulates a continuous commanded angle.
- The tool-holder frame used the joint-axis vector (−y) where the wrist link's +y axis belonged, so the tool never seated.
- Digital nets assumed every pin started low. They now start undriven, with each receiver at its pull-resistor level; a driver's EN pin is pulled high, so the motor starts disabled. Receivers also re-read their pins when their logic supply comes up; without this, drivers whose 3.3 V logic supply (VIO) was still ramping missed the enable edge at boot.

**Drivers disabled at boot → the arm sags.** With back-drivable planetary gearboxes (30:1, 75–80 % efficient), the elbow collapsed onto its hard stop in about 0.3 s. That is physically correct. The firmware now enables the drivers at boot (`holdAtBoot`) to hold the arm. Real arms solve this with brakes or self-locking drives, which is worth modelling later.

**Workpiece representation.** A height map (one height per 0.4 mm cell, in the plate's rotating frame) serves both printing and milling. Voxels and multi-direction height maps (tri-dexel) were considered and deferred on memory and speed grounds. Consequence: no undercuts, and cutting assumes a near-vertical tool.

**Spindle speed.** Open-loop PWM with a nominal 9000 rpm maximum gives about 10,200 rpm at 8000 rpm commanded. This was left as is, because an uncalibrated open-loop spindle behaves the same way. A tachometer is a candidate addition.
