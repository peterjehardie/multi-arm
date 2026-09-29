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

**Planner.** The first planner stopped fully at the end of every G-code segment (exact stop). It was replaced by a one-move look-ahead: corner speed is set by junction deviation (0.02 mm) and capped so the next move can still stop within its own length. A full multi-move backward pass was not attempted.

**Polar-mode move length.** The derived plate rotation had been counted in the move length (at a 100 mm radius), so polar prints ran about 8× slower than the programmed feed. It was also counted as a commanded rotation, which disabled blending. Polar moves are now timed by the tool path alone. The demo went from 389 s to 216 s.

**Thermal-runaway seed.** Runaway protection was seeded with a temperature that had not been read yet (NaN). As a result, normal bed heating tripped a false fault after 120 s. It is now seeded on the first valid reading.

**Supply-network lookups.** Node voltages were looked up by building a string key on every call, which took about 11 % of run time. The node index is now cached on the port, making the sim about 25 % faster.

**Power-on inrush.** Without a soft start, the PSU charged about 5000 µF instantly, showing roughly 1400 A for one step. A 20 ms output ramp, as real switch-mode supplies have, brings the peak to about 12 A. Side effect: the MCU now boots about 5 ms after power-on, and a G-code line sent earlier was lost. The host now waits for the board's `start` message, the way a USB serial port only appears once the device runs.

**Command buffer.** The firmware accepted the whole G-code file at once, because it acknowledged every line immediately. It now holds 16 lines and withholds `ok` while full, which paces the host as real firmware does.

**Viewer.** Physics runs in the page (requestAnimationFrame with a 12 ms budget per frame). Headless Chromium with software rendering manages only about 0.2× real time, which is a rendering limit, not a physics one. A Web Worker split remains an option.

## Session 2 (2026-09-29)

**Model → part pipeline.** A slicer and a CAM module were added in `src/cam/`, together with a revolved demo knob written out as an STL file.
- The slicer offsets outlines by moving vertices along their averaged normals, instead of using a polygon-clipping library. It detects top and bottom skin from the part's top-surface height map.
- Sliced outlines first carried tiny zig-zags where the cutting plane grazed triangle corners. Outlines are now cleaned before being offset.
- The hybrid job slices each layer at its lower face, so up-facing surfaces print slightly oversize and the finishing cut has material to remove.
- The spindle's cutter was changed from a flat end mill to a ball-nosed one, so that one tool can both rough and finish curved surfaces.

**Roughing bug.** The slab loop stopped one slab early and left the lowest 1.6 mm of the block uncut outside the finishing area. It was fixed before any results were used.

**Job length.** At 0.6 m/s² acceleration, a 27-layer knob made of half-millimetre segments ran at roughly 10 G-code lines per second, about 15 minutes of machine time. The planner's acceleration was raised to 1.5 m/s² (hobby-printer range, well within the arm's torque margin) and the knob was shrunk to 16 mm across.

**Performance measurements (this container, Node 22):**
- Material removal on the height map costs 2.8 µs per update, 1000 updates per second, which is 0.29 % of run time while cutting. The material representation is not the bottleneck; the 25 µs physics step is.
- The same physics kernel (6 stepper channels plus the 37-node supply network, 10⁶ steps = 25 s simulated) took 3.4 s in JavaScript, 1.8 s in C at `-O2` and 1.5 s at `-O3 -ffast-math`. Native code is about 2× faster here.

**Plate stall in off-centre polar printing (found by running the full machine).** In the first full hybrid run the printed knob came out smeared and displaced. A trace showed every step pulse reaching the plate's driver while the plate fell behind: 11° by the first layer change, 77° by 160 s. The stepper was stalling. The planner limited acceleration only along the tool path, and off the plate centre polar mode turns small tool-path changes into large plate-speed changes. Three causes were found and fixed:
1. **No per-joint limits.** Moves are now also limited by each joint's speed, by its acceleration along the path, by the acceleration caused by path curvature (d²q/ds²·ṡ²), and by the largest sudden speed change a joint may take at a corner.
2. **A move ending inside a tick left the rest of that tick unused, and the next tick caught up** with up to 2 ms of motion in 1 ms. Moves now hand over to the next one within the same tick. A second bug refused the handover time when it lay inside the current tick; that window was widened by one tick.
3. **Corner speeds used the tool-path acceleration**, although the next move might be allowed less by its joints, so it could not stop in time before a retraction.

Result from the planner-only check: plate peak speed 84°/s (limit 90), worst per-tick speed change 3 step-quanta, where it had been 54. `test/firmware.test.js` fails on the old planner (172°/s demanded) and passes now. The arm-only tip error had not shown any of this, so a part-frame error, which includes the plate angle, was added.

**Top skin fell into sparse infill.** In the first full-grid hybrid run the finished dome showed a crosshatch of the sparse infill underneath. The deposition model let a top-skin bead run down into infill gaps like a liquid, so the skin came out thin. Beads now bridge any drop deeper than about one bead thickness (0.35 mm). A height map cannot store the hollow under a bridge, so such columns count as filled. Also, plastic that oozed while the nozzle hovered was being dropped into a single cell, making a 9 mm spike; it now spreads over the patch under the nozzle.

**Final knob runs (0.25 mm grid):**
- Hybrid: 582 s of machine time. 811 mm³ extruded = 811 mm³ deposited; the finishing cut removed 155 mm³. Mean arm tip error 0.41 mm; part-frame error peaked at 1.9 mm in one transient, not traced yet.
- Machined: 334 s, 3051 mm³ of wax removed, part-frame error under 0.9 mm.
- Both run at about 0.93–0.95× real time in Node, with two simulations sharing the CPU.

## Session 3 (2026-09-29)

**"Not much happens after Run."**
- Every job opened with about 20 s of homing (one joint at a time, a few degrees each) and heating, and the page gave no feedback while it happened.
- The physics also ran at only about 0.6× real time in a browser.
- Added a quick start: the firmware takes the joints as already referenced at the rest pose, as an arm with absolute encoders would. `G28` is then skipped. Without switch referencing the tip error rises to about 1 mm, as expected.
- The firmware now reports its activity in words.
- The viewer has a job card (activity, layer or pass, progress, time estimate), view presets including one that follows the tool, and switches to max speed when a job loads.

**Physics speed-up, about 2.7× during printing (Node: 1.31 s → 0.44 s of computing per simulated second):**
- The supply network is solved every 100 µs instead of every 25 µs; its voltages move on millisecond time scales.
- Loop resistance and inductance are cached, refreshed every 10 ms on the thermal stage and whenever circuits are re-traced.
- Each driver caches its current-decay factor and its port references.
- Each motor computes the sine and cosine of its rotor angle once per step. They are shared by back-EMF and torque, with the detent term from double-angle identities.
- Idle MOSFET modules skip their update.
- Arm dynamics refresh every 500 µs instead of every 200 µs.

All three jobs re-ran with the same results within grid differences. They now simulate at 2.3–2.4× real time headless.

## Session 4 (2026-09-29)

**Architecture for porting.**
- Added typed port fields, declared parameters and state on every class, a data-driven step schedule, typed machine-side events, a state table with snapshot, restore and hash, a model export and a golden reference trace.
- The new checks at once found three state values with no starting value (buck output voltage, a driver's logic-power flag, the servo's pulse timer) and a stray `inertia` field on the motor shaft port. All are fixed.
- The state hash was identical before and after the refactor (60568afed06941a9 for the 3 s quick-start ring demo), so no physics changed.
- The mounted part's inertia was briefly wired from the workpiece straight to the turntable. That was moved to the assembly level, because components must not reference each other.

## Session 5 (2026-09-29)

**Enclosure and touch probe.** Added an enclosure (chamber air as one thermal mass, a door switch wired to the controller, an exhaust fan behind a MOSFET) and a touch probe tool (T2). The firmware got a door interlock for the spindle, `G38.2` probing and `M106`/`M107`. With a 60 °C bed and nothing else heating it, the closed chamber warms by only about 2 K in 10 minutes: about 11 W from the bed against about 7 W/K of panels, with a time constant near 750 s. Radiation is not modelled.

**Demo mode.** Profiling showed the 25 µs step costs most of the time; the ~14,000 events per second (step pulses, logic edges) are cheap in comparison. The first idea was to disable parts of the machine; it was dropped because it would break the anchor rule. Demo mode instead keeps every part, wire and step pulse and replaces the fast physics with ideal versions on a 1 ms step:
- the driver sets the coil currents exactly;
- the rotor follows the current vector;
- the gearboxes are rigid;
- the arm and plate follow kinematically.

Result: about 20–25× real time headless instead of 2.2×. Once the steps are 40× fewer, the events dominate the cost. A 1 ms step was checked against the fastest remaining dynamics: the extruder melt relaxation (about 12 ms) and the spindle's electromechanical time constant (about 38 ms). Both are stable with explicit steps.

**Operator actions.** The door cannot be opened from G-code; a person does it. Opening it when the PC sends the line was rejected: the PC runs up to 16 lines ahead of the firmware. The chosen route is `M118 @door open 4`: the firmware prints it when it executes that line, and a scripted operator on the PC side acts on the physical door.

**Tour, full physics vs demo.** The same tour in full physics took 101 s of computing against 8.8 s in demo mode. In full physics the 1 mm pocket came out 2.0 mm deep, and the probe read the block 0.56 mm high and the plate 0.46 mm high. The cause is the arm sagging under the heavier spindle and the lighter probe, without homing (quick start). Demo mode has no sag: pocket 1.07 mm, block top 8.01 mm, plate 0.03 mm.
