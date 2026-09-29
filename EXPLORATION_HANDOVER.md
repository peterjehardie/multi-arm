> Non-binding exploration handover. Records where the work was heading when the session stopped. No rules.

# Handover: session 1 (2026-09-28)

## State when the session stopped
- A working demo exists. The physics core is in `src/`, a headless runner in `src/headless/run.js`, a browser viewer in `web/`, and 18 physics tests in `test/`.
- `scenarios/demo.gcode` runs end to end in about 216 s of simulated time, at about 1.5× real time in Node. It homes the arm, picks up the hot end, prints a 4-layer ring with the plate turning (polar mode), changes to the spindle, mills a slot through the ring, and returns the tool. The tip error stays around 0.4 mm, coming from tolerances and gravity wind-up.
- The core idea in code: components only touch their own ports. Energy and information move only through connections that exist physically: wires, gearboxes, thermal contacts, cams and linkages, pogo pins, a slip ring, and process contacts. `Assembly.validate()` reports floating ports and unmounted parts.

## Where the work was heading

### 1. The route to "near-100 % transfer"
The premise is that accurate enough physics makes the learning transfer. The demo is not yet evidence for that. The route that was being considered:
1. **Component bench checks.** Measure a real motor, driver, heater block and extruder on a bench, then compare against the matching sim part. Measurements: step-response current waveforms, torque–speed curve, heat-up curve, extrusion force against flow. The test files show the pattern, but they compare against datasheets, not measurements.
2. **Run the real firmware against the plant.** Today the firmware is JavaScript written to the MCU's peripheral interface. The strongest transfer claim would come from running the actual compiled C firmware in an MCU emulator (Renode, QEMU or simavr), with its pins wired to this plant. Then the code a learner writes is the code that runs on the real board.
3. **Build one physical machine and record the same traces** (currents, joint encoders, temperatures, tip position by dial gauge or camera). Close the gaps parameter by parameter.
4. **Learner study.** Find out whether people who learned on the sim do better on the real machine. This is the only direct test of the premise.

### 2. Physics not yet modelled (candidate list, roughly by expected impact on realism)
- **Mechanical:** bending of the links as beams; bearing play; cable-harness drag and stiffness torque on the joints; periodic gearbox transmission error; thermal growth of the links; collisions (arm against table, part, rack); motor rotor gyroscopic terms; brakes or self-locking drives for power loss.
- **Drives:** chopper current ripple and actual switching; StealthChop (voltage-mode) versus SpreadCycle; MicroPlyer step interpolation; stall detection (StallGuard); TMC register configuration over UART; joint encoders for closed-loop position.
- **Electrical:** wire inductance in the supply network (voltage spikes on load steps); crosstalk and noise from motor wires into signal wires; fuses and polyfuses; PSU control-loop dynamics; mains input; contact wear and oxidation.
- **Thermal:** heat-break and heatsink path with a fan; part-cooling fan; bed thermal gradient (a single lump today).
- **Additive:** die swell, bead cooling and shrinkage, layer adhesion as a function of temperature, warping, first-layer squish against bed height error, stringing and oozing (partly emerges already).
- **Subtractive:** per-tooth cutting forces (a Kienzle-type model), chip thinning, tool deflection and breakage, chatter (it needs the arm's stiffness, which is already there), tilted-tool cutting (needs a better workpiece representation than a height map).
- **Workpiece:** tri-dexel or sparse voxels for undercuts and 5-axis work.

### 3. Software structure next steps that were being weighed
- Move the physics into a Web Worker, so the viewer stays smooth when the sim runs slower than real time.
- Viewer hooks still missing on the core side: a `shape` hint on components (the viewer guesses discs and cylinders); a real place for room air (convection lines are drawn as stubs); ever-increasing indices on the plant and host logs; and a sample counter on the recorder, so the scope can skip redraws.
- Multi-move look-ahead (a backward pass over the whole queue). Only one move ahead is blended today.
- Calibration exercises for learners: measuring switch offsets and link lengths, bed height mapping, PID autotune.
- Fault injection: a loose crimp, a dirty pogo pin, an undersized wire, a failed thermistor, lost steps.
- Scenario library: bed-heating power budget, ground-loop demonstration, resonance of the arm, milling chatter.
- Saving and loading a run (state snapshot) for teaching.

### 4. Open questions for the user
- Target real hardware family for the controller (RP2040, STM32 or ESP32). This decides which emulator route is realistic.
- Whether joints should get encoders (closed loop) or stay open-loop steppers as in most DIY builds.
- Which materials to prioritise for machining (foam, wax, wood, printed PLA, aluminium).

# Handover addendum: session 2 (2026-09-29)

## State
- `src/cam/` holds an STL reader and writer, a slicer, drop-cutter CAM (ball or flat cutter; roughing slabs and finishing raster) and job directives (`; @stock ...`).
- `scenarios/knob.stl` is the demo part. It has two jobs: `knob-hybrid.gcode` (print, then finish the dome) and `knob-machined.gcode` (carve from wax).
- The viewer has a job picker, an **Open STL…** button (slices in the page) and a toolpath overlay on the plate.

## Where this was heading: material representation and speed
- **Current:** a height map (one height per 0.4 mm cell). It is enough for printing and for 3-axis milling from above. It cannot hold undercuts, and it cannot take a cut from a tilted tool.
- **Next step if 5-axis or undercuts are wanted:** tri-dexel, which is three height-map-like grids of rays along X, Y and Z. Each ray stores the intervals where material exists. Memory is about 3·N² intervals rather than N³ voxels, cuts and deposits are interval operations along the rays under the tool, and a mesh for display can be rebuilt locally around the tool. The measured 0.29 % share for material work leaves room for a representation 10–30× more expensive.
- **Alternatives considered:**
  - Dense voxels: 0.2 mm over 100 mm is 125 million cells, too much memory.
  - Sparse voxels or a narrow-band distance field (an OpenVDB-style store): elegant, but more machinery.
  - Mesh booleans: exact, but slow and fragile under thousands of small cuts per second.
- **Speed route if needed:**
  1. Move the physics into a Web Worker, so rendering never steals its time.
  2. Port the 25 µs hot loop (drivers, coils, rotors, gearboxes, supply network) to Rust or C compiled to WebAssembly. The ~2× native gain measured here should largely carry over in the browser.
  3. Leave the slow stages (thermal, process, firmware logic) in JavaScript.

  Native-only (outside the browser) matters mainly for the MCU-emulator route, which runs natively anyway.

## Planner state (session 2)
The plate stall found in the first hybrid run is fixed in the firmware planner: per-joint limits, handover within a tick, and joint-aware corner speeds (see the log). The part-frame error (`Machine.partError`, scope channel "part-frame error") is the number to watch for print or cut accuracy, because it includes the plate angle. The material grid is now 0.25 mm.

# Handover addendum: session 4 (2026-09-29)

## Porting: done so far
- Typed ports: each domain has a fixed field list (`PORT_FIELDS`).
- Every class declares `static PARAMS` and `static STATE`. Together these are the struct definitions a native port needs.
- The step order is data: `SCHEDULE` in `machine/machine.js`.
- Machine-side events are typed records (`sim.post`): logic edges and contact bounce. Callbacks remain only on the controller side (firmware timers, USB).
- `core/state.js`: state table (871 scalars and 9 arrays), snapshot, restore and a 64-bit state hash.
- `core/export.js` and `tools/export-model.js`: the whole built machine as JSON: components, connections, circuits, supply network, logic nets, schedule and state layout.
- `test/golden/`: a reference trace (ring demo, quick start, 3 s, 10 signals, final hash) and the architecture tests.

## Porting: still open
- Behaviour still lives in JavaScript classes, with small closures for circuit elements (a motor coil's back-EMF). In C this becomes an element-type enum plus an index into the owner's struct.
- The firmware uses closures and a queue of strings. The intended route is real C firmware on an MCU emulator behind the controller seam, with the JavaScript firmware as the reference behaviour.
- The viewer reads objects directly. A native kernel (for example a Rust or C++ build compiled to WebAssembly, running in a Web Worker) would instead expose the state table by label, which `core/state.js` already defines.
- The dynamics and linear-algebra helpers allocate small arrays. Fixed buffers are natural in a native port.
- Bit-exact agreement with JavaScript is not realistic: math-library sine and exponential differ by the last bit, and stepper dynamics amplify tiny differences over long runs. Compare short windows within tolerance, and long runs by outcomes (volumes, errors, timings).
- Candidate order for a port: core (events, schedule, state) → circuits, supply network, drivers, motors → mechanics → heat → process → controller seam.

## Candidate tools (analysis in the session 4 conversation, not decided)
- **Good fit** (low forces, reuses existing physics):
  - touch probe for measuring, bed mapping and self-calibration, the biggest accuracy gain;
  - paste, glue or silicone dispenser, which is the extruder model with a syringe;
  - pen, drag knife and scoring;
  - vacuum pick-and-place with a camera;
  - diode laser engraving (needs an enclosure for eye safety);
  - hot-air reflow;
  - heat-set inserts pressed into printed parts.
- **Possible with care:**
  - soldering iron for through-hole joints (needs contact-force control, a solder feeder, flux and tip cleaning; the thermal side, a tip cooling on contact with a pad, fits the existing thermal network);
  - PCB isolation milling and drilling (needs probing of the copper height).
- **Poor fit:** heavy metal milling, welding.

## Side approach (numbers from the model)
- **Tip stiffness:** gearbox and motor holding stiffness in series, 2019 / 3167 / 1680 / 416 N·m/rad for joints 1–4. From above, a 10 N load deflects the tip 0.95 mm radially and 0.29 mm sideways or vertically. With a horizontal tool: 0.20 mm along the tool axis, 0.27 mm sideways and 0.63 mm vertically. Side approach is no softer than top approach, just different.
- **Wrist range:** a horizontal tool needs the wrist at about +120°; the design allows +60°.
- **Plate clearance:** the spindle body (45 mm across) needs its tip at least 22.5 mm above the plate, so side work needs a riser or fixture.
- **What else it needs:**
  1. wider wrist travel (or an added wrist axis);
  2. collision checks (tool bodies against plate, part and rack);
  3. a material model that can hold undercuts (tri-dexel);
  4. tilted-tool CAM, with side drilling as the first and simplest case.
