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
