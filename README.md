# multi-arm: a physically anchored robot-arm workshop simulator

> Exploration demo. This README describes the demo as it stands, not decisions about a future build. See `EXPLORATION_HANDOVER.md` for where the work was heading.

A small DIY robotic arm stands next to a rotating build plate. Tools swap on its wrist: a hot end for 3D printing (adding material) and a spindle for milling (removing material). The simulation is built on one rule:

**Nothing happens without a physical thing that makes it happen.** A signal needs a wire. A torque needs a shaft or a gearbox. Heat needs a surface to flow through. A tool's heater only works while its contacts touch the wrist's contacts.

The goal is a sim that is close enough to the real machine that working on it teaches real skills: wiring, drives, firmware, tuning, printing and machining.

## Run it

Requires Node 18 or later. There is nothing to install.

```sh
npm test                     # 29 physics, slicer, CAM and architecture checks (about 30 s)
npm run demo                 # headless: home, heat, print a ring, change tool, mill a slot
node src/headless/run.js scenarios/demo.gcode --preheated   # skip the heat-up
node src/headless/run.js scenarios/mill-wax.gcode --stock wax  # pocket a wax block
node src/headless/run.js scenarios/knob-hybrid.gcode --preheated # print a knob, finish its dome
node src/headless/run.js scenarios/knob-machined.gcode          # carve the knob from wax
node tools/gen-parts.js      # regenerate the knob STL and its two jobs
npm run serve                # then open http://localhost:8080/
```

In the viewer, press **Load demo**, then **Run** (space bar also toggles). Click any wire or component, or pick one in the Parts tab, to see its live values, its ports and what they connect to, and a plain-language note on what physics it carries. Other controls:
- **colour wires by current**: each wire lights up with the current it carries.
- **stock**: puts a wax, foam or pine block on the plate.
- **Scope**: plots any four recorded signals (bus voltage, coil currents, gearbox wind-up, tip error, temperatures, extrusion force, spindle current and more).

The physics runs in the page. On a normal machine it keeps up with real time at the "max" speed setting.

## How the model is built

### The physical-anchor graph (`src/core/graph.js`)
- A **Component** is a physical object: a motor, a driver board, a heater block. It is mounted on a **Body** (the bench, an arm link, the plate, a tool) at a position, so it has a place in the world.
- A component has **Ports**: terminals, shafts, thermal faces. Ports carry a physical domain:
  - electrical: voltage and current;
  - rotational: angle, speed and torque;
  - thermal: temperature and heat flow;
  - material: plastic in, chips out.
- A **Connection** joins two ports and is itself physical: a wire of a given gauge following a route along the arm, a gearbox, a thermal contact, a cam on a switch, a pogo pin. A wire's length comes from its route, so its resistance comes from its length.
- Components never reference each other. `Assembly.validate()` reports any unmounted part or floating port, the way a floating input pin would misbehave on a real board.

### Time
Two kinds of time run together, as in real mixed-signal hardware:
- **Exact events:** a pin switching, a timer interrupt, a logic edge arriving at the far end of a wire after RC charging and flight time, a USB frame.
- **Fixed physics steps:**

| Rate | What runs |
|---|---|
| 25 µs (one stepper-chopper period) | Supply network, coil currents, motor rotors, gearboxes, arm and plate dynamics |
| 200 µs | Kinematics, arm mass matrix, tool changer, limit switches |
| 1 ms | Deposition and cutting, ADC filters, the data recorder |
| 10 ms | Heat flow, wire and motor heating |

### What each domain models

| Domain | Plain description | Where |
|---|---|---|
| Supply network | Every supply terminal is a node and every power wire a resistor. Solving the network each step gives each board its real voltage, including drop along wires and ground offsets where motor current returns through logic ground wires. The PSU cannot sink current, so braking motors pump the bus up. | `electrical/dcnetwork.js`, `electrical/power.js` |
| Circuits | Each load (motor coil, heater, spindle) is a series loop traced through the actual wires, connectors, slip ring and tool-changer pins. Its current follows the loop's resistance, inductance and back-EMF exactly. | `electrical/circuit.js` |
| Logic signals | A pin charges the wire and the input's capacitance. The receiver sees the edge when the voltage, measured against its own ground, crosses its threshold. Pull resistors set undriven levels, and unpowered logic ignores its inputs. | `electrical/digital.js` |
| Stepper drives | The driver advances a sine table on each STEP edge and regulates coil currents within the supply voltage. At speed the motor's back-EMF eats the voltage headroom and torque falls. Also modelled: detent torque, winding heating, and driver chip overheating. | `electrical/stepper.js` |
| Arm mechanics | Newton–Euler rigid-body dynamics. Link inertias are summed from the parts actually mounted on each link. Each joint is driven through a gearbox with stiffness, backlash and load-dependent friction, and each motor rotor is its own moving mass behind that gearbox. | `mechanical/arm.js`, `mechanical/transmission.js` |
| Heat | Lumped thermal masses joined by contact conductances: heater cartridge, heater block, thermistor bead, melt zone, bed plate, room air. | `thermal/thermal.js`, `electrical/devices.js` |
| Extrusion | Motor, gears, drive gear gripping the filament, the filament as a spring, shear-thinning and temperature-dependent melt flow, heat taken by melting, and gear slip. Pressure lag and oozing come out of these on their own. | `process/extruder.js` |
| Deposition and cutting | Both act on one height map in the plate's rotating frame. Plastic spreads under the nozzle with volume conserved. A spinning cutter removes material, and cutting power = specific energy × removal rate. The cutting force pushes back on the arm (through its gearboxes) and on the plate. | `process/workpiece.js`, `process/contacts.js` |
| Tool changer | A servo turns a latch; a tool locks only if seated within capture range. Pogo pins open when the tool is released, and every circuit is then re-traced. | `mechanical/toolchanger.js` |
| Controller | A 3.3 V microcontroller board with GPIO, 8 ns timer compare, hardware PWM, a 12-bit ADC with noise, pull-ups, and USB in 1 ms frames. | `mcu/board.js` |
| Firmware | G-code over USB and a 1 kHz servo tick. It runs inverse kinematics and schedules step pulses on hardware timers. Its planner looks one move ahead and limits every joint's speed, acceleration and corner speed jump, so the steppers can follow. Also: homing on limit switches, heater PID with thermal-runaway protection, polar mode (the plate turns so the arm stays in its own plane), spindle soft-start and tool-change sequences. It knows only the nominal geometry. | `firmware/` |

The machine itself (every part, pin assignment, wire, gauge, colour and route) is assembled in `src/machine/build.js`. Nominal dimensions and part choices are in `src/machine/spec.js`. The built machine differs from them by seeded manufacturing tolerances, so the firmware's picture of the arm is slightly wrong, as on a real build.

### From a model to a finished part (`src/cam/`)
A job starts from a 3D model (an STL triangle mesh) and becomes G-code for the firmware.
- **Slicer** (`slicer.js`), for printing. It cuts the mesh into layers and traces two walls inside each outline. It hatches the inside: solid near the top and bottom of the part, 25 % sparse elsewhere. It orders the paths, adds retracted travel moves, and works out how much filament each move pushes. The option `sliceAt: 'bottom'` prints up-facing surfaces slightly oversize, which leaves material for a finishing cut.
- **CAM** (`cam.js`), for the spindle. It uses a "drop cutter": for each point, the cutter is lowered onto a height map of the part's top surface until it touches, which gives the lowest height it can go without cutting into the part. Finishing is one closely spaced raster pass. Roughing takes the same passes in horizontal slabs, leaving a 0.3 mm allowance for finishing.
- **Jobs** (`tools/gen-parts.js` writes them):
  - `scenarios/knob.stl`: the demo part, a fluted, dome-topped knob.
  - `scenarios/knob-hybrid.gcode`: prints the knob slightly oversize, then finishes the dome with the ball-nosed cutter.
  - `scenarios/knob-machined.gcode`: carves the knob from a wax block.
  - A job can declare its stock in a comment, for example `; @stock wax 24 24 8 -30 0` (material, size in mm, centre in mm).
- In the viewer, **Open STL…** slices your own model in the page and loads it as a print job. The planned path is drawn on the plate: orange for extrusion, violet for cutting, blue for travel.

### G-code dialect (demo)
- `G0` / `G1`: X Y Z in mm on the plate (origin at the plate centre, Z above the plate top). C is the plate angle in degrees, A the tool tilt in degrees, E extrusion in mm, F feed in mm/min.
- `G28` home, `G4` dwell, `G90` / `G91` absolute / relative XYZ, `M82` / `M83` absolute / relative E, `G92 E`.
- `M104` / `M109` hot end temperature (set / set and wait), `M140` / `M190` bed temperature.
- `M3 S<rpm>` / `M5` spindle on / off, `M6 T0|T1` tool change (hot end / spindle), `M114` report position.
- `M620` / `M621` polar mode on / off.

## Architecture: layers and seams

The code is layered so each part could be replaced by an implementation in another language.

| Layer | What it is | Where |
|---|---|---|
| Description | The drawings and wiring diagram: parts, where they sit, how they connect. Exportable as plain data (`node tools/export-model.js` writes `out/model.json`). | `machine/spec.js`, `machine/build.js`, `core/export.js` |
| Machine kernel | Components and connections with declared parameters and state. Ports carry fixed fields per domain. The step order is data (`SCHEDULE`), and machine-side events are data. The full state can be snapshotted, restored and fingerprinted (`core/state.js`). | `core/`, `electrical/`, `mechanical/`, `thermal/`, `process/`, `machine/machine.js` |
| Controller seam | The microcontroller's pins, timers, PWM, ADC and USB. The only way in or out for firmware. | `mcu/board.js` |
| Firmware | Runs only against the controller seam. | `firmware/` |
| Tools and viewer | Slicer and CAM (offline), headless runner, browser viewer. | `cam/`, `headless/`, `web/` |

The checks in `test/architecture.test.js` guard this layering:
- every part declares its parameters and state;
- ports carry only their domain's fields;
- runs are deterministic, and snapshot and restore are exact;
- the export is complete;
- a recorded reference trace (`test/golden/`) pins the physics numerically.

A port to C, C++ or Rust can load `out/model.json` and compare its own state and traces against the same reference. Rerun `node tools/golden.js` only when a physics change is intended.

## Layout

```
src/core        graph (anchor rules), simulator clock, linear algebra, units, state table, model export
src/electrical  wires and loops, supply network, logic nets, PSU, buck, drivers, motors, devices
src/mechanical  arm dynamics, gearboxes, turntable, tool changer
src/thermal     thermal masses and contacts
src/process     workpiece height map, extruder, deposition and cutting contacts
src/cam         meshes and STL, slicer, CAM (drop cutter), demo models, job directives
src/mcu         controller board, MCU peripherals, USB cable, host PC
src/firmware    firmware and nominal kinematics
src/machine     spec (the drawings), build (the wiring diagram), machine (step order, recorder)
src/headless    command-line runner
web/            browser viewer: app (loop, controls), view3d (scene), inspector, scope, docs (plain-language notes); three.js vendored
tools/          serve.js (local static server), gen-demo.js (writes scenarios/demo.gcode)
test/           physics checks
```
