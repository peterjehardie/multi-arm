# multi-arm: a physically anchored robot-arm workshop simulator

> Exploration demo. This README describes the demo as it stands, not decisions about a future build. See `EXPLORATION_HANDOVER.md` for where the work was heading.

A small DIY robotic arm stands next to a rotating build plate. Tools swap on its wrist: a hot end for 3D printing (adding material) and a spindle for milling (removing material). The simulation is built on one rule:

**Nothing happens without a physical thing that makes it happen.** A signal needs a wire. A torque needs a shaft or a gearbox. Heat needs a surface to flow through. A tool's heater only works while its contacts touch the wrist's contacts.

The goal is a sim that is close enough to the real machine that working on it teaches real skills: wiring, drives, firmware, tuning, printing and machining.

## Run it

Requires Node 18 or later. There is nothing to install.

```sh
npm test                     # 18 physics checks (about 20 s)
npm run demo                 # headless: home, heat, print a ring, change tool, mill a slot
node src/headless/run.js scenarios/demo.gcode --preheated   # skip the heat-up
npm run serve                # then open http://localhost:8080/web/
```

In the viewer, press **Load demo**, then **Run**. Click any wire or component to see its live values and a plain-language note on what physics it carries.

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
| Firmware | G-code over USB and a 1 kHz servo tick. It runs inverse kinematics and schedules step pulses on hardware timers. Also: homing on limit switches, heater PID with thermal-runaway protection, polar mode (the plate turns so the arm stays in its own plane), spindle soft-start and tool-change sequences. It knows only the nominal geometry. | `firmware/` |

The machine itself (every part, pin assignment, wire, gauge, colour and route) is assembled in `src/machine/build.js`. Nominal dimensions and part choices are in `src/machine/spec.js`. The built machine differs from them by seeded manufacturing tolerances, so the firmware's picture of the arm is slightly wrong, as on a real build.

### G-code dialect (demo)
- `G0` / `G1`: X Y Z in mm on the plate (origin at the plate centre, Z above the plate top). C is the plate angle in degrees, A the tool tilt in degrees, E extrusion in mm, F feed in mm/min.
- `G28` home, `G4` dwell, `G90` / `G91` absolute / relative XYZ, `M82` / `M83` absolute / relative E, `G92 E`.
- `M104` / `M109` hot end temperature (set / set and wait), `M140` / `M190` bed temperature.
- `M3 S<rpm>` / `M5` spindle on / off, `M6 T0|T1` tool change (hot end / spindle), `M114` report position.
- `M620` / `M621` polar mode on / off.

## Layout

```
src/core        graph (anchor rules), simulator clock, linear algebra, units
src/electrical  wires and loops, supply network, logic nets, PSU, buck, drivers, motors, devices
src/mechanical  arm dynamics, gearboxes, turntable, tool changer
src/thermal     thermal masses and contacts
src/process     workpiece height map, extruder, deposition and cutting contacts
src/mcu         controller board, MCU peripherals, USB cable, host PC
src/firmware    firmware and nominal kinematics
src/machine     spec (the drawings), build (the wiring diagram), machine (step order, recorder)
src/headless    command-line runner
web/            browser viewer (three.js vendored)
test/           physics checks
```
