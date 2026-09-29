// Plain-language explanations for every kind of component and connection in
// the simulation. Each entry says what the physical thing is and what the
// simulation actually models for it. Keys are the `kind` strings used in
// src/ (Component.kind / Connection.kind).

export const DOCS = {
  // ------------------------------------------------------------ components
  stepper: {
    title: 'Stepper motor',
    text: `A motor whose spinning part is a toothed permanent magnet (50 teeth, so 200 full steps per turn). Two coils of copper wire sit around it; the currents in the two coils together set an angle, and the magnet is pulled toward that angle like a compass needle. As the rotor spins it also generates a voltage in the coils (back-EMF) that fights the driver, so at high speed less current gets in and torque drops. The sim also models the magnet's own "detent" cogging, bearing friction, and the coils and case warming up from I²R losses.`,
  },
  driver: {
    title: 'Stepper driver (TMC2209-class)',
    text: `A small board that turns STEP/DIR pulses from the controller into coil currents. Each rising STEP edge advances a counter that indexes a sine table, giving the two coil current targets (microstepping). A "chopper" switches the supply across each coil every 25 µs to hold the current at target, but it can never apply more than the supply voltage, which is where the real torque-versus-speed limit comes from. The sim also checks pulse timing (DIR must settle before STEP), draws power from the 24 V bus, and heats the chip.`,
  },
  controller: {
    title: 'Controller board',
    text: `The microcontroller board that runs the firmware. The firmware can only do what real silicon can: set or read pins, run hardware PWM and timers, read the ADC and talk over USB. A 1 kHz tick plans motion and schedules step pulses; a 10 Hz loop reads the thermistors (through a 4.7 kΩ pull-up and filter capacitor) and runs the heater PID loops. The firmware knows only the nominal drawings, never the true build.`,
  },
  psu: {
    title: 'Power supply (24 V)',
    text: `A mains switch-mode supply that holds 24 V behind a small output resistance, so the voltage sags slightly as current rises. It cannot take current back in, and above its current limit (16 A) it folds into constant-current mode. Every motor, heater and board ultimately draws its energy from here, through real wires.`,
  },
  buck: {
    title: 'Buck converter (24 V → 5 V)',
    text: `A small step-down regulator that makes 5 V for the logic and the latch servo. It draws from the 24 V side whatever power its output delivers, divided by its efficiency (about 88 %). Below about 6 V input it switches off (under-voltage lockout).`,
  },
  terminal: {
    title: 'Terminal block / splice',
    text: `A bar of copper with screw terminals. Every wire landing on the same bar is the same electrical point, so current can flow from one wire into the others. The sim traces circuits straight through it; it has no resistance of its own beyond the wires' crimps.`,
  },
  mosfet: {
    title: 'MOSFET switch module',
    text: `A low-side electronic switch for a load (hot-end heater, bed heater, spindle or exhaust fan). A logic signal on SIG turns the transistor on, connecting the load to ground through a few milliohms. The gate can switch several times inside one physics step (PWM), so the sim integrates the load current piecewise between those exact switching times. The spindle module has a flyback diode that lets motor current keep flowing briefly when it switches off.`,
  },
  heater: {
    title: 'Heater',
    text: `A resistive element (a cartridge in the hot end, a silicone pad under the bed). Current through its resistance makes heat (P = I²R) in its own small mass, and that heat flows into the part it presses against through a thermal contact. Its resistance rises slightly with temperature. Energy is counted for every PWM pulse, so short pulses heat exactly as much as they should.`,
  },
  thermistor: {
    title: 'Thermistor (100 k NTC)',
    text: `A tiny glass bead whose resistance falls steeply as it warms (beta 3950). It is glued into the part it measures, so it has its own small heat capacity and lags behind the real temperature. The controller reads it as a voltage divider against a pull-up resistor; the sim even includes the bead's self-heating from the measuring current.`,
  },
  'limit-switch': {
    title: 'Limit switch',
    text: `A mechanical microswitch pressed by a cam on a joint, used for homing. When the joint angle passes the trip point the contact closes, bouncing open and shut a few times over about a millisecond like a real contact. It has a little hysteresis, and its exact trip angle carries a manufacturing error the firmware does not know about.`,
  },
  servo: {
    title: 'Hobby servo (tool latch)',
    text: `A small RC servo that turns the tool-changer latch. It measures the width of 50 Hz control pulses (1 ms → 0°, 2 ms → 180°) and turns its horn toward that angle at a limited speed. It slows down if its 5 V supply sags, and draws more current while moving.`,
  },
  'slip-ring': {
    title: 'Slip ring',
    text: `Brushes on the fixed frame rubbing on rings that turn with the build plate, so bed heater and thermistor wires can pass onto a plate that rotates without limit. Each ring adds a small contact resistance (20 mΩ) in series with the circuit.`,
  },
  'changer-master': {
    title: 'Tool changer (arm side)',
    text: `The wrist flange that grabs tools. A servo-driven cam locks a tool plate against it (locks above ~70° of horn travel, releases below ~30°), but only if a plate is actually seated within about a millimetre and a few degrees. Spring-loaded pogo pins carry power and signals across. A locked tool's mass is added to the arm's last link, which changes how the arm moves.`,
  },
  'tool-plate': {
    title: 'Tool plate',
    text: `The mating plate on each tool (hot end, spindle, touch probe), with pads for the pogo pins. When parked it sits in its rack holder; when locked it moves with the wrist. If the latch releases it away from its holder, the sim reports the tool as dropped.`,
  },
  extruder: {
    title: 'Extruder and hot end',
    text: `A geared drive wheel pushes 1.75 mm filament into a heated melt zone. The filament between gear and melt acts like a spring: pushing compresses it, and that force is what pushes molten plastic out of the nozzle. Melt flow depends on pressure and on temperature (plastic is far stiffer when cooler, and shear-thinning), and melting takes heat from the block, so fast printing cools the melt. Too much force and the gear slips on the filament ("clicking").`,
  },
  'dc-motor': {
    title: 'Spindle motor (775 brushed DC)',
    text: `A brushed DC motor with an end mill in its collet. Current makes torque, speed makes back-EMF that limits current, and bearing and brush friction slow it down. The cutting load from the workpiece arrives as a braking torque, so the spindle visibly slows and draws more current when it bites into material.`,
  },
  'thermal-mass': {
    title: 'Thermal mass',
    text: `A lump of metal (the aluminium bed plate, the hot-end heater block) treated as one temperature. Its temperature changes by the net heat flowing in through its thermal contacts divided by its heat capacity (J/K). Heaters, thermistors and room air all connect to it only through thermal contacts.`,
  },
  ambient: {
    title: 'Room air',
    text: `The surrounding air, treated as an infinitely large heat sink at a fixed room temperature. Hot parts lose heat to it through convection connections.`,
  },
  turntable: {
    title: 'Turntable',
    text: `The rotating build plate on a bearing, driven by a stepper through a reduction. Its inertia includes the plate, heater and whatever part is standing on it (recomputed as the part grows). Bearing friction and the push-back from printing or cutting forces act on it, so it can lag or be pushed around.`,
  },
  workpiece: {
    title: 'Workpiece',
    text: `The part on the plate, stored as a grid of material columns (0.4 mm cells) in the plate's rotating frame. Printing raises cells under the nozzle: plastic can't rise above the nozzle tip, so it spreads sideways, which sets the bead width. Milling lowers every cell under the cutter's end face. Volume is conserved both ways, so what the extruder pushed out is what the part gains.`,
  },
  arm: {
    title: 'Robot arm',
    text: `Four rigid links joined by revolute joints, each driven through a gearbox. The arm obeys Newton's laws for linked bodies: gravity, how hard each joint is to accelerate (including coupling between joints) and forces at the tool are all computed each step. Link masses are summed from the structure plus every motor and part bolted on. The true link lengths differ slightly from the drawings, so the tip lands where physics says, not where the firmware thinks.`,
  },
  host: {
    title: 'Host PC',
    text: `The computer streaming G-code to the controller over USB. It keeps up to 4 lines in flight and sends the next one each time the firmware replies "ok", just like a real print host.`,
  },
  enclosure: {
    title: 'Enclosure (panels, door, exhaust fan)',
    text: `A box of acrylic panels on an aluminium frame around the arm and the plate. Its air is one lump of heat: the hot end and bed warm it, and it loses heat to the room through the panels, with the exhaust air and, much faster, through an open door. The door has a switch the firmware reads: while the door is open the spindle is not allowed to run (an interlock), and a job that needs it waits until the door is shut. The exhaust fan is a small 24 V motor switched by a MOSFET (M106 on, M107 off); its speed follows its current and takes a second or two to spin up, and the air it moves carries heat and fumes out.`,
  },
  probe: {
    title: 'Touch probe',
    text: `A tool with a stylus on a spring-loaded, three-point seat. When the stylus tip presses on something (the part, the stock, the plate) it lifts off its seat after a few hundredths of a millimetre and its contact closes. The signal reaches the controller through the tool changer's pogo pins. The firmware moves down slowly (G38.2) until the contact closes, stops, and reports where it believes the tip was ("PRB:x,y,z"). Because the firmware only knows the nominal drawings, the reported heights show the machine's own small errors: this is how a real machine measures its bed, its part and itself.`,
  },
  part: {
    title: 'Mechanical part',
    text: `A bracket or rack with no electrical or moving function. It is drawn for context; if it has mass and is mounted on an arm link, that mass counts toward the link's inertia.`,
  },

  // ------------------------------------------------------------ connections
  wire: {
    title: 'Wire',
    text: `A real copper conductor cut to the length of its route (plus slack) through the machine. Its resistance comes from that length, its gauge (AWG) and its temperature, plus a crimp resistance at each end, so a long thin wire drops noticeable voltage. It heats from the current it carries and cools to the air. On signal wires, edges arrive after the wire's capacitance has charged, and a shifted ground reference changes where the receiver sees them.`,
  },
  'usb-cable': {
    title: 'USB cable',
    text: `Full-speed USB between host and controller. Data moves in 1 ms frames, so a line of G-code arrives at the next frame boundary after it is sent, plus its transmission time at 12 Mbit/s. Messages cannot overtake each other.`,
  },
  gearbox: {
    title: 'Gearbox',
    text: `A reduction joining a motor to a joint: the output turns 1/N times per input turn and multiplies the torque. The teeth and shafts bend like a stiff spring, so under load the output lags the motor (wind-up). There is a little free play (backlash) where no torque passes at all, plus friction that grows with the torque carried.`,
  },
  cam: {
    title: 'Switch cam',
    text: `A cam on a joint that presses a limit switch. It reads the joint angle without loading it: the switch sees exactly where the joint really is, not where the firmware thinks.`,
  },
  linkage: {
    title: 'Latch linkage',
    text: `The mechanical link from the servo horn to the tool-changer latch cam. The latch sees the real horn angle, so a slow or under-powered servo means a slow lock.`,
  },
  'sensor-coupling': {
    title: 'Sensor coupling',
    text: `A flag, cam or magnet on a shaft that a sensor reads without adding any load.`,
  },
  'gear-mesh': {
    title: 'Gear mesh',
    text: `The extruder motor's pinion meshing with the big drive gear: a rigid ratio. The filament's push-back reaches the motor divided by the ratio, with mesh losses in whichever direction power flows.`,
  },
  'thermal-contact': {
    title: 'Thermal contact',
    text: `A physical interface where heat moves between two parts (a cartridge pressed into a bore, a thermistor glued in a hole). Heat flows from hot to cold in proportion to the temperature difference: q = G·(Ta − Tb), where G is the contact's conductance in W/K.`,
  },
  convection: {
    title: 'Convection to air',
    text: `Heat loss from a hot surface to the room air, in proportion to how much hotter the surface is than the room. This is what the heater has to keep replacing to hold temperature.`,
  },
  'pogo-contact': {
    title: 'Pogo pin contact',
    text: `A spring-loaded pin on the wrist pressing on a pad on the tool plate. When a tool is locked on it conducts with a few tens of milliohms; when the tool is released it opens, so a parked tool's heater or motor is simply an open circuit. Every circuit through the changer is re-traced after each lock or release, and the sim tracks time spent above the pin's current rating.`,
  },
  deposition: {
    title: 'Deposition (nozzle → part)',
    text: `Where the hot-end nozzle meets the part. The plastic the extruder pushed out is laid down along the nozzle's path in the plate's frame. If the nozzle is pressed into the part a stiff contact force pushes it back up, and the equal and opposite force pushes on the turntable.`,
  },
  'probe-contact': {
    title: 'Probe contact (stylus → surface)',
    text: `Where the probe's stylus tip meets whatever is under it: the workpiece's height map, or the bare plate. Each millisecond the sim works out how far the tip has been pushed below that surface. Past the probe's pre-travel the probe's contact closes; lifting the tip opens it again (with a little hysteresis). Nothing else about the part changes: a probe measures, it does not cut.`,
  },
  cutting: {
    title: 'Cutting (end mill → part)',
    text: `Where the spinning cutter meets the part. Cutting power = the material's specific cutting energy × the volume removed per second; that becomes a braking torque on the spindle and a force on the tool (mostly against the feed) that pushes back through the arm and gearboxes, with the reaction on the turntable. A cutter that is not spinning cannot cut: it rams the part instead.`,
  },
};

export function docFor(kind) {
  return DOCS[kind] ?? { title: kind, text: 'No description for this kind yet.' };
}
