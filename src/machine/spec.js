// The design: nominal dimensions and part choices, as they would appear on
// the drawings and the bill of materials. The simulated machine is built from
// this spec plus manufacturing tolerances. The firmware configuration is also
// derived from it, but only ever sees the nominal numbers, exactly like a
// real firmware config file written from the drawings.

const DEG = Math.PI / 180;

export const SPEC = {
  seed: 7,

  // --- arm geometry (metres) -------------------------------------------------
  arm: {
    baseHeight: 0.075,      // floor of the base to the yaw bearing (J1)
    shoulderHeight: 0.085,  // J1 bearing to shoulder axis (J2)
    L2: 0.22,               // shoulder -> elbow
    L3: 0.20,               // elbow -> wrist
    L4: 0.045,              // wrist axis -> tool-changer mating face
  },
  tolerances: {
    lengthSD: 0.15e-3,      // link length error, 1 sigma [m]
    switchSD: 0.08 * DEG,   // limit-switch trip position error, 1 sigma [rad]
  },

  // --- build plate on the turntable ------------------------------------------
  table: {
    center: [0.27, 0, 0.06],  // plate top surface centre, world [m]
    radius: 0.09,
    workSize: 0.12,           // height-map square [m]
    cell: 0.25e-3,
  },

  // --- tool rack (mating-face centre when parked, world) ----------------------
  rack: {
    hotend: [0.06, -0.26, 0.20],
    spindle: [0.06, 0.26, 0.20],
    probe: [0.06, 0.18, 0.20],
  },

  // --- enclosure (world, metres): acrylic box with a door on the +x side -------
  // The electronics bay stays outside (cooler, reachable); cables enter
  // through glands in the back wall.
  enclosure: {
    box: { min: [-0.09, -0.34, 0], max: [0.42, 0.34, 0.56] },
    glands: [[-0.09, 0.0, 0.015], [-0.09, -0.12, 0.015]],
  },
  tools: {
    hotend: { tipOffset: 0.058, mass: 0.1 },
    // 1/8" (3.175 mm) two-flute ball-nosed end mill: can rough and finish curved surfaces.
    spindle: { tipOffset: 0.092, cutterRadius: 1.5875e-3, flutes: 2, cutter: 'ball' },
    // Touch probe: 2 mm ruby ball on a 40 mm stylus, closes on 0.04 mm of travel.
    probe: { tipOffset: 0.075, pretravel: 0.04e-3 },
  },

  // --- drives -----------------------------------------------------------------
  // ratio: gearbox reduction; stiffness at output [N m/rad]; backlash [rad].
  drives: {
    j1: { motor: '17HS4401', ratio: 30, stiffness: 2500, backlash: 4 / 60 * DEG, efficiency: 0.8, coulomb: 0.15, damping: 4, Irun: 1.4 },
    j2: { motor: '17HS4401', ratio: 50, stiffness: 3500, backlash: 4 / 60 * DEG, efficiency: 0.75, coulomb: 0.25, damping: 5, Irun: 1.6 },
    j3: { motor: '17HS4401', ratio: 30, stiffness: 2000, backlash: 5 / 60 * DEG, efficiency: 0.8, coulomb: 0.12, damping: 3, Irun: 1.4 },
    j4: { motor: '17HS2408', ratio: 20, stiffness: 600, backlash: 6 / 60 * DEG, efficiency: 0.85, coulomb: 0.04, damping: 0.6, Irun: 0.6 },
    table: { motor: '17HS4401', ratio: 36, stiffness: 1500, backlash: 5 / 60 * DEG, efficiency: 0.8, coulomb: 0.1, damping: 2, Irun: 1.2 },
    e: { motor: '36STH20', ratio: 7.5, Irun: 0.9 },
  },
  microsteps: 16,

  // Homing switches (nominal trip angles) and resting pose at power-on.
  homing: {
    order: ['j4', 'j3', 'j2', 'j1', 'table'],
    j1: { trip: 12 * DEG, dir: +1 },
    j2: { trip: 100 * DEG, dir: +1 },
    j3: { trip: -75 * DEG, dir: +1 },
    j4: { trip: -70 * DEG, dir: +1 },
    table: { trip: 8 * DEG, dir: +1 },
  },
  restPose: [0, 90 * DEG, -90 * DEG, -90 * DEG],
  limits: {
    j1: [-175 * DEG, 175 * DEG], j2: [-25 * DEG, 160 * DEG],
    j3: [-160 * DEG, 20 * DEG], j4: [-150 * DEG, 60 * DEG],
  },
  maxJointSpeed: { j1: 60 * DEG, j2: 45 * DEG, j3: 60 * DEG, j4: 120 * DEG, table: 90 * DEG },
  // Firmware motion limits per joint: acceleration [rad/s^2] and the largest
  // instantaneous speed change at a corner [rad/s] a stepper follows reliably.
  maxJointAccel: { j1: 500 * DEG, j2: 400 * DEG, j3: 500 * DEG, j4: 1000 * DEG, table: 1500 * DEG },
  maxJointJump: { j1: 3 * DEG, j2: 2 * DEG, j3: 3 * DEG, j4: 5 * DEG, table: 4 * DEG },
};

// Motor datasheet values (typical catalogue numbers).
export const MOTORS = {
  // 42 x 42 x 40 mm, 1.7 A, 1.5 ohm, 2.8 mH, 40 N cm holding.
  '17HS4401': { R: 1.5, L: 2.8e-3, Kt: 0.40 / (Math.SQRT2 * 1.7), J: 54e-7, detent: 0.022, mass: 0.28, size: [0.042, 0.042, 0.040] },
  // 42 x 42 x 23 mm pancake, 1.0 A, 2.0 ohm, 1.8 mH, 16 N cm.
  '17HS2408': { R: 2.0, L: 1.8e-3, Kt: 0.16 / (Math.SQRT2 * 1.0), J: 20e-7, detent: 0.012, mass: 0.15, size: [0.042, 0.042, 0.023] },
  // 36 mm round pancake extruder motor, 1.0 A, 2.1 ohm, 1.5 mH, 10 N cm.
  '36STH20': { R: 2.1, L: 1.5e-3, Kt: 0.10 / (Math.SQRT2 * 1.0), J: 9e-7, detent: 0.006, mass: 0.13, size: [0.036, 0.036, 0.020] },
};

// 775-size brushed DC motor at 24 V: ~10,000 rpm no load, ~30 A stall.
export const SPINDLE_MOTOR = { R: 0.8, L: 0.25e-3, Ke: 0.0215, J: 2.2e-5, tauF: 0.010, b: 4e-6 };
