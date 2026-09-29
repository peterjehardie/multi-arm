// Generates scenarios/tour.gcode: a short tour of every tool and feature.
// Best run with quick start and a preheated hot end, in demo mode.
//   1. enclosure exhaust fan on
//   2. hot end prints a small ring while the plate turns (polar mode)
//   3. spindle: the door is opened, the interlock holds the job until it is shut
//   4. spindle mills a pocket into a wax block
//   5. touch probe measures the block top, the pocket floor and the bare plate
import { writeFileSync } from 'node:fs';

const out = [];
const g = (s) => out.push(s);
const f = (x) => x.toFixed(3);
const say = (s) => g(`M118 ${s}`);
const layer = 0.3, width = 0.5, R = 9, layers = 3, segs = 48;
const ePerMm = (layer * width) / (Math.PI * 0.875 ** 2);
const block = { x: -30, y: 0, top: 8 };

g('; tour: every tool and feature in about four minutes of machine time');
g(`; @stock wax 20 20 ${block.top} ${block.x} ${block.y}`);
g('G28                ; home (skipped with quick start)');
g('G90');
say('Tour 1/5: the enclosure. Exhaust fan on: fumes and chips leave through one duct.');
g('M106 S180');
g('M140 S60           ; bed at 60 C');
g('M104 S210');

say('Tour 2/5: 3D printing. The hot end prints a ring, the plate turns so the arm stays in its plane.');
g('M6 T0              ; pick up the hot end');
g('M109 S210          ; wait for the hot end');
g('M83');
g(`G0 X${R} Y0 Z5`);
g('M620               ; polar mode on');
for (let L = 0; L < layers; L++) {
  const z = layer * (L + 1);
  g(`; layer ${L + 1}`);
  g(`G1 X${R} Y0 Z${f(z)} F600`);
  g('G1 E0.8 F1200');
  for (let k = 1; k <= segs; k++) {
    const a = (2 * Math.PI * k) / segs;
    const d = (2 * Math.PI * R) / segs;
    g(`G1 X${f(R * Math.cos(a))} Y${f(R * Math.sin(a))} E${f(d * ePerMm)} F${L === 0 ? 700 : 1000}`);
  }
  g('G1 E-0.8 F1800');
}
g('G0 Z10');
g('M621               ; polar mode off');
g('M104 S0');

say('Tour 3/5: safety interlock. The operator opens the door, and the spindle may not start until it is shut.');
g('M6 T1              ; change to the spindle');
g('M118 @door open 4  ; the operator opens the door for 4 s');
g('G4 P300');
g('M3 S8000           ; held by the interlock until the door is shut');

say('Tour 4/5: milling. The spindle cuts a 10 x 8 mm pocket, 1 mm deep, into the wax block.');
g(`G0 X${block.x - 5} Y${block.y - 4} Z${block.top + 3}`);
for (const depth of [0.5, 1.0]) {
  const z = block.top - depth;
  g(`G1 Z${f(z)} F120`);
  let dir = 1;
  for (let y = -4; y <= 4 + 1e-9; y += 1) {
    const x0 = block.x - 5, x1 = block.x + 5;
    g(`G1 Y${f(block.y + y)} F300`);
    g(`G1 X${f(dir > 0 ? x1 : x0)} F300`);
    dir = -dir;
  }
  g(`G0 Z${block.top + 3}`);
  g(`G0 X${block.x - 5} Y${block.y - 4}`);
}
g('M5');

say('Tour 5/5: measuring. The touch probe finds the block top, the pocket floor and the bare plate.');
g('M6 T2              ; change to the touch probe');
g(`G0 X${block.x + 7} Y${block.y + 7} Z${block.top + 6}`);
g(`G38.2 Z${block.top - 4} F120   ; block top (8 mm nominal)`);
g(`G0 Z${block.top + 6}`);
g(`G0 X${block.x} Y${block.y}`);
g(`G38.2 Z${block.top - 4} F120   ; pocket floor (7 mm nominal)`);
g(`G0 Z${block.top + 6}`);
g('G0 X20 Y0');
g('G38.2 Z-4 F120     ; bare plate (0 mm nominal)');
g('G0 Z14');
g('M107               ; fan off');
g('G0 X0 Y0 Z40');
say('Tour done. Each PRB line is a measured height, and the differences from nominal are the machine\'s own errors.');
g('M114');
writeFileSync(new URL('../scenarios/tour.gcode', import.meta.url), out.join('\n') + '\n');
console.log(`wrote ${out.length} lines`);
