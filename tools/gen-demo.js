// Generates scenarios/demo.gcode: home, heat, print a ring with the plate
// turning (polar mode), change tools, mill a slot across the ring.
import { writeFileSync } from 'node:fs';

const out = [];
const g = (s) => out.push(s);
const f = (x) => x.toFixed(3);
const layer = 0.3, width = 0.5, R = 12, layers = 4, segs = 72;
const ePerMm = (layer * width) / (Math.PI * 0.875 ** 2);

g('; demo: print a ring on the turning plate, then mill a slot through it');
g('G28                ; home all joints and the plate');
g('M104 S210          ; hot end target (heats once the hot end tool is on)');
g('M6 T0              ; pick up the hot end tool');
g('M109 S210          ; wait for the hot end');
g('G90');
g('M83                ; relative extrusion');
g(`G0 X${R} Y0 Z5`);
g('M620               ; polar mode: plate turns, arm stays in its plane');
let e = 0;
for (let L = 0; L < layers; L++) {
  const z = layer * (L + 1);
  g(`; layer ${L + 1}`);
  g(`G1 X${R} Y0 Z${f(z)} F600`);
  g('G1 E0.8 F1200      ; prime / unretract');
  for (let k = 1; k <= segs; k++) {
    const a = (2 * Math.PI * k) / segs;
    const x = R * Math.cos(a), y = R * Math.sin(a);
    const d = (2 * Math.PI * R) / segs;
    g(`G1 X${f(x)} Y${f(y)} E${f(d * ePerMm)} F${L === 0 ? 600 : 900}`);
    e += d * ePerMm;
  }
  g('G1 E-0.8 F1800     ; retract');
}
g('G0 Z10');
g('M621               ; polar mode off');
g('M104 S0');
g('M6 T1              ; change to the spindle');
g('M3 S8000           ; spindle on');
g(`G0 X-${R + 6} Y0 Z4`);
g('G1 Z0.6 F120       ; plunge beside the ring');
g(`G1 X${R + 6} F150  ; slot through the ring, 0.6 mm deep`);
g('G0 Z10');
g('M5');
g('M6 T0              ; put the spindle back, hot end on');
g('G0 X0 Y0 Z40');
g('M114');
writeFileSync(new URL('../scenarios/demo.gcode', import.meta.url), out.join('\n') + '\n');
console.log(`wrote ${out.length} lines, ${e.toFixed(1)} mm filament`);
