// Generates the demo part and its two job files:
//   scenarios/knob.stl             the part as designed (triangle mesh)
//   scenarios/knob-hybrid.gcode    print it slightly oversize, then finish the
//                                  top with the ball-nosed cutter (hybrid)
//   scenarios/knob-machined.gcode  carve it from a wax block (subtractive only)
import { writeFileSync } from 'node:fs';
import { knobMesh } from '../src/cam/models.js';
import { writeBinarySTL, rasterizeTop, meshBounds } from '../src/cam/mesh.js';
import { sliceMesh, printGcode } from '../src/cam/slicer.js';
import { finishPass, roughPasses, millGcode } from '../src/cam/cam.js';
import { SPEC } from '../src/machine/spec.js';

const dir = new URL('../scenarios/', import.meta.url);
const R = SPEC.tools.spindle.cutterRadius * 1000; // cutter radius, mm
const grid = (tris, pad = 3, cell = 0.2) => {
  const { lo, hi } = meshBounds(tris);
  return rasterizeTop(tris, {
    x0: lo[0] - pad, y0: lo[1] - pad, cell,
    nx: Math.ceil((hi[0] - lo[0] + 2 * pad) / cell), ny: Math.ceil((hi[1] - lo[1] + 2 * pad) / cell),
  });
};
const round = (x) => Math.round(x);

// ------------------------------------------------------------ the part
const knob = knobMesh({ R: 8, wallH: 3.5, H: 6.5, flutes: 9 });
writeFileSync(new URL('knob.stl', dir), writeBinarySTL(knob, 'multi-arm demo knob'));

// ------------------------------------------------------------ hybrid job
{
  const place = [28, 0]; // well off the plate centre: in polar mode the plate never has to flip
  const sliced = sliceMesh(knob, { place, sliceAt: 'bottom' });
  const print = printGcode(sliced, {
    preamble: [
      '; knob, hybrid: print slightly oversize, then finish the dome with the ball-nosed cutter',
      'G28', 'M104 S210', 'M6 T0', 'M109 S210', 'G90', 'M83',
      `G0 X${place[0] + 10} Y${place[1]} Z5`, 'M620              ; polar mode: the plate turns, the arm stays in its plane',
    ],
    postamble: ['G0 Z15', 'M621', 'M104 S0'],
  });
  const hm = grid(knob);
  const partR = 8.6;
  const finish = finishPass(hm, { R, shape: 'ball', stepover: 0.45, step: 0.4, floor: 0.6, clip: (x, y) => Math.hypot(x, y) < partR + R });
  const mill = millGcode([finish], {
    place, safeZ: 12, feed: 540, plunge: 120,
    lines: ['M6 T1', 'M3 S9000',
      'G0 X0 Y0 Z15      ; over the plate centre, where turning the plate moves nothing',
      'M620              ; polar mode again: the plate brings each point to the arm'],
  });
  const lines = [...print.lines, ...mill.lines, 'G0 X0 Y0 Z15', 'M621', 'M5', 'M6 T0', 'G0 X0 Y0 Z40', 'M114'];
  writeFileSync(new URL('knob-hybrid.gcode', dir), lines.join('\n') + '\n');
  console.log(`knob-hybrid: ${print.stats.layers} layers, ${round(print.stats.filament_mm)} mm filament, print ~${round(print.stats.time_s)} s + finish ~${round(mill.stats.time_s)} s, ${lines.length} lines`);
}

// ------------------------------------------------------------ machined job
{
  const place = [-30, 0];
  const stock = { size: [24, 24, 8], top: 8 };
  // A slightly smaller knob standing on a 1 mm base left in the block.
  const part = knobMesh({ R: 7.5, wallH: 3, H: 6.6, flutes: 9 });
  for (let i = 2; i < part.length; i += 3) part[i] += 1.0;
  const hm = grid(part, 4);
  const rect = { x0: -stock.size[0] / 2 + 0.5, x1: stock.size[0] / 2 - 0.5, y0: -stock.size[1] / 2 + 0.5, y1: stock.size[1] / 2 - 0.5 };
  const rough = roughPasses(hm, { R, shape: 'ball', top: stock.top, bottom: 1.0, stepdown: 1.8, stepover: 1.6, allowance: 0.3, rect, step: 0.8 });
  const finish = finishPass(hm, { R, shape: 'ball', stepover: 0.45, step: 0.4, floor: 1.0, clip: (x, y) => Math.hypot(x, y) < 8.1 + R });
  const mill = millGcode([...rough, finish], {
    place, safeZ: 12, feed: 600, plunge: 90,
    lines: [
      '; knob, machined from a wax block with the ball-nosed cutter: roughing slabs, then finishing',
      `; @stock wax ${stock.size.join(' ')} ${place.join(' ')}`,
      'G28', 'M6 T1', 'M3 S9000', 'G90',
    ],
  });
  const lines = [...mill.lines, 'M5', 'G0 X0 Y0 Z40', 'M114'];
  writeFileSync(new URL('knob-machined.gcode', dir), lines.join('\n') + '\n');
  console.log(`knob-machined: ${rough.length} roughing slabs, cutting ~${round(mill.stats.time_s)} s, ${lines.length} lines`);
}
