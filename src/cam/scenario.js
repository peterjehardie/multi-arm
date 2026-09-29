// Scenario directives embedded in G-code comments, so a job file can say
// what must be on the plate before it runs:
//   ; @stock wax 30 30 8 -30 0     material, size x y z (mm), centre x y (mm)

import { MATERIALS } from '../process/workpiece.js';

export function parseDirectives(text) {
  const out = { stock: [] };
  for (const m of text.matchAll(/^;\s*@stock\s+(\w+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)/gm))
    out.stock.push({ material: m[1], size: [+m[2], +m[3], +m[4]], centre: [+m[5], +m[6]] });
  return out;
}

export function applyDirectives(machine, text) {
  const d = parseDirectives(text);
  for (const s of d.stock) {
    const mat = MATERIALS[s.material];
    if (!mat) throw new Error(`Unknown stock material "${s.material}"`);
    machine.workpiece.addStock(mat, s.size[0] / 1000, s.size[1] / 1000, s.size[2] / 1000, s.centre[0] / 1000, s.centre[1] / 1000);
  }
  return d;
}
