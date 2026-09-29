// Records the reference trace used by test/architecture.test.js and by any
// port of the simulator to another language: the ring demo with quick start,
// 3 s of machine time, selected signals every 50 ms and the final state hash.
//   node tools/golden.js      (rerun only when a physics change is intended)
import { readFileSync, writeFileSync } from 'node:fs';
import { goldenRun } from '../test/golden/run.js';

const g = goldenRun(readFileSync(new URL('../scenarios/demo.gcode', import.meta.url), 'utf8'));
writeFileSync(new URL('../test/golden/demo-quickstart-3s.json', import.meta.url), JSON.stringify(g));
console.log(`wrote ${g.samples.length} samples of ${g.signals.length} signals, final hash ${g.hash}`);
