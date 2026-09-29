// Writes out/model.json: the built machine as plain data for a native port
// (see src/core/export.js), plus the state layout and a reference state hash.
import { writeFileSync, mkdirSync } from 'node:fs';
import { Machine, SCHEDULE } from '../src/machine/machine.js';
import { exportModel } from '../src/core/export.js';
import { stateTable, hashState } from '../src/core/state.js';

const m = new Machine();
const model = exportModel(m, { schedule: SCHEDULE });
const table = stateTable(m);
model.stateLayout = {
  scalars: table.scalars.map((e) => ({ label: e.label, kind: e.kind })),
  arrays: table.arrays.map((e) => ({ label: e.label, length: e.array.length, type: e.array.constructor.name })),
};
model.referenceHashAtBuild = hashState(m, table);
mkdirSync('out', { recursive: true });
writeFileSync('out/model.json', JSON.stringify(model, null, 1));
const n = (k) => model[k].length;
console.log(`out/model.json: ${n('bodies')} bodies, ${n('components')} components, ${n('connections')} connections, ${n('circuits')} circuits, ${n('logicNets')} logic nets, ${model.stateLayout.scalars.length} state scalars, ${model.problems.length} problems`);
if (model.problems.length) console.log(model.problems.join('\n'));
