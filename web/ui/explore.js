// Explore tab: the machine as a tree grouped by subsystem (Power, Controller,
// Drives, Arm, Tools, Build plate, Enclosure, Heat), or flat by kind. Every
// component and connection appears at least once; anything the rules below
// do not place lands in "Other", so nothing is ever hidden.

import { icon } from './icons.js';
import { docFor } from '../docs.js';

const JOINTS = [
  ['j1', 'Joint 1: base'], ['j2', 'Joint 2: shoulder'], ['j3', 'Joint 3: elbow'], ['j4', 'Joint 4: wrist'],
  ['table', 'Plate rotation'], ['e', 'Extruder feed'],
];
const TOOLS = [['hotend', 'Hot end'], ['spindle', 'Spindle'], ['probe', 'Touch probe']];
const POWER_IDS = new Set(['psu', 'tb24', 'buck', 'tb5', 'en_splice']);
const HEAT_KINDS = new Set(['heater', 'thermistor', 'thermal-mass', 'ambient', 'thermal-contact', 'convection', 'enclosure']);

function escapeHtml(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]); }

// Build the subsystem tree: [{ key, title, icon, note, items?, groups? }].
export function subsystemTree(m) {
  const comps = [...m.A.components.values()];
  const conns = [...m.A.connections.values()];
  const placed = new Set();
  const C = (c) => ({ type: 'component', id: c.id, obj: c });
  const X = (c) => ({ type: 'connection', id: c.id, obj: c });
  const take = (list) => { for (const r of list) placed.add(r.obj); return list; };
  const ends = (c) => [c.a?.owner, c.b?.owner].filter(Boolean);
  const touches = (c, set) => ends(c).some((o) => set.has(o));
  const bodyOf = (c) => c.mount?.body?.name ?? '';
  const byId = (id) => m.A.components.get(id);

  // Power: supply, regulators, distribution and the wires joining them.
  const powerComps = comps.filter((c) => POWER_IDS.has(c.id) || c.kind === 'psu' || c.kind === 'buck');
  const powerSet = new Set(powerComps);
  const powerWires = conns.filter((c) => c.kind === 'wire' && ends(c).length === 2 &&
    ends(c).every((o) => powerSet.has(o) || o.kind === 'controller') && touches(c, powerSet));

  // Controller: board, host, USB, switching modules.
  const ctrlComps = comps.filter((c) => ['controller', 'host', 'mosfet'].includes(c.kind));
  const ctrlConns = conns.filter((c) => c.kind === 'usb-cable');
  const fetSet = new Set(comps.filter((c) => c.kind === 'mosfet'));
  const fetWires = conns.filter((c) => c.kind === 'wire' && touches(c, fetSet));

  // Drives, one per joint.
  const driveGroups = [];
  for (const [n, title] of JOINTS) {
    const set = new Set([byId(`drv_${n}`), byId(`mot_${n}`), byId(`sw_${n}`)].filter(Boolean));
    if (!set.size) continue;
    const mech = conns.filter((c) => c.id === `gb_${n}` || c.id === `cam_${n}` || (c.kind !== 'wire' && c.kind !== 'pogo-contact' && touches(c, set) && ['gearbox', 'cam', 'gear-mesh'].includes(c.kind)));
    const wires = conns.filter((c) => c.kind === 'wire' && touches(c, set) && !touches(c, powerSet));
    const power = conns.filter((c) => c.kind === 'wire' && touches(c, set) && touches(c, powerSet));
    driveGroups.push({
      key: `drive-${n}`, title,
      sections: [
        ['Parts', take([...set].map(C))],
        ['Mechanical links', take(mech.map(X))],
        ['Signal and motor wires', take(wires.map(X))],
        ['Power wires', take(power.map(X))],
      ],
    });
  }

  // Tools: the changer and each tool's parts.
  const changerComps = comps.filter((c) => c.kind === 'changer-master' || c.kind === 'servo' || c.id === 'rack');
  const changerSet = new Set(changerComps);
  const pogo = conns.filter((c) => c.kind === 'pogo-contact');
  const changerLinks = conns.filter((c) => c.kind === 'linkage' || (c.kind === 'wire' && touches(c, changerSet) && ends(c).every((o) => changerSet.has(o) || o.kind === 'controller' || o.kind === 'terminal')));
  const toolGroups = [{
    key: 'tool-changer', title: 'Tool changer',
    sections: [['Parts', take(changerComps.map(C))], ['Links and wires', take(changerLinks.map(X))], ['Pogo pins', take(pogo.map(X))]],
  }];
  for (const [t, title] of TOOLS) {
    const parts = comps.filter((c) => bodyOf(c) === `tool_${t}`);
    const set = new Set(parts);
    const links = conns.filter((c) => c.kind !== 'pogo-contact' && touches(c, set));
    if (!parts.length) continue;
    toolGroups.push({
      key: `tool-${t}`, title,
      sections: [['Parts', take(parts.map(C))], ['Connections', take(links.filter((c) => c.kind !== 'wire').map(X))], ['Wires', take(links.filter((c) => c.kind === 'wire').map(X))]],
    });
  }

  // Build plate: turntable, slip ring, and everything riding on the plate.
  const plateComps = comps.filter((c) => c.kind === 'turntable' || c.kind === 'slip-ring' || bodyOf(c) === 'table');
  const plateSet = new Set(plateComps);
  const plateConns = conns.filter((c) => touches(c, plateSet) && !['gearbox', 'cam'].includes(c.kind));

  // Enclosure: the box, its fan switch and wires, its wall to the room.
  const encComps = comps.filter((c) => c.kind === 'enclosure' || c.id === 'fet_fan');
  const encCore = new Set(comps.filter((c) => c.kind === 'enclosure'));
  const encConns = conns.filter((c) => touches(c, encCore));

  // Arm.
  const armComps = comps.filter((c) => c.kind === 'arm');

  // Heat: a cross-cutting view (items also appear in their home subsystem).
  const heatComps = comps.filter((c) => HEAT_KINDS.has(c.kind));
  const heatConns = conns.filter((c) => HEAT_KINDS.has(c.kind));

  const tree = [
    { key: 'power', title: 'Power', icon: 'bolt', note: 'The 24 V supply, the 5 V regulator and how power is shared out.',
      sections: [['Parts', take(powerComps.map(C))], ['Wires', take(powerWires.map(X))]] },
    { key: 'controller', title: 'Controller', icon: 'chip', note: 'The board running the firmware, the PC streaming G-code, and the load switches.',
      sections: [['Parts', take(ctrlComps.map(C))], ['Links', take(ctrlConns.map(X))], ['Load switch wires', take(fetWires.map(X))]] },
    { key: 'drives', title: 'Drives', icon: 'gear', note: 'One stepper drive per joint: driver board, motor, gearbox, limit switch and wires.', groups: driveGroups },
    { key: 'arm', title: 'Arm', icon: 'arm', note: 'The four links and their joints, moved by the drives above.', sections: [['Parts', take(armComps.map(C))]] },
    { key: 'tools', title: 'Tools', icon: 'wrench', note: 'The wrist changer and the three tools it can pick up.', groups: toolGroups },
    { key: 'plate', title: 'Build plate', icon: 'plate', note: 'The turning plate, its heater and the part being made on it.',
      sections: [['Parts', take(plateComps.map(C))], ['Connections', take(plateConns.map(X))]] },
    { key: 'enclosure', title: 'Enclosure', icon: 'enclosure', note: 'Panels, door switch and exhaust fan around the machine.',
      sections: [['Parts', take(encComps.map(C))], ['Connections', take(encConns.map(X))]] },
    { key: 'heat', title: 'Heat', icon: 'thermo', note: 'Every heater, sensor, thermal mass and heat path, in one place.',
      sections: [['Parts', heatComps.map(C)], ['Heat paths', heatConns.map(X)]] },
  ];
  const rest = [...comps.filter((c) => !placed.has(c) && !HEAT_KINDS.has(c.kind)).map(C), ...conns.filter((c) => !placed.has(c) && !HEAT_KINDS.has(c.kind)).map(X)];
  // Wires still unplaced: attach to "Other wiring".
  if (rest.length) tree.push({ key: 'other', title: 'Other wiring and parts', icon: 'box', note: 'Everything not listed under a subsystem above.', sections: [['Items', rest]] });
  return tree;
}

function kindTree(m) {
  const groups = new Map();
  const add = (kind, obj, type) => {
    const key = `${type === 'component' ? 'Parts' : 'Connections'}|${docFor(kind).title}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push({ type, id: obj.id, obj });
  };
  for (const c of m.A.components.values()) add(c.kind, c, 'component');
  for (const c of m.A.connections.values()) add(c.kind, c, 'connection');
  return [...groups].sort().map(([key, items]) => ({ key, title: key.split('|')[1], icon: key.startsWith('Parts') ? 'box' : 'link', sections: [['', items]] }));
}

export class Explorer {
  constructor(root, filterInput, { onSelect }) {
    this.root = root;
    this.filterInput = filterInput;
    this.onSelect = onSelect;
    this.mode = 'system';
    this.m = null;
    this.selKey = null;
    filterInput.addEventListener('input', () => this.filter());
    root.addEventListener('click', (e) => {
      const b = e.target.closest('.item');
      if (!b) return;
      this.onSelect({ type: b.dataset.type, id: b.dataset.id });
    });
  }

  attach(m) { this.m = m; this.build(); }
  setMode(mode) { this.mode = mode; this.build(); }

  build() {
    const m = this.m;
    if (!m) return;
    const openKeys = new Set([...this.root.querySelectorAll('details[open]')].map((d) => d.dataset.key));
    const first = !this.root.childElementCount;
    const tree = this.mode === 'kind' ? kindTree(m) : subsystemTree(m);
    const itemHtml = (r) => {
      const o = r.obj;
      const sw = o.color ? `<span class="swatch" style="background:${escapeHtml(o.color)}"></span>` : '';
      return `<button type="button" class="item" data-type="${r.type}" data-id="${escapeHtml(o.id)}" data-search="${escapeHtml(`${o.label} ${o.id} ${o.kind} ${docFor(o.kind).title}`.toLowerCase())}">${sw}<span class="nm">${escapeHtml(o.label)}</span><code>${escapeHtml(o.id)}</code></button>`;
    };
    const sectionsHtml = (sections) => sections.filter(([, items]) => items.length).map(([h, items]) =>
      `${h ? `<div class="sub-h">${escapeHtml(h)} <span>(${items.length})</span></div>` : ''}${items.map(itemHtml).join('')}`).join('');
    const count = (g) => (g.sections ?? []).reduce((n, [, it]) => n + it.length, 0) + (g.groups ?? []).reduce((n, s) => n + count(s), 0);
    const chev = icon('chevron', 'chev');
    const html = tree.map((g) => {
      const n = count(g);
      if (!n) return '';
      const inner = g.groups
        ? g.groups.map((s) => `<details class="g2" data-key="${s.key}"${openKeys.has(s.key) ? ' open' : ''}><summary>${chev}<span>${escapeHtml(s.title)}</span><span class="count">${count(s)}</span></summary><div class="items">${sectionsHtml(s.sections)}</div></details>`).join('')
        : `<div class="items">${sectionsHtml(g.sections)}</div>`;
      return `<details class="g1" data-key="${g.key}"${openKeys.has(g.key) ? ' open' : ''}><summary>${chev}<span class="gi">${icon(g.icon)}</span><span>${escapeHtml(g.title)}</span><span class="count">${n}</span></summary>${g.note ? `<p class="g-note">${escapeHtml(g.note)}</p>` : ''}${inner}</details>`;
    }).join('');
    this.root.innerHTML = html + '<div class="none" hidden>No parts match.</div>';
    if (first && this.mode === 'system') this.root.querySelector('details[data-key="drives"]')?.setAttribute('open', '');
    this.markSelection();
    this.filter();
  }

  filter() {
    const q = this.filterInput.value.trim().toLowerCase();
    let anyAll = false;
    for (const det of [...this.root.querySelectorAll('details')].reverse()) {
      let any = false;
      for (const b of det.querySelectorAll('.item')) {
        const hit = !q || b.dataset.search.includes(q);
        b.hidden = !hit; any ||= hit;
      }
      for (const h of det.querySelectorAll(':scope > .items > .sub-h')) {
        let el = h.nextElementSibling, vis = false;
        while (el && el.classList.contains('item')) { vis ||= !el.hidden; el = el.nextElementSibling; }
        h.hidden = !vis;
      }
      det.hidden = !any;
      if (q) det.open = any;
      anyAll ||= any;
    }
    this.root.querySelector('.none').hidden = anyAll;
  }

  setSelection(ref) {
    this.selKey = ref ? `${ref.type}:${ref.id}` : null;
    this.markSelection();
  }
  markSelection() {
    for (const b of this.root.querySelectorAll('.item[aria-current]')) b.removeAttribute('aria-current');
    if (!this.selKey) return;
    const [type, id] = [this.selKey.slice(0, this.selKey.indexOf(':')), this.selKey.slice(this.selKey.indexOf(':') + 1)];
    for (const b of this.root.querySelectorAll('.item')) if (b.dataset.type === type && b.dataset.id === id) b.setAttribute('aria-current', 'true');
  }

  // Open the groups containing the selection and scroll it into view.
  reveal() {
    const b = this.root.querySelector('.item[aria-current="true"]');
    if (!b) return;
    for (let d = b.closest('details'); d; d = d.parentElement?.closest('details')) d.open = true;
    b.scrollIntoView({ block: 'nearest' });
  }
}
