// Inspector panel: what the selected component or connection is, its live
// state from inspect(), its ports and what each port is connected to.

import { docFor } from './docs.js';

// Units from key names: suffix first (iA_A, winding_C), then prefix (V_out).
const SUFFIX = [
  ['_mm3s', 'mm³/s'], ['_mm3', 'mm³'], ['_JperK', 'J/K'], ['_WperK', 'W/K'], ['_kgm2', 'kg·m²'],
  ['_arcmin', '′'], ['_deg', '°'], ['_rpm', 'rpm'], ['_Nm', 'N·m'], ['_ohm', 'Ω'], ['_mm', 'mm'],
  ['_A', 'A'], ['_V', 'V'], ['_W', 'W'], ['_C', '°C'], ['_N', 'N'], ['_m', 'm'], ['_s', 's'], ['_g', 'g'],
];
const PREFIX = [['V_', 'V'], ['I_', 'A'], ['P_', 'W'], ['T_', '°C'], ['R_', 'Ω']];
const SI = new Set(['A', 'V', 'W', 'Ω', 'N·m', 'N']);

export function splitKey(key) {
  for (const [s, u] of SUFFIX) if (key.endsWith(s) && key.length > s.length) return { name: key.slice(0, -s.length), unit: u };
  for (const [p, u] of PREFIX) if (key.startsWith(p) && key.length > p.length) return { name: key, unit: u };
  return { name: key, unit: '' };
}

export function fmtNum(v, unit = '') {
  if (typeof v !== 'number') return String(v);
  if (!Number.isFinite(v)) return Number.isNaN(v) ? '—' : (v > 0 ? '∞' : '−∞');
  if (unit === '' && Number.isInteger(v)) return String(v);
  if (SI.has(unit)) {
    const a = Math.abs(v);
    let s = 1, p = '';
    if (a < 1e-7) return `0 ${unit}`;
    if (a < 1e-3) { s = 1e6; p = 'µ'; } else if (a < 1) { s = 1e3; p = 'm'; }
    else if (a >= 1e6) { s = 1e-6; p = 'M'; } else if (a >= 1e3) { s = 1e-3; p = 'k'; }
    return `${sig(v * s)} ${p}${unit}`;
  }
  if (unit === '°C') return `${v.toFixed(1)} °C`;
  if (unit === '°' || unit === '′') return `${v.toFixed(2)}${unit}`;
  if (unit === 'rpm') return `${v.toFixed(0)} rpm`;
  return unit ? `${sig(v)} ${unit}` : sig(v);
}
function sig(v) {
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 1000) return v.toFixed(0);
  if (a >= 100) return v.toFixed(1);
  if (a >= 10) return v.toFixed(2);
  if (a >= 0.001) return v.toFixed(3);
  return v.toExponential(2);
}

function el(tag, attrs = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else if (k === 'style') e.style.cssText = v;
    else e.setAttribute(k, v);
  }
  for (const k of kids) if (k != null) e.append(k instanceof Node ? k : document.createTextNode(String(k)));
  return e;
}

export class Inspector {
  constructor(root, { onSelect }) {
    this.root = root;
    this.onSelect = onSelect;
    this.m = null;
    this.ref = null;
    this.liveBody = null;
    this.showEmpty();
  }
  attach(m) { this.m = m; }

  showEmpty() {
    this.ref = null;
    this.liveBody = null;
    this.root.replaceChildren(el('div', { class: 'empty' },
      el('p', {}, 'Click a part or a wire in the 3D view (or pick one in the Parts tab) to see what it is and what the simulation is doing inside it.'),
      el('p', { class: 'muted' }, 'Drag to orbit, right-drag to pan, scroll to zoom.')));
  }

  link(label, ref, cls = '') {
    return el('a', { href: '#', class: `link ${cls}`, onclick: (ev) => { ev.preventDefault(); this.onSelect(ref); } }, label);
  }

  show(ref) {
    const m = this.m;
    if (!ref || !m) return this.showEmpty();
    const obj = ref.type === 'component' ? m.A.components.get(ref.id) : m.A.connections.get(ref.id);
    if (!obj) return this.showEmpty();
    this.ref = ref;
    this.obj = obj;
    const doc = docFor(obj.kind);
    const head = el('div', { class: 'insp-head' },
      el('div', { class: 'insp-title' },
        obj.color ? el('span', { class: 'swatch', style: `background:${obj.color}` }) : null,
        obj.label),
      el('div', { class: 'insp-meta' },
        el('span', { class: 'pill' }, obj.kind), ' ', el('code', {}, obj.id)));
    const docEl = el('div', { class: 'doc' }, el('div', { class: 'doc-title' }, doc.title), el('p', {}, doc.text));

    const facts = el('table', { class: 'kv' });
    const addFact = (k, v) => facts.append(el('tr', {}, el('th', {}, k), el('td', {}, v)));
    if (ref.type === 'component') {
      if (obj.mass) addFact('mass', fmtNum(obj.mass * 1000, 'g'));
      addFact('size', obj.size.map((s) => (s * 1000).toFixed(0)).join(' × ') + ' mm');
      if (obj.mount) addFact('mounted on', `${obj.mount.body.name} body`);
    } else {
      if (obj.kind === 'wire') {
        addFact('gauge', `${obj.awg} AWG`);
        addFact('cut length', fmtNum(obj.length * 1000, 'mm'));
        addFact('insulation', el('span', {}, el('span', { class: 'swatch', style: `background:${obj.color}` }), obj.color));
      }
      if (obj.N) addFact('ratio', `${obj.N} : 1`);
      if (obj.G !== undefined) addFact('conductance', fmtNum(obj.G, 'W/K'));
      addFact('route', `${obj.route.length} waypoint${obj.route.length === 1 ? '' : 's'}`);
    }

    this.liveBody = el('tbody');
    const live = el('table', { class: 'kv live' }, this.liveBody);

    const conn = el('div', { class: 'ports' });
    if (ref.type === 'component') {
      const ports = [...obj.ports.values()];
      const shown = ports.filter((p) => p.connections.length > 0);
      const hidden = ports.length - shown.length;
      for (const p of shown) {
        const row = el('div', { class: 'port' },
          el('div', { class: 'port-name' }, el('code', {}, p.name), el('span', { class: `dom dom-${p.domain}` }, p.domain), el('span', { class: 'muted' }, p.role)));
        for (const c of p.connections) {
          const other = c.other(p);
          row.append(el('div', { class: 'port-link' },
            '→ ', this.link(c.kind === 'wire' ? `${c.id} (${c.awg} AWG)` : `${c.label}`, { type: 'connection', id: c.id }, c.kind === 'wire' ? 'wire' : ''),
            c.color ? el('span', { class: 'swatch', style: `background:${c.color}` }) : null,
            ' to ', this.link(`${other.owner.label}`, { type: 'component', id: other.owner.id }), el('code', { class: 'muted' }, `.${other.name}`)));
        }
        conn.append(row);
      }
      if (hidden) conn.append(el('div', { class: 'muted small' }, `${hidden} unused port${hidden > 1 ? 's' : ''} not shown`));
      if (!ports.length) conn.append(el('div', { class: 'muted small' }, 'No ports.'));
    } else {
      for (const [end, p] of [['A', obj.a], ['B', obj.b]]) {
        conn.append(el('div', { class: 'port' },
          el('div', { class: 'port-name' }, el('span', { class: 'pill' }, `end ${end}`), ' ',
            this.link(p.owner.label, { type: 'component', id: p.owner.id }), el('code', { class: 'muted' }, `.${p.name}`),
            el('span', { class: `dom dom-${p.domain}` }, p.domain), el('span', { class: 'muted' }, p.role))));
      }
    }

    this.root.replaceChildren(head, docEl,
      el('h3', {}, 'Live values'), live,
      el('h3', {}, 'Facts'), facts,
      el('h3', {}, ref.type === 'component' ? 'Ports and what they connect to' : 'Ends'), conn);
    this.refresh();
  }

  // Called ~5 Hz.
  refresh() {
    if (!this.liveBody || !this.obj) return;
    let vals;
    try { vals = this.obj.inspect() ?? {}; } catch (e) { vals = { error: String(e) }; }
    if (this.ref.type === 'connection' && this.obj.kind === 'wire') {
      vals = { ...vals, V_drop: this.obj.i * this.obj.resistance() };
    }
    const rows = Object.entries(vals);
    if (!rows.length) {
      this.liveBody.replaceChildren(el('tr', {}, el('td', { class: 'muted', colspan: 2 }, 'This part has no internal state; it only passes things along.')));
      return;
    }
    // Re-use rows when the key set is unchanged (keeps text selectable).
    const keys = rows.map(([k]) => k).join('|');
    if (this.liveKeys !== keys || this.liveBody.children.length !== rows.length) {
      this.liveKeys = keys;
      this.liveBody.replaceChildren(...rows.map(([k]) => {
        const { name } = splitKey(k);
        return el('tr', {}, el('th', { title: k }, name.replace(/_/g, ' ')), el('td'));
      }));
    }
    rows.forEach(([k, v], i) => {
      const td = this.liveBody.children[i].children[1];
      let text;
      if (typeof v === 'boolean') { td.className = v ? 'yes' : 'no'; text = v ? 'yes' : 'no'; }
      else { td.className = ''; text = typeof v === 'number' ? fmtNum(v, splitKey(k).unit) : String(v); }
      if (td.textContent !== text) td.textContent = text;
    });
  }
}
