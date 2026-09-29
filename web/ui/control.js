// Control tab: drive the machine by hand. Every button sends ordinary G-code
// through the host PC (so it appears in the console and queues behind a
// running job), except the enclosure door, which is a person opening or
// shutting the physical door.

import { icon } from './icons.js';

const TOOLS = [
  { t: 0, name: 'hotend', label: 'Hot end', icon: 'print', note: 'print' },
  { t: 1, name: 'spindle', label: 'Spindle', icon: 'mill', note: 'mill' },
  { t: 2, name: 'probe', label: 'Probe', icon: 'probe', note: 'measure' },
];

export class ControlPanel {
  // send(lines[]): send G-code lines; getMachine(): current Machine;
  // setDoor(open): open or shut the enclosure door; onProbeClear().
  constructor(root, { send, getMachine, setDoor, onProbeClear }) {
    this.root = root;
    this.send = send;
    this.getMachine = getMachine;
    this.setDoor = setDoor;
    this.onProbeClear = onProbeClear;
    this.step = 1;
    this.probes = [];
    this.render();
  }

  render() {
    const r = this.root;
    r.innerHTML = `
      <div class="banner warn" data-el="homeBanner" hidden>${icon('warn')}<div><b>Not homed.</b> Moves, tool changes and probing are ignored until the machine is homed. Press <b>Home</b> (about 20 s in full physics), or tick <i>Quick start</i> and Reset to start homed.</div></div>
      <div class="banner warn" data-el="queueBanner" hidden>${icon('info')}<div><b>A job is queued.</b> Commands from here wait until the job's remaining <span data-el="queueN">0</span> lines have been sent. Reset to drive the machine by hand straight away.</div></div>
      <div class="banner bad" data-el="doorBanner" hidden>${icon('door')}<div><b>Door open.</b> The spindle interlock is active: the spindle stops and a milling job waits until the door is shut.</div></div>

      <section class="card">
        <h3>${icon('home')} Machine</h3>
        <div class="row">
          <button type="button" class="btn" data-g="G28" title="G28: find each joint's limit switch">${icon('home')}<span>Home</span></button>
          <button type="button" class="btn" data-g="M114" title="M114: the firmware reports where it thinks the tool is (see Console)">${icon('pin')}<span>Report position</span></button>
          <span class="grow"></span>
          <span class="stat"><i>homed</i><b data-el="homed">—</b></span>
        </div>
      </section>

      <section class="card">
        <h3>${icon('wrench')} Tool</h3>
        <div class="tools3">
          ${TOOLS.map((t) => `<button type="button" class="btn toolbtn" data-tool="${t.name}" data-g="M6 T${t.t}" title="M6 T${t.t}: put the current tool back in the rack and pick up the ${t.label.toLowerCase()}">${icon(t.icon)}<span>${t.label}</span><small>M6 T${t.t}</small></button>`).join('')}
        </div>
        <p class="hint-sm">On the wrist: <b data-el="tool">—</b>. A tool change takes about 15 s of machine time.</p>
      </section>

      <section class="card">
        <h3>${icon('control')} Jog <span class="muted small" style="text-transform:none;letter-spacing:0;font-weight:400">plate coordinates</span></h3>
        <div class="jog">
          <div class="pad" role="group" aria-label="Move in X and Y">
            <span></span>
            <button type="button" class="btn" data-jog="Y+" aria-label="Y plus">${icon('up')}<span class="axis">Y+</span></button>
            <span></span>
            <button type="button" class="btn" data-jog="X-" aria-label="X minus">${icon('left')}<span class="axis">X−</span></button>
            <button type="button" class="btn" data-g="M114" aria-label="Report position" title="Report position (M114)">${icon('pin')}</button>
            <button type="button" class="btn" data-jog="X+" aria-label="X plus">${icon('right')}<span class="axis">X+</span></button>
            <span></span>
            <button type="button" class="btn" data-jog="Y-" aria-label="Y minus">${icon('down')}<span class="axis">Y−</span></button>
            <span></span>
          </div>
          <div class="zcol" role="group" aria-label="Move in Z">
            <button type="button" class="btn" data-jog="Z+" aria-label="Z up">${icon('up')}<span class="axis">Z+</span></button>
            <span class="zl">Z</span>
            <button type="button" class="btn" data-jog="Z-" aria-label="Z down">${icon('down')}<span class="axis">Z−</span></button>
          </div>
        </div>
        <div class="row" style="margin-top:10px">
          <span class="lbl-sm">Step</span>
          <div class="seg small" role="radiogroup" aria-label="Jog step" data-el="stepSeg">
            <button type="button" role="radio" data-step="1" aria-checked="true">1 mm</button>
            <button type="button" role="radio" data-step="10" aria-checked="false">10 mm</button>
          </div>
        </div>
        <p class="hint-sm">Each press sends <code>G91</code>, one <code>G0</code> move, then <code>G90</code>. Moves out of the arm's reach are skipped by the firmware.</p>
      </section>

      <section class="card">
        <h3>${icon('flame')} Heat and spindle</h3>
        <div class="row">
          <span class="lbl-sm">Hot end</span>
          <button type="button" class="btn compact" data-g="M104 S210" title="M104 S210">Heat 210 °C</button>
          <button type="button" class="btn compact" data-g="M104 S0" title="M104 S0">Off</button>
          <span class="grow"></span><span class="readout" data-el="hot">—</span>
        </div>
        <div class="row">
          <span class="lbl-sm">Bed</span>
          <button type="button" class="btn compact" data-g="M140 S60" title="M140 S60">Heat 60 °C</button>
          <button type="button" class="btn compact" data-g="M140 S0" title="M140 S0">Off</button>
          <span class="grow"></span><span class="readout" data-el="bed">—</span>
        </div>
        <div class="row">
          <span class="lbl-sm">Spindle</span>
          <button type="button" class="btn compact" data-g="M3 S8000" title="M3 S8000">Start 8000 rpm</button>
          <button type="button" class="btn compact" data-g="M5" title="M5">Stop</button>
          <span class="grow"></span><span class="readout" data-el="rpm">—</span>
        </div>
        <p class="hint-sm">The hot-end heater is on the tool, so it only heats while the hot end is on the wrist. The spindle only turns with the spindle tool on and the door shut.</p>
      </section>

      <section class="card">
        <h3>${icon('enclosure')} Enclosure</h3>
        <div class="row">
          <span class="toggle"><button type="button" class="switch" role="switch" aria-checked="false" data-el="door" aria-label="Door open"></button><span data-el="doorText">Door closed</span></span>
          <span class="grow"></span>
          <span class="stat"><i>chamber</i><b data-el="chamber">—</b></span>
        </div>
        <div class="row">
          <span class="lbl-sm">Exhaust fan</span>
          <button type="button" class="btn compact" data-g="M106 S255" title="M106 S255">On</button>
          <button type="button" class="btn compact" data-g="M106 S128" title="M106 S128">Half</button>
          <button type="button" class="btn compact" data-g="M107" title="M107">Off</button>
          <span class="grow"></span><span class="readout" data-el="fan">—</span>
        </div>
        <p class="hint-sm">Opening the door is something you do to the machine, not a G-code: the firmware only sees its door switch change.</p>
      </section>

      <section class="card">
        <h3>${icon('polar')} Polar mode</h3>
        <div class="row">
          <button type="button" class="btn compact" data-g="M620" title="M620">On</button>
          <button type="button" class="btn compact" data-g="M621" title="M621">Off</button>
          <span class="grow"></span><span class="readout" data-el="polar">—</span>
        </div>
        <p class="hint-sm">In polar mode the plate turns so every point is worked on the side facing the arm, and the arm only moves in its own vertical plane.</p>
      </section>

      <section class="card">
        <h3>${icon('target')} Probe results</h3>
        <div class="row">
          <button type="button" class="btn compact" data-g="G90|G38.2 Z-2 F120" title="G38.2: move down slowly until the stylus touches (needs the probe tool)">${icon('down')}<span>Probe down here</span></button>
          <span class="grow"></span>
          <button type="button" class="btn ghost compact" data-el="clearProbes">${icon('clear')}<span>Clear</span></button>
        </div>
        <div data-el="probeBody"></div>
      </section>`;

    this.el = {};
    for (const e of r.querySelectorAll('[data-el]')) this.el[e.dataset.el] = e;

    r.addEventListener('click', (ev) => {
      const b = ev.target.closest('button');
      if (!b || !r.contains(b)) return;
      if (b.dataset.g) this.send(b.dataset.g.split('|'));
      else if (b.dataset.jog) this.jog(b.dataset.jog);
      else if (b.dataset.step) {
        this.step = Number(b.dataset.step);
        for (const s of this.el.stepSeg.querySelectorAll('button')) s.setAttribute('aria-checked', String(s === b));
      } else if (b === this.el.door) {
        const m = this.getMachine();
        if (m?.enclosure) this.setDoor(!m.enclosure.doorOpen);
      } else if (b === this.el.clearProbes) {
        this.probes = []; this.renderProbes(); this.onProbeClear?.();
      }
    });
    this.renderProbes();
  }

  jog(dir) {
    const axis = dir[0], sign = dir[1] === '-' ? -1 : 1;
    const d = sign * this.step;
    const feed = axis === 'Z' ? 600 : 1500;
    this.send(['G91', `G0 ${axis}${d} F${feed}`, 'G90']);
  }

  addProbe(x, y, z, t) {
    this.probes.push({ x, y, z, t });
    this.renderProbes();
  }

  renderProbes() {
    const body = this.el.probeBody;
    if (!this.probes.length) {
      body.innerHTML = '<p class="probe-empty">No measurements yet. Pick up the probe (M6 T2) and press <b>Probe down here</b>, or load the <b>Touch probe</b> job.</p>';
      return;
    }
    const f = (v) => v.toFixed(3);
    body.innerHTML = `<table class="probe-table"><thead><tr><th>#</th><th>X mm</th><th>Y mm</th><th>Z mm</th><th>t</th></tr></thead><tbody>${
      this.probes.map((p, i) => `<tr><td>${i + 1}</td><td>${f(p.x)}</td><td>${f(p.y)}</td><td>${f(p.z)}</td><td>${p.t.toFixed(1)} s</td></tr>`).join('')
    }</tbody></table><p class="hint-sm">Heights are what the firmware believes, in plate coordinates (Z = 0 is the plate top). The difference from the true surface is the machine's own error.</p>`;
  }

  // ~5 Hz.
  refresh() {
    const m = this.getMachine();
    if (!m || !this.el.homed) return;
    const s = m.status();
    const fw = m.firmware;
    const set = (k, v) => { const e = this.el[k]; if (e && e.textContent !== v) e.textContent = v; };
    set('homed', s.homed ? 'yes' : 'no');
    this.el.homeBanner.hidden = !!s.homed;
    const toolName = { hotend: 'hot end', spindle: 'spindle', probe: 'touch probe', none: 'no tool' }[s.tool] ?? s.tool;
    set('tool', toolName);
    for (const b of this.root.querySelectorAll('.toolbtn')) b.classList.toggle('current', b.dataset.tool === s.tool);
    const tgt = (h) => (h && h.target > 0 ? ` → ${h.target.toFixed(0)} °C` : '');
    set('hot', `${s.hotend_C.toFixed(1)} °C${tgt(fw.heaters?.hotend)}`);
    set('bed', `${s.bed_C.toFixed(1)} °C${tgt(fw.heaters?.bed)}`);
    set('rpm', `${s.spindle_rpm.toFixed(0)} rpm`);
    set('polar', fw.polar ? 'on' : 'off');
    const pending = m.host.lines.length - m.host.sent;
    this.el.queueBanner.hidden = !(pending > 8);
    set('queueN', String(pending));
    const enc = m.enclosure;
    if (enc) {
      let e = {};
      try { e = enc.inspect(); } catch { /* optional */ }
      const open = !!enc.doorOpen;
      this.el.door.setAttribute('aria-checked', String(open));
      set('doorText', open ? 'Door open' : 'Door closed');
      this.el.doorBanner.hidden = !open;
      set('chamber', Number.isFinite(e.chamber_C) ? `${e.chamber_C.toFixed(1)} °C` : '—');
      set('fan', Number.isFinite(e.fan_percent) ? `${e.fan_percent.toFixed(0)} %  ${(e.exhaust_m3h ?? 0).toFixed(1)} m³/h` : '—');
    }
  }
}
