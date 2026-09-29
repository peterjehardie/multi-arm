// Job catalogue for the Jobs rail, and a quick machine-time estimate from
// the G-code text (straight-line moves with a trapezoidal speed profile,
// dwells and tool changes). It is only a guide: the real machine also waits
// for joint speed limits, the plate turning and heaters.

import { icon } from './icons.js';

// machineTime: measured simulated seconds (demo mode, quick start, preheated);
// jobs without it get estimateTime() from their G-code.
export const JOBS = [
  {
    file: 'tour.gcode', machineTime: 216, optional: true, featured: true, icon: 'tour', tags: ['print', 'mill', 'probe'],
    title: 'Feature tour',
    blurb: 'Every tool and feature in one narrated run: fan, printing, tool changes, milling, probing and the door interlock. Best in Demo mode.',
  },
  {
    file: 'demo.gcode', machineTime: 189, icon: 'print', tags: ['print', 'mill'],
    title: 'Ring and slot',
    blurb: 'Prints a small ring on the turning plate, then swaps to the spindle and mills a slot through it.',
  },
  {
    file: 'probe.gcode', machineTime: 44, icon: 'probe', tags: ['probe'],
    title: 'Touch probe',
    blurb: 'Measures a wax block and the bare plate. The heights appear in Control and show the machine’s own errors.',
  },
  {
    file: 'mill-wax.gcode', machineTime: 115, icon: 'mill', tags: ['mill'], stock: 'wax',
    title: 'Pocket in wax',
    blurb: 'Mills a 12 × 12 mm pocket, 1 mm deep, into a 30 mm wax block (placed for you).',
  },
  {
    file: 'knob-machined.gcode', machineTime: 311, icon: 'mill', tags: ['mill'],
    title: 'Knob from wax',
    blurb: 'Carves a fluted knob out of a wax block: roughing in slabs, then one fine finishing pass.',
  },
  {
    file: 'knob-hybrid.gcode', machineTime: 554, icon: 'print', tags: ['print', 'mill'],
    title: 'Knob: print, then finish',
    blurb: 'Prints the knob slightly oversize, then finishes its dome with the ball-nose cutter. The long one.',
  },
];

const TAG_LABEL = { print: 'Print', mill: 'Mill', probe: 'Probe', tour: 'Tour' };

// Rough machine time in seconds. Assumes quick start (no homing run) and
// preheated heaters unless told otherwise.
export function estimateTime(text, { homing = false } = {}) {
  const A = 1500;           // mm/s^2 path acceleration (firmware: 1.5 m/s^2)
  const RAPID = 2400;       // mm/min for G0 without F
  let x = 0, y = 0, z = 50, f = 1200, abs = true, t = 0;
  const moveTime = (d, feed) => {
    if (d <= 0) return 0;
    const v = Math.max(1, feed) / 60;
    return d < (v * v) / A ? 2 * Math.sqrt(d / A) : d / v + v / A;
  };
  for (const raw of text.split('\n')) {
    const line = raw.replace(/;.*$/, '').trim().toUpperCase();
    if (!line) continue;
    const val = (k) => { const r = line.match(new RegExp(`${k}(-?[\\d.]+)`)); return r ? +r[1] : null; };
    if (line.startsWith('G90')) { abs = true; continue; }
    if (line.startsWith('G91')) { abs = false; continue; }
    if (/^M6\b/.test(line)) { t += 14; continue; }
    if (line.startsWith('G28')) { if (homing) t += 25; continue; }
    if (line.startsWith('M620')) { t += 2; continue; }
    if (/^M3\b/.test(line)) { t += 1.5; continue; }
    if (/^G4\b/.test(line)) { t += (val('P') ?? 0) / 1000 + (val('S') ?? 0); continue; }
    const g = line.match(/^G(0|1|38\.2)\b/);
    if (!g) continue;
    const F = val('F');
    if (F !== null) f = F;
    const X = val('X'), Y = val('Y'), Z = val('Z');
    const nx = X === null ? x : abs ? X : x + X, ny = Y === null ? y : abs ? Y : y + Y, nz = Z === null ? z : abs ? Z : z + Z;
    const d = Math.hypot(nx - x, ny - y, nz - z);
    const E = val('E');
    const feed = g[1] === '0' && F === null ? RAPID : f;
    t += d > 0 ? moveTime(d, feed) : E !== null ? Math.abs(E) / (feed / 60) : 0;
    x = nx; y = ny; z = nz;
  }
  return t * 1.15; // corners and joint limits
}

export function fmtEstimate(s) {
  if (!Number.isFinite(s)) return '';
  if (s < 60) return `about ${Math.max(5, Math.round(s / 5) * 5)} s`;
  if (s < 3600) return `about ${Math.round(s / 60)} min`;
  return `about ${(s / 3600).toFixed(1)} h`;
}

// Build the job cards. `onLoad(job)` is called by each Load button.
export function renderJobs(root, jobs, onLoad) {
  root.replaceChildren();
  const cards = new Map();
  for (const job of jobs) {
    const card = document.createElement('article');
    card.className = 'jobitem';
    card.dataset.file = job.file;
    if (job.optional) card.hidden = true;
    if (job.featured) card.classList.add('featured');
    card.innerHTML = `
      <span class="job-ico">${icon(job.icon)}</span>
      <span class="job-body">
        <span class="job-head"><span class="job-title"></span><button type="button" class="btn compact job-load">Load</button></span>
        <span class="job-blurb"></span>
        <span class="job-meta">
          ${job.featured ? '<span class="tag tag-start">Start here</span>' : ''}
          ${job.tags.map((t) => `<span class="tag tag-${t}">${TAG_LABEL[t] ?? t}</span>`).join('')}
          <span class="job-time" title="Estimated machine time (simulated time, not wall-clock time)"></span>
        </span>
        <span class="job-mini" hidden><span></span></span>
      </span>`;
    card.querySelector('.job-title').textContent = job.title;
    card.querySelector('.job-blurb').textContent = job.blurb;
    const btn = card.querySelector('button');
    btn.setAttribute('aria-label', `Load job: ${job.title}`);
    btn.addEventListener('click', () => onLoad(job));
    root.append(card);
    cards.set(job.file, card);
  }
  return cards;
}
