// First-visit guided tour: a spotlight on each key area with a short card.
// Remembered in localStorage (every access wrapped: the page may run in a
// sandbox where storage throws).

const KEY = 'multiarm.tourDone.v1';

export function storageGet(k) { try { return window.localStorage.getItem(k); } catch { return null; } }
export function storageSet(k, v) { try { window.localStorage.setItem(k, v); } catch { /* storage unavailable */ } }

export const STEPS = [
  {
    target: null,
    title: 'Welcome to the workshop',
    text: 'A small DIY robot arm stands next to a turning build plate. It swaps tools on its wrist: a hot end to print, a spindle to mill and a touch probe to measure. Every signal, torque and heat flow goes through a physical part you can click on.',
  },
  {
    target: '[data-tour="jobs"]',
    title: 'Start with a job',
    text: 'Each card is a ready-made G-code job with what it shows and roughly how long it takes. Press Load, then Run. You can also open your own STL model and slice it right here.',
  },
  {
    target: '[data-tour="run"]',
    title: 'Run, pause, speed',
    text: 'Run and pause with this button or the space bar. Reset rebuilds the machine. Speed sets simulated time against real time; Max goes as fast as your computer can.',
  },
  {
    target: '[data-tour="mode"]',
    title: 'Demo or full physics',
    text: 'Demo mode simplifies the physics (ideal motors, a kinematic arm) so jobs finish quickly. Full physics simulates every coil current, gearbox and wire: slower, but it behaves like the real build.',
  },
  {
    target: '[data-tour="view"]',
    title: 'Look and click',
    text: 'Drag to orbit, right-drag to pan, scroll to zoom. Click any part or wire to inspect it. Top right: camera views. Bottom left: layers such as wire current and the planned toolpath.',
  },
  {
    target: '[data-tour="panel"]',
    title: 'Inspect, explore, control',
    text: 'Inspect explains the selected part and shows its live values. Explore lists the machine by subsystem. Control lets you home, jog, change tools, open the door or run the fan. Signals and Console show the scope setup and the firmware messages.',
  },
  {
    target: '[data-tour="scope"]',
    title: 'Watch the signals',
    text: 'The scope plots four recorded signals live: supply voltage, coil currents, temperatures, tip error and more. Click a channel chip, or open Signals, to choose what to watch. Press ? at any time for help and shortcuts.',
  },
];

export class Tour {
  constructor({ root, spot, card, stepEl, titleEl, textEl, dots, back, next, skip, onClose }) {
    Object.assign(this, { root, spot, card, stepEl, titleEl, textEl, dots, back, next, skip, onClose });
    this.i = 0;
    back.addEventListener('click', () => this.go(this.i - 1));
    next.addEventListener('click', () => (this.i >= STEPS.length - 1 ? this.close() : this.go(this.i + 1)));
    skip.addEventListener('click', () => this.close());
    root.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); this.close(); }
      else if (e.key === 'ArrowRight') { e.preventDefault(); next.click(); }
      else if (e.key === 'ArrowLeft' && this.i > 0) { e.preventDefault(); this.go(this.i - 1); }
      else if (e.key === 'Tab') this.trapFocus(e);
    });
    this.onResize = () => { if (!this.root.hidden) this.place(); };
    window.addEventListener('resize', this.onResize);
    window.addEventListener('scroll', this.onResize, true);
  }

  static shouldAutoStart() { return storageGet(KEY) !== '1'; }

  open(start = 0) {
    this.lastFocus = document.activeElement;
    this.root.hidden = false;
    this.go(start);
  }

  close() {
    this.root.hidden = true;
    storageSet(KEY, '1');
    this.onClose?.();
    this.lastFocus?.focus?.();
  }

  get isOpen() { return !this.root.hidden; }

  go(i) {
    this.i = Math.max(0, Math.min(STEPS.length - 1, i));
    const s = STEPS[this.i];
    this.stepEl.textContent = `Step ${this.i + 1} of ${STEPS.length}`;
    this.titleEl.textContent = s.title;
    this.textEl.textContent = s.text;
    this.back.hidden = this.i === 0;
    this.next.textContent = this.i === STEPS.length - 1 ? 'Start exploring' : 'Next';
    this.dots.innerHTML = STEPS.map((_, k) => `<span class="${k === this.i ? 'on' : ''}"></span>`).join('');
    const el = s.target ? document.querySelector(s.target) : null;
    if (el && el.getClientRects().length) {
      const r = el.getBoundingClientRect();
      if (r.top < 0 || r.bottom > window.innerHeight) el.scrollIntoView({ block: 'center' });
    }
    this.place();
    this.next.focus();
  }

  // Position the spotlight on the target and the card next to it.
  place() {
    const s = STEPS[this.i];
    const el = s.target ? document.querySelector(s.target) : null;
    const vw = window.innerWidth, vh = window.innerHeight;
    const cw = this.card.offsetWidth || 360, ch = this.card.offsetHeight || 220;
    const r = el && el.getClientRects().length ? el.getBoundingClientRect() : null;
    if (!r) {
      this.spot.classList.add('center');
      this.card.style.left = `${Math.max(16, (vw - cw) / 2)}px`;
      this.card.style.top = `${Math.max(16, (vh - ch) / 2)}px`;
      return;
    }
    this.spot.classList.remove('center');
    const pad = 4;
    const L = Math.max(4, r.left - pad), T = Math.max(4, r.top - pad);
    const R = Math.min(vw - 4, r.right + pad), B = Math.min(vh - 4, r.bottom + pad);
    Object.assign(this.spot.style, { left: `${L}px`, top: `${T}px`, width: `${Math.max(0, R - L)}px`, height: `${Math.max(0, B - T)}px` });
    // Prefer right, then left, then below, then above, else overlay inside.
    const gap = 14;
    let x, y;
    if (R + gap + cw <= vw - 8) { x = R + gap; y = T; }
    else if (L - gap - cw >= 8) { x = L - gap - cw; y = T; }
    else if (B + gap + ch <= vh - 8) { x = (L + R) / 2 - cw / 2; y = B + gap; }
    else if (T - gap - ch >= 8) { x = (L + R) / 2 - cw / 2; y = T - gap - ch; }
    else { x = (vw - cw) / 2; y = Math.min(vh - ch - 16, Math.max(16, (T + B) / 2 - ch / 2)); }
    x = Math.min(vw - cw - 8, Math.max(8, x));
    y = Math.min(vh - ch - 8, Math.max(8, y));
    this.card.style.left = `${x}px`;
    this.card.style.top = `${y}px`;
  }

  trapFocus(e) {
    const f = [...this.card.querySelectorAll('button')].filter((b) => !b.hidden);
    if (!f.length) return;
    const first = f[0], last = f[f.length - 1];
    if (e.shiftKey && document.activeElement === first) { e.preventDefault(); last.focus(); }
    else if (!e.shiftKey && document.activeElement === last) { e.preventDefault(); first.focus(); }
  }
}
