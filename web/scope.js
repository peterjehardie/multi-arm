// Oscilloscope: up to four recorder channels, each with its own autoscaled
// y range, over a selectable time window. Draws min/max per pixel column so
// a 20 s window at 1 kHz stays crisp and fast.
//
// The channel pickers (selects) and the live readouts can live anywhere in
// the page; `names` are optional elements that show each channel's name.
// Trace colours come from the CSS tokens --ch1..--ch4 (theme aware).

export const SCOPE_COLORS = ['#3987e5', '#d95926', '#199e70', '#c98500'];

export function scopeColors() {
  const css = getComputedStyle(document.documentElement);
  return SCOPE_COLORS.map((c, k) => css.getPropertyValue(`--ch${k + 1}`).trim() || c);
}

export class Scope {
  constructor({ canvas, selects, readouts, names = [], windowSelect, onChange }) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.selects = selects;
    this.readouts = readouts;
    this.names = names;
    this.windowSelect = windowSelect;
    this.onChange = onChange;
    this.m = null;
    selects.forEach((s) => s.addEventListener('change', () => this.changed()));
  }

  attach(m, defaults) {
    this.m = m;
    const names = [...m.recorder.channels.keys()];
    this.selects.forEach((sel, k) => {
      const prev = sel.value || defaults[k] || '';
      sel.innerHTML = '';
      sel.append(new Option('(off)', ''));
      for (const n of names) sel.append(new Option(`${n} [${m.recorder.channels.get(n).unit}]`, n));
      sel.value = names.includes(prev) ? prev : '';
    });
    this.changed();
  }

  // Set all four channels at once (missing names switch a channel off).
  setChannels(list) {
    if (!this.m) return;
    this.selects.forEach((sel, k) => {
      const n = list[k] ?? '';
      sel.value = this.m.recorder.channels.has(n) ? n : '';
    });
    this.changed();
  }

  channels() { return this.selects.map((s) => s.value); }

  changed() {
    this.names.forEach((el, k) => { if (el) el.textContent = this.selects[k]?.value || 'off'; });
    this.onChange?.(this.channels());
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    const r = this.canvas.getBoundingClientRect();
    const w = Math.max(10, Math.round(r.width * dpr)), h = Math.max(10, Math.round(r.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    this.dpr = dpr;
  }

  draw() {
    if (!this.m) return;
    this.resize();
    const { ctx, canvas } = this;
    const dpr = this.dpr, W = canvas.width, H = canvas.height;
    const css = getComputedStyle(document.documentElement);
    const bg = css.getPropertyValue('--scope-bg').trim() || '#0b0e12';
    const grid = css.getPropertyValue('--scope-grid').trim() || '#1f2630';
    const text = css.getPropertyValue('--muted').trim() || '#8a94a3';
    const colors = scopeColors();
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, W, H);

    const win = Number(this.windowSelect.value) || 5;
    const tEnd = this.m.t, t0 = tEnd - win;
    const padL = 6 * dpr, padR = 6 * dpr, padT = 16 * dpr, padB = 16 * dpr;
    const pw = W - padL - padR, ph = H - padT - padB;

    // Grid + time labels.
    ctx.strokeStyle = grid; ctx.lineWidth = 1;
    ctx.font = `${10 * dpr}px ui-monospace, SFMono-Regular, Menlo, monospace`;
    ctx.fillStyle = text; ctx.textBaseline = 'top';
    const divs = 10;
    ctx.beginPath();
    for (let i = 0; i <= divs; i++) {
      const x = Math.round(padL + (pw * i) / divs) + 0.5;
      ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph);
    }
    for (let i = 0; i <= 4; i++) {
      const y = Math.round(padT + (ph * i) / 4) + 0.5;
      ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y);
    }
    ctx.stroke();
    ctx.textAlign = 'center';
    for (let i = 0; i <= divs; i += 2) {
      const x = padL + (pw * i) / divs;
      const dt = -win + (win * i) / divs;
      ctx.fillText(i === divs ? `t=${tEnd.toFixed(2)} s` : `${dt.toFixed(win < 2 ? 1 : 0)} s`, Math.min(W - 30 * dpr, Math.max(20 * dpr, x)), padT + ph + 3 * dpr);
    }

    const active = [];
    this.selects.forEach((sel, k) => { if (sel.value) active.push({ k, name: sel.value }); });
    if (!active.length) {
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('No signals selected: choose some in the Signals tab', W / 2, H / 2);
    }
    const labelW = 110 * dpr;
    this.readouts.forEach((r) => { r.textContent = '—'; });
    active.forEach(({ k, name }, slot) => {
      const ch = this.m.recorder.channels.get(name);
      if (!ch) return;
      const [ts, vs] = this.m.recorder.series(name, win);
      let lo = Infinity, hi = -Infinity;
      for (let i = 0; i < vs.length; i++) { const v = vs[i]; if (Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
      const last = vs.length ? vs[vs.length - 1] : NaN;
      if (this.readouts[k]) this.readouts[k].textContent = Number.isFinite(last) ? `${fmt(last)} ${ch.unit}` : '—';
      if (!(hi >= lo)) return;
      let span = hi - lo;
      if (span < 1e-9 + Math.abs(hi) * 1e-4) { const c = (hi + lo) / 2, d = Math.max(Math.abs(c) * 0.05, 1e-3); lo = c - d; hi = c + d; span = hi - lo; }
      lo -= span * 0.08; hi += span * 0.08; span = hi - lo;
      const yOf = (v) => padT + ph - ((v - lo) / span) * ph;
      // Min/max per pixel column.
      const cols = Math.max(1, Math.floor(pw));
      const cmin = new Float32Array(cols).fill(Infinity), cmax = new Float32Array(cols).fill(-Infinity);
      for (let i = 0; i < ts.length; i++) {
        const v = vs[i];
        if (!Number.isFinite(v)) continue;
        const c = Math.floor(((ts[i] - t0) / win) * cols);
        if (c < 0 || c >= cols) continue;
        if (v < cmin[c]) cmin[c] = v;
        if (v > cmax[c]) cmax[c] = v;
      }
      ctx.strokeStyle = colors[k]; ctx.lineWidth = 1.25 * dpr;
      ctx.beginPath();
      let pen = false;
      for (let c = 0; c < cols; c++) {
        if (cmin[c] === Infinity) { pen = false; continue; }
        const x = padL + c;
        const y1 = yOf(cmin[c]), y2 = yOf(cmax[c]);
        if (!pen) { ctx.moveTo(x, y1); pen = true; } else ctx.lineTo(x, y1);
        if (y2 !== y1) ctx.lineTo(x, y2);
      }
      ctx.stroke();
      // Per-channel scale labels (max at top, min at bottom), side by side.
      ctx.textAlign = 'left';
      const x = padL + 4 * dpr + slot * labelW;
      const hiT = `${fmt(hi)} ${ch.unit}`, loT = `${fmt(lo)} ${ch.unit}`;
      ctx.fillStyle = bg;
      ctx.fillRect(x - 2 * dpr, padT + ph - 14 * dpr, ctx.measureText(loT).width + 4 * dpr, 13 * dpr);
      ctx.fillStyle = colors[k];
      ctx.textBaseline = 'top';
      ctx.fillText(hiT, x, 2 * dpr);
      ctx.textBaseline = 'bottom';
      ctx.fillText(loT, x, padT + ph - 2 * dpr);
    });
  }
}

function fmt(v) {
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 1000) return v.toFixed(0);
  if (a >= 100) return v.toFixed(1);
  if (a >= 10) return v.toFixed(2);
  if (a >= 0.01) return v.toFixed(3);
  return v.toExponential(1);
}
