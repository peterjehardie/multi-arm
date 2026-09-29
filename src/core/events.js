// Binary min-heap of timed events. Ties are broken by insertion order so that
// the simulation is deterministic.

export class EventQueue {
  constructor() {
    this.heap = [];
    this.seq = 0;
  }
  get size() { return this.heap.length; }
  peekTime() { return this.heap.length ? this.heap[0].t : Infinity; }

  // An event is either a callback (fn) or a typed record: tag names its
  // kind and target/a/b carry its data (no code), which is what a native
  // port would store.
  push(t, fn, tag = '', target = null, a = null, b = null) {
    const ev = { t, s: this.seq++, fn, tag, target, a, b, cancelled: false };
    const h = this.heap;
    h.push(ev);
    let i = h.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (less(h[p], ev)) break;
      h[i] = h[p];
      i = p;
    }
    h[i] = ev;
    return ev;
  }

  pop() {
    const h = this.heap;
    const top = h[0];
    const last = h.pop();
    if (h.length) {
      let i = 0;
      const n = h.length;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i, mv = last;
        if (l < n && less(h[l], mv)) { m = l; mv = h[l]; }
        if (r < n && less(h[r], mv)) { m = r; mv = h[r]; }
        if (m === i) break;
        h[i] = h[m];
        i = m;
      }
      h[i] = last;
    }
    return top;
  }
}

function less(a, b) {
  return a.t < b.t || (a.t === b.t && a.s < b.s);
}
