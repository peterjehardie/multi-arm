// Inline SVG icons (24 x 24, stroke = currentColor). No icon fonts, no network.

const P = {
  play: '<path d="M7 5v14l12-7z" fill="currentColor" stroke="none"/>',
  pause: '<rect x="6" y="5" width="4" height="14" rx="1" fill="currentColor" stroke="none"/><rect x="14" y="5" width="4" height="14" rx="1" fill="currentColor" stroke="none"/>',
  reset: '<path d="M4 12a8 8 0 1 0 2.4-5.7"/><path d="M4 4v5h5"/>',
  help: '<circle cx="12" cy="12" r="9"/><path d="M9.5 9.5a2.5 2.5 0 1 1 3.5 2.3c-.7.3-1 .9-1 1.6v.6"/><circle cx="12" cy="17" r=".6" fill="currentColor"/>',
  sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/>',
  moon: '<path d="M20 14.5A8 8 0 0 1 9.5 4a8 8 0 1 0 10.5 10.5z"/>',
  auto: '<circle cx="12" cy="12" r="8.5"/><path d="M12 3.5v17a8.5 8.5 0 0 0 0-17z" fill="currentColor" stroke="none"/>',
  inspect: '<circle cx="11" cy="11" r="6.5"/><path d="M20 20l-4.3-4.3"/><path d="M11 8.5v5M8.5 11h5"/>',
  tree: '<rect x="3" y="3.5" width="6" height="5" rx="1"/><rect x="13" y="9.5" width="8" height="4.5" rx="1"/><rect x="13" y="16" width="8" height="4.5" rx="1"/><path d="M6 8.5v9.7h7M6 11.8h7"/>',
  control: '<circle cx="12" cy="12" r="8.5"/><path d="M12 6.5l2 2.5h-4zM12 17.5l2-2.5h-4zM6.5 12l2.5-2v4zM17.5 12l-2.5-2v4z" fill="currentColor" stroke="none"/>',
  wave: '<path d="M2 12h3l2.5-6 4 12 3-9 2 3H22"/>',
  terminal: '<rect x="3" y="4.5" width="18" height="15" rx="2"/><path d="M7 9.5l3 2.5-3 2.5M12.5 15h4.5"/>',
  jobs: '<path d="M4 7h16M4 12h16M4 17h10"/>',
  upload: '<path d="M12 16V4M7 9l5-5 5 5"/><path d="M4 16v3a1 1 0 0 0 1 1h14a1 1 0 0 0 1-1v-3"/>',
  print: '<path d="M8 4h8l-1 5H9z"/><path d="M12 9v4"/><path d="M5 20h14M7 17h10"/>',
  mill: '<path d="M10 3h4v6h-4z"/><path d="M11 9h2l.5 7-1.5 2-1.5-2z"/><path d="M4 20h16"/>',
  probe: '<path d="M12 3v11"/><circle cx="12" cy="16" r="2.2"/><path d="M8 3h8M4 21h16"/>',
  tour: '<circle cx="12" cy="12" r="8.5"/><path d="M15.5 8.5l-2 5-5 2 2-5z"/>',
  home: '<path d="M4 11l8-7 8 7"/><path d="M6 10v10h12V10"/>',
  pin: '<circle cx="12" cy="10" r="3"/><path d="M12 21s-7-6-7-11a7 7 0 0 1 14 0c0 5-7 11-7 11z"/>',
  door: '<path d="M6 21V3h12v18"/><path d="M4 21h16"/><circle cx="15" cy="12" r=".9" fill="currentColor"/>',
  fan: '<circle cx="12" cy="12" r="1.8"/><path d="M12 10.2c-1-3.5.5-6.2 3-6.2 2 0 2.6 2.7 0 4.2zM13.8 12c3.5-1 6.2.5 6.2 3 0 2-2.7 2.6-4.2 0zM12 13.8c1 3.5-.5 6.2-3 6.2-2 0-2.6-2.7 0-4.2zM10.2 12c-3.5 1-6.2-.5-6.2-3 0-2 2.7-2.6 4.2 0z"/>',
  flame: '<path d="M12 3c1 3.5 5 5.5 5 10a5 5 0 0 1-10 0c0-2.5 1.5-3.5 2-5.5 1 1.2 1.5 2 1.5 3.5C12 8 12.5 6 12 3z"/>',
  spin: '<path d="M20 12a8 8 0 1 1-3-6.2"/><path d="M20 4v5h-5"/>',
  polar: '<circle cx="12" cy="12" r="8.5"/><circle cx="12" cy="12" r="4"/><path d="M12 3.5V12l6 6"/>',
  target: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3.5"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>',
  up: '<path d="M12 19V5M6 11l6-6 6 6"/>',
  down: '<path d="M12 5v14M6 13l6 6 6-6"/>',
  left: '<path d="M19 12H5M11 6l-6 6 6 6"/>',
  right: '<path d="M5 12h14M13 6l6 6-6 6"/>',
  chevron: '<path d="M6 9l6 6 6-6"/>',
  close: '<path d="M6 6l12 12M18 6L6 18"/>',
  cube: '<path d="M12 3l8 4.5v9L12 21l-8-4.5v-9z"/><path d="M12 12l8-4.5M12 12v9M12 12L4 7.5"/>',
  plate: '<ellipse cx="12" cy="13" rx="9" ry="4"/><path d="M3 13v2c0 2.2 4 4 9 4s9-1.8 9-4v-2"/>',
  follow: '<circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/>',
  wires: '<path d="M3 17c4 0 4-10 9-10s5 10 9 10"/><path d="M3 12h3M18 12h3"/>',
  path: '<path d="M4 19c3-9 6 1 8-6s5-3 8-8"/><circle cx="4" cy="19" r="1.5" fill="currentColor"/>',
  link: '<path d="M9 15l6-6"/><path d="M11 6.5l1.5-1.5a4 4 0 0 1 5.7 5.7L16.7 12M13 17.5L11.5 19a4 4 0 0 1-5.7-5.7L7.3 12"/>',
  box: '<path d="M4 8l8-4 8 4v8l-8 4-8-4z"/><path d="M4 8l8 4 8-4M12 12v8"/>',
  bolt: '<path d="M13 2L5 14h6l-1 8 8-12h-6z"/>',
  chip: '<rect x="6" y="6" width="12" height="12" rx="1.5"/><path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4"/>',
  gear: '<circle cx="12" cy="12" r="3"/><path d="M12 2.5l1.6 2.6 3-.6.6 3 2.6 1.6-1.4 2.9 1.4 2.9-2.6 1.6-.6 3-3-.6L12 21.5l-1.6-2.6-3 .6-.6-3-2.6-1.6L5.6 12 4.2 9.1l2.6-1.6.6-3 3 .6z"/>',
  arm: '<circle cx="6" cy="18" r="2"/><path d="M4 21h6M6 16l4-8 6 3"/><circle cx="10" cy="8" r="1.5"/><path d="M16 11l3-2"/>',
  wrench: '<path d="M14.5 6.5a4 4 0 0 0 5 5L12 19a2.1 2.1 0 0 1-3-3l7.5-7.5a4 4 0 0 1-2-2z"/>',
  thermo: '<path d="M10 14V5a2 2 0 0 1 4 0v9a4 4 0 1 1-4 0z"/><path d="M12 9v7"/>',
  clear: '<path d="M4 7h16M9 7V4h6v3M6 7l1 13h10l1-13"/>',
  search: '<circle cx="11" cy="11" r="6.5"/><path d="M20 20l-4.3-4.3"/>',
  layers: '<path d="M12 3l9 5-9 5-9-5z"/><path d="M3 13l9 5 9-5"/>',
  camera: '<rect x="3" y="7" width="18" height="12" rx="2"/><circle cx="12" cy="13" r="3.5"/><path d="M8.5 7l1.5-2.5h4L15.5 7"/>',
  keyboard: '<rect x="2.5" y="6" width="19" height="12" rx="2"/><path d="M6 10h.01M9.5 10h.01M13 10h.01M16.5 10h.01M7 14h10"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v6"/><circle cx="12" cy="7.7" r=".7" fill="currentColor"/>',
  warn: '<path d="M12 3l10 18H2z"/><path d="M12 10v5"/><circle cx="12" cy="18" r=".7" fill="currentColor"/>',
  check: '<path d="M5 12.5l4.5 4.5L19 7.5"/>',
  send: '<path d="M4 12l16-8-6 16-2.5-6.5z"/>',
  enclosure: '<rect x="3.5" y="4" width="17" height="16" rx="1"/><path d="M12 4v16M3.5 9h17"/>',
  panel: '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M15 4v16"/>',
  collapse: '<path d="M6 15l6-6 6 6"/>',
};

export function icon(name, cls = '') {
  return `<svg class="ico ${cls}" viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">${P[name] ?? ''}</svg>`;
}

// Replace every <i data-icon="name"></i> placeholder in the document.
export function hydrateIcons(root = document) {
  for (const el of root.querySelectorAll('i[data-icon]')) {
    el.outerHTML = icon(el.dataset.icon, el.className);
  }
}
