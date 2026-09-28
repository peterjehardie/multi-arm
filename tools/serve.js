#!/usr/bin/env node
// Tiny zero-dependency static file server for the browser viewer.
//
//   node tools/serve.js [--port 8080]      then open http://localhost:8080/
//
// Serves the repository root (so the page at /web/ can import ../src/...).
// "/" redirects to /web/index.html.

import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { join, normalize, extname, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = normalize(join(fileURLToPath(import.meta.url), '..', '..'));
const args = process.argv.slice(2);
const pi = args.indexOf('--port');
const PORT = Number(pi >= 0 ? args[pi + 1] : process.env.PORT ?? 8080);

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.gcode': 'text/plain; charset=utf-8',
  '.md': 'text/markdown; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.ico': 'image/x-icon',
  '.pgm': 'image/x-portable-graymap',
};

const server = createServer(async (req, res) => {
  try {
    const url = new URL(req.url, 'http://localhost');
    let path = decodeURIComponent(url.pathname);
    if (path === '/' || path === '/web') {
      res.writeHead(302, { Location: '/web/index.html' });
      return res.end();
    }
    if (path.endsWith('/')) path += 'index.html';
    const file = normalize(join(ROOT, path));
    if (file !== ROOT && !file.startsWith(ROOT + sep)) {
      res.writeHead(403); return res.end('Forbidden');
    }
    const st = await stat(file).catch(() => null);
    if (!st || !st.isFile()) {
      res.writeHead(404, { 'Content-Type': 'text/plain' });
      return res.end(`Not found: ${path}`);
    }
    const body = await readFile(file);
    res.writeHead(200, {
      'Content-Type': MIME[extname(file).toLowerCase()] ?? 'application/octet-stream',
      'Content-Length': body.length,
      'Cache-Control': 'no-cache',
    });
    res.end(req.method === 'HEAD' ? undefined : body);
  } catch (err) {
    res.writeHead(500, { 'Content-Type': 'text/plain' });
    res.end(String(err));
  }
});

// Loopback only: this serves the whole repository, so keep it off the network.
server.listen(PORT, '127.0.0.1', () => {
  console.log(`Serving ${ROOT}\n  open http://localhost:${PORT}/web/index.html`);
});
