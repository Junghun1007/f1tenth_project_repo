// node --test src/auto_control/test/tunnel/remote_preview_geometry.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const html = readFileSync(new URL('../../auto_control_tunnel/web/remote_preview.html', import.meta.url), 'utf8');
const end = html.lastIndexOf('/*', html.indexOf('@pure-end'));
const pure = html.slice(html.indexOf('@pure-begin'), end).replace(/^[^\n]*\n/, '');
const lib = await import('data:text/javascript,' + encodeURIComponent(pure));

test('laneEdges offsets by half width along the path normal', () => {
  const { left, right } = lib.laneEdges([[0, 0], [1, 0]], 0.6);
  assert.deepEqual(left, [[0, 0.3], [1, 0.3]]);
  assert.deepEqual(right, [[0, -0.3], [1, -0.3]]);
  assert.deepEqual(lib.laneEdges([[0, 0]], 0.6), { left: [], right: [] });
});

test('display hides missing or stale values', () => {
  assert.equal(lib.display(null, 2, 0), '--');
  assert.equal(lib.display(1.234, 2, 600), '--');
  assert.equal(lib.display(1.234, 2, 100), '1.23');
});

test('tunnel darkness target and 1s approach', () => {
  assert.equal(lib.tunnelTarget('ENTERING', 0), 1);
  assert.equal(lib.tunnelTarget('EXITING', 1), 0);
  assert.equal(lib.tunnelTarget(null, 0.4), 0.4);
  assert.ok(Math.abs(lib.approach(0, 1, 0.6) - 0.8343) < 1e-3);
  assert.ok(Math.abs(lib.approach(0, 1, 1.0) - 0.95) < 1e-3);
  assert.equal(lib.approach(0.5, 0.5, 1), 0.5);
});
