import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { apiPath, request } from './api.mjs';

test('paths cannot send session credentials outside Onshape API', () => {
  assert.equal(apiPath('/api/v14/documents?limit=1'), '/api/v14/documents?limit=1');
  for (const p of ['https://example.com/api/test', '//example.com/api/test', '/api/../signin', '/api/x#fragment'])
    assert.throws(() => apiPath(p));
});
test('real fetch transport supports JSON, empty and binary responses', async () => {
  const original = globalThis.fetch;
  const folder = await mkdtemp(join(tmpdir(), 'onshape-api-test-'));
  const page = { evaluate: (fn, args) => fn(args) };
  try {
    let calls = 0;
    globalThis.fetch = async (path, options) => {
      calls++;
      assert.equal(options.credentials, 'same-origin');
      assert.equal(options.redirect, 'error');
      assert.equal(options.headers['X-XSRF-TOKEN'], 'test-token');
      assert.deepEqual(JSON.parse(options.body), { script: 'example' });
      return Response.json({ result: 26 });
    };
    const r = await request(page, 'test-token', { method: 'POST', path: '/api/v14/test', body: { script: 'example' } });
    assert.deepEqual(r.data, { result: 26 }); assert.equal(calls, 1);
    globalThis.fetch = async () => new Response(null, { status: 204 });
    assert.deepEqual(await request(page, '', { path: '/api/v14/test' }), { status: 204, data: null });
    const bytes = Buffer.from([0, 255, 1, 128]);
    globalThis.fetch = async () => new Response(bytes, { headers: { 'content-type': 'application/octet-stream' } });
    const out = join(folder, 'part.stl');
    await request(page, '', { path: '/api/v14/test', out });
    assert.deepEqual(await readFile(out), bytes);
    calls = 0;
    globalThis.fetch = async () => { calls++; return new Response('limited', { status: 429 }); };
    await assert.rejects(request(page, '', { path: '/api/v14/test' }), /HTTP 429/);
    assert.equal(calls, 1);
  } finally { globalThis.fetch = original; await rm(folder, { recursive: true }); }
});
