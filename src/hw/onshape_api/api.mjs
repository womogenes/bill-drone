import { chromium } from 'playwright-core';
import { execFileSync } from 'node:child_process';
import { writeFile } from 'node:fs/promises';
import { createInterface } from 'node:readline';
import { pathToFileURL } from 'node:url';

const origin = 'https://cad.onshape.com';
export function apiPath(path) {
  const url = new URL(path, origin);
  if (!path.startsWith('/api/') || url.origin !== origin || !url.pathname.startsWith('/api/') || url.hash)
    throw new Error('Use an Onshape /api/... path, including any query string');
  return url.pathname + url.search;
}

// Generic transport: same method/path/JSON body as the documented API.
export async function request(page, xsrf, { method = 'GET', path, body, out }) {
  path = apiPath(path);
  if (!['GET', 'POST', 'PUT', 'PATCH', 'DELETE', 'HEAD'].includes(method)) throw new Error('Invalid HTTP method');
  if (body !== undefined && ['GET', 'HEAD'].includes(method)) throw new Error('GET/HEAD cannot have a body');
  const result = await page.evaluate(async ({ method, path, body, xsrf }) => {
    const headers = { Accept: '*/*', 'X-XSRF-TOKEN': xsrf };
    if (body !== undefined) headers['Content-Type'] = 'application/json';
    const r = await fetch(path, { method, headers, credentials: 'same-origin', redirect: 'error',
      body: body === undefined ? undefined : JSON.stringify(body), signal: AbortSignal.timeout(120000) });
    const bytes = new Uint8Array(await r.arrayBuffer());
    let binary = '';
    for (let i = 0; i < bytes.length; i += 32768) binary += String.fromCharCode(...bytes.subarray(i, i + 32768));
    return { status: r.status, contentType: r.headers.get('content-type') || '', base64: btoa(binary) };
  }, { method, path, body, xsrf });
  const bytes = Buffer.from(result.base64, 'base64');
  if (result.status < 200 || result.status >= 300) {
    // No retry: a write may have applied, and 429 must not trigger a retry loop.
    throw new Error(`Onshape HTTP ${result.status}; inspect live state before retrying a write`);
  }
  if (result.contentType.includes('text/html')) throw new Error('Unexpected HTML; session may have expired');
  const { status, contentType } = result;
  if (out) { await writeFile(out, bytes); return { status, contentType, out, bytes: bytes.length }; }
  if (!bytes.length) return { status, data: null };
  if (/json/i.test(contentType)) return { status, data: JSON.parse(bytes.toString('utf8')) };
  return { status, contentType, base64: result.base64 };
}

export async function connect({ email = process.env.ONSHAPE_EMAIL, headless = true } = {}) {
  if (!email) throw new Error('Set ONSHAPE_EMAIL');
  const browser = await chromium.launch({ headless, executablePath: process.env.ONSHAPE_CHROMIUM || chromium.executablePath() });
  try {
    const page = await browser.newPage();
    await page.goto(`${origin}/signin`);
    await page.getByPlaceholder('Email', { exact: true }).fill(email);
    await page.getByRole('button', { name: 'Continue', exact: true }).click();
    // Read the literal password assignment only; never source shell code or read API keys.
    let password = process.env.ONSHAPE_PASSWORD || execFileSync('python', ['-c',
      "import pathlib,re,shlex; s=(pathlib.Path.home()/'.bashrc').read_text(); m=re.search(r'^\\s*(?:export\\s+)?ONSHAPE_PASSWORD\\s*=\\s*(.+)$',s,re.M); assert m, 'Missing ONSHAPE_PASSWORD'; print(shlex.split(m.group(1),comments=True)[0],end='')"
    ], { encoding: 'utf8' });
    await page.getByPlaceholder('Password', { exact: true }).fill(password);
    password = undefined;
    await page.getByRole('button', { name: 'Sign in', exact: true }).click();
    await page.waitForURL('**/documents**', { timeout: 60000, waitUntil: 'domcontentloaded' });
    // This endpoint was observed in the Onshape web client's own traffic.
    const xsrf = await page.evaluate(async () => {
      const r = await fetch('/api/v14/clientinfo/xsrf', { credentials: 'same-origin', redirect: 'error' });
      if (!r.ok) throw new Error('Could not obtain session XSRF token');
      const data = await r.json();
      if (data.xsrfHeaderName?.toLowerCase() !== 'x-xsrf-token' || !data.xsrfTokenName)
        throw new Error('Unexpected XSRF metadata');
      const cookie = document.cookie.split('; ').find(c => c.startsWith(data.xsrfTokenName + '='));
      if (!cookie) throw new Error('Session XSRF cookie is missing');
      return decodeURIComponent(cookie.slice(cookie.indexOf('=') + 1));
    });
    return { page, request: spec => request(page, xsrf, spec), close: () => browser.close() };
  } catch (error) { await browser.close(); throw error; }
}

// One JSON request per input line; one JSON result per output line. EOF closes the browser.
if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  let session;
  try {
    session = await connect();
    process.stderr.write('Onshape browser session ready\n');
    const lines = createInterface({ input: process.stdin });
    for await (const line of lines) {
      if (!line.trim()) continue;
      try { console.log(JSON.stringify({ ok: true, ...await session.request(JSON.parse(line)) })); }
      catch (error) { console.log(JSON.stringify({ ok: false, error: error.message })); process.exitCode = 1; }
    }
  } catch (error) { console.error(error.message); process.exitCode = 1; }
  finally { await session?.close(); }
}
