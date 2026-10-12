Onshape browser API

Generic JSON API requests through an authenticated Playwright browser. Uses
the same method, /api/... path, query parameters and JSON body as Onshape's
documented API. No API keys, OAuth, daemon, server, endpoint catalog or retries.

INSTALL
  cd /home/willi/coding/ee/bill-drone/src/hw/onshape_api
  npm ci
  # Only if Playwright Chromium is not already installed:
  npx playwright-core install chromium

RUN
  ONSHAPE_EMAIL=wyf@mit.edu node api.mjs < requests.jsonl

One JSON request per input line, one JSON result per output line. A single
browser/login is reused for all lines. Keep stdin open for an interactive
session; EOF closes this dedicated browser. Ready/errors go to stderr.

Request fields:
  path    Required /api/... path, including query string; no external URLs.
  method  GET (default), POST, PUT, PATCH, DELETE or HEAD.
  body    Optional JSON value. Not permitted for GET/HEAD.
  out     Optional local filename: save exact response bytes instead of JSON.

Example requests.jsonl (one object per line):
  {"path":"/api/v14/documents/d/d7e760af3cc2197cafa5a123/w/0a0eede3b0e261a3b55f7eb1/elements"}
  {"method":"POST","path":"/api/v14/partstudios/d/d7e760af3cc2197cafa5a123/w/0a0eede3b0e261a3b55f7eb1/e/2accbec6a50a8bcb52c80fba/featurescript","body":{"script":"function(context is Context, queries is map) { return size(evaluateQuery(context, qEverything(EntityType.BODY))); }"}}

Generate request files using JSON.stringify/json.dump rather than shell-escaping
FeatureScript. For writes, supply the documented request body directly; there
is no additional confirmation flag. Existing authorization rules still apply.

As a library (keep the session for a batch):
  import {connect} from './api.mjs';
  const api = await connect({email: 'wyf@mit.edu'});
  try {
    const result = await api.request({path: '/api/v14/...'});
  } finally { await api.close(); }

AUTHENTICATION
Password is read from ONSHAPE_PASSWORD in the process environment, or the literal
ONSHAPE_PASSWORD assignment in ~/.bashrc. The file is parsed, never sourced.
Do not put passwords in commands, request files, tool output or git. No cookies
or credentials are saved by this tool. Set ONSHAPE_CHROMIUM to override the
browser executable; otherwise use Playwright's pinned Chromium installation.
MFA/CAPTCHA/manual login challenges are not automated.

QUOTA
Onshape says browser and session-authenticated API Explorer calls are excluded
from annual API limits. This tool executes fetch inside the signed-in browser,
using the browser's cookies and XSRF token. It never falls back to counted
API-key/OAuth authentication. An account-counter before/after audit has not
been performed, so this is not an independently measured quota guarantee.
Rate limits and normal access permissions still apply.
https://onshape-public.github.io/docs/auth/limits/

LIMITS AND ERROR HANDLING
Uses JSON request bodies. Multipart uploads are not implemented. Binary
responses work with out (or return base64 without out). Redirects are rejected;
external signed download URLs need a separate, explicitly scoped downloader.
Handle asynchronous export/translation job IDs using their documented APIs.
Responses are buffered in memory. No cache: each request is fresh.
Non-2xx responses return ok:false; the CLI exits nonzero if any request fails
but continues processing subsequent input lines. Submit dependent writes only
after inspecting previous results, rather than pipelining them blindly.
HTTP 200 is not proof of successful geometry: inspect notices/featureStatus.
After a timeout or ambiguous write failure, read live state before resending.
Use sourceMicroversion/serializationVersion guards where supported.

TEST
  npm test
Tests execute the fetch transport with simulated JSON, binary, empty and 429
responses, and check URL scoping. Live verification is read-only plus a
temporary FeatureScript evaluation; no model edits are needed.

Verified 2026-10-10: live GET returned 14 document tabs; live POST evaluation
returned 26 bodies with no notices. Both returned HTTP 200 in one login session.
