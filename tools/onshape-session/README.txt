Onshape browser-session wrapper
===============================

Superseded for agent use by src/hw/onshape_api/api.mjs and its README.txt.

Runs JSON requests inside a dedicated Playwright Chromium page authenticated
with your Onshape login. It does not read API keys, use OAuth, or export cookies.
The browser stays open in the server process; the default is headless.

Setup / start (from this directory):
  npm ci
  node cli.mjs serve

Optional environment settings:
  ONSHAPE_EMAIL          Login email (default: wyf@mit.edu)
  ONSHAPE_CHROMIUM       Path to a Playwright-compatible Chromium executable
  ONSHAPE_DOCUMENT_URL   Initial Onshape document to load

The default Chromium path uses the already-installed chromium-1208 in ~/.cache.
Use `serve --headed` to show this separate browser on the desktop.
Password is read at runtime from the literal ONSHAPE_PASSWORD assignment in
~/.bashrc. No credential is included in this directory. MFA/manual challenges
are not handled automatically. There is no API-key fallback.

Use from another terminal:
  node cli.mjs status
  node cli.mjs elements example-params.json
  node cli.mjs elements example-params.json --fresh
  node cli.mjs screenshot

Operations and parameters:
  elements       documentId, workspaceId
  features       documentId, workspaceId, elementId (Part Studio)
  source         documentId, workspaceId, elementId (Feature Studio)
  assembly       documentId, workspaceId, elementId (Assembly)
  evaluate       documentId, workspaceId, elementId, script
  updateSource   documentId, workspaceId, elementId, contents; requires --write
  insertInstances documentId, workspaceId, elementId, transformGroups; --write

Pass parameters in a JSON file. Parameter files for writes should contain the
exact reviewed source/transforms. --write is an explicit call option, not a
second interactive approval. Evaluate executes temporary FeatureScript and
does not persist CAD geometry changes.

Scope and behavior:
  - API version v14 was observed in the actual web client's network traffic.
  - Requests execute with fetch in the logged-in Onshape page, not server-side
    API-key authentication. XSRF is learned from browser requests in memory.
  - Only named CAD operations are exposed; no arbitrary cross-origin proxy.
  - Requests are serialized. GET results cache for 30 seconds; --fresh bypasses.
    A completed write invalidates the cache. Refresh after edits in another UI.
  - Errors (including 429) are returned without automatic retries. If a write
    times out, inspect live state before resending; it may already have applied.
  - Onshape version-skew checks and geometric validation remain the caller's
    responsibility. updateSource currently sends contents only; inspect source
    immediately before changes and avoid concurrent editors.
  - Private Unix socket under /tmp/onshape-session-UID, no HTTP server.
  - status captures only method/path/query parameter NAMES/status; no headers,
    query values, request bodies, passwords, cookies, or response payloads.
  - Ctrl-C stops the server and cleans up its socket. After a crash, verify no
    server is running before removing a stale socket manually.
  - Use the explicit screenshot operation for a PNG saved in the private temp
    directory. The in-app browser is a separate session from this wrapper.

Validation performed on 2026-10-10:
  Captured browser request metadata while opening Drone assembly.
  elements: HTTP 200, 14 elements; repeated read returned cached=true.
  evaluate: HTTP 200, 26 bodies, no FeatureScript notices.
  Unit checks cover path validation, read cache/fresh behavior, write guard,
  cache invalidation, and no retry on HTTP 429.
  CAD-writing operations are implemented but have NOT been live-tested.

Quota qualification:
  Onshape documents browser and session-authenticated API Explorer calls as
  excluded from annual API limits. This transport runs in a browser session;
  we have not measured the account's counter before/after as an independent
  quota audit. Rate limits still apply. Future service changes may affect this.
  https://onshape-public.github.io/docs/auth/limits/
  https://playwright.dev/docs/network
