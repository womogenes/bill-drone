# 3D printing workflow

- Use the Creality Print GUI through available computer-use tools to prepare and launch prints. Do not substitute a slicer CLI or direct printer API for this workflow.
- Before starting a print, inspect and confirm all applicable printer, filament, process, object-specific, and print-dialog settings, and review the sliced preview. Present the confirmed settings to the user before launch; resolve any unconfirmed choices first.
- Read `~/README.md` for the machine's computer-use setup. On the host inspected on 2026-10-06, `/home/willi/README.md` contained system notes but no Creality/CUA instructions; do not assume native GUI control is available without checking the active host/tools.
- Track print progress using the go2rtc feed at http://thinkmax-lenovo-02:1984/.
- When the user asks to watch a print, show actual captured print frames in commentary alongside progress observations. Do not claim monitoring without checking the feed.
- Do not use image generation for real engineering designs.

# Onshape API budget

- Prefer the minimal browser-session transport in `src/hw/onshape_api/api.mjs` for CAD API calls. Read `src/hw/onshape_api/README.txt` first; it accepts documented API paths and JSON bodies without API-key authentication. Do not silently fall back to API keys/OAuth. Browser-session annual-quota exemption is documented by Onshape; rate limits still apply.

- Conserve Onshape API calls; the user reported roughly 25% of the annual quota used on 2026-10-10.
- Reuse retrieved source, IDs, schemas, and results within a task. Inspect live state before edits, but avoid redundant reads and polling.
- Prepare changes locally, combine related geometry checks into one FeatureScript evaluation, and verify the saved result once. Request additional renders only when needed.
- Prefer browser UI for routine visual inspection when available. Running API calls in parallel does not reduce quota usage.
- Published limits and counting rules: https://onshape-public.github.io/docs/auth/limits/ . Check current documentation when quoting quotas; do not assume the user's subscription tier.

## Code style

- Never prefix variable or function names with underscores.
- Double quotes for Python strings.
