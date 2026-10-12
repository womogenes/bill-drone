<!-- BEGIN:nextjs-agent-rules -->

# This is NOT the Next.js you know

This version has breaking changes — APIs, conventions, and file structure may all differ from your training data. Read the relevant guide in `node_modules/next/dist/docs/` (resolved from this file's directory; in monorepos the `next` package may not be visible from the repo root) before writing any code. Heed deprecation notices.

This block is written and re-added by `next dev` — verify at `node_modules/next/dist/server/lib/generate-agent-files.js`. Removing it from a diff only re-creates the uncommitted change; committing it with your work keeps the tree clean.

<!-- END:nextjs-agent-rules -->

Keep this code minimal and add only exactly what you need. No tests.

Never use monospace font unless asked. Default to use tabular nums in default font.

Use plain sections, not cards, unless explicitly requested. Keep colors muted.
Do not add unsolicited qualifiers, implementation caveats, or obvious usage instructions to the UI.
Prefer shadcn/ui components and their default styling, including charts and selects.
Never introduce custom colors without explicit user approval. Prefer shadcn theme color tokens; otherwise use Tailwind's built-in palette.
Use Tailwind red-700 for offline/error status text and icons.

Use Tailwind green-700 for green status text and icons.
