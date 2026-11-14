import { execFile } from 'node:child_process';
import { access, readdir, readFile, writeFile } from 'node:fs/promises';
import { homedir } from 'node:os';
import { resolve } from 'node:path';
import { promisify } from 'node:util';
import { STAND_RUNS } from '@/lib/stand';

export const runtime = 'nodejs';
export const dynamic = 'force-dynamic';

const RUN_NAME = /^\d{4}-\d\d-\d\dT\d\d-\d\d-\d\d$/;
const exists = (path: string) =>
  access(path).then(
    () => true,
    () => false
  );

export type StandRun = { name: string; started: string; profile: string };

// GET lists the runs, newest first; ?name=<run> returns its analysis (sw/stand.py analyze, run once and kept as analysis.txt beside stand.png); ?name=<run>&plot returns that plot.
export async function GET(request: Request) {
  const params = new URL(request.url).searchParams;
  const name = params.get('name');
  try {
    if (name === null) {
      const runs: StandRun[] = [];
      for (const entry of await readdir(STAND_RUNS)) {
        if (!RUN_NAME.test(entry)) continue;
        const meta = await readFile(
          resolve(STAND_RUNS, entry, 'meta.json'),
          'utf8'
        )
          .then(JSON.parse)
          .catch(() => null);
        if (!meta) continue;
        const [day, time] = entry.split('T');
        runs.push({
          name: entry,
          started: `${day}T${time.replaceAll('-', ':')}Z`,
          profile: meta.profile,
        });
      }
      runs.sort((a, b) => b.name.localeCompare(a.name));
      return Response.json(runs, { headers: { 'Cache-Control': 'no-store' } });
    }
    // Only a run's own name, so the query cannot reach other files.
    if (!RUN_NAME.test(name))
      return Response.json({ error: 'Unknown run' }, { status: 404 });
    const directory = resolve(STAND_RUNS, name);
    const text = resolve(directory, 'analysis.txt');
    if (
      !(await exists(text)) ||
      !(await exists(resolve(directory, 'stand.png')))
    ) {
      const { stdout } = await promisify(execFile)(
        resolve(homedir(), '.local/bin/uv'),
        ['run', '-m', 'sw.stand', 'analyze', directory],
        { cwd: resolve(process.cwd(), '../..'), timeout: 120_000 }
      );
      await writeFile(text, stdout);
    }
    if (params.has('plot'))
      return new Response(await readFile(resolve(directory, 'stand.png')), {
        headers: { 'Content-Type': 'image/png', 'Cache-Control': 'no-store' },
      });
    return Response.json(
      { analysis: await readFile(text, 'utf8') },
      { headers: { 'Cache-Control': 'no-store' } }
    );
  } catch (error) {
    return Response.json(
      { error: error instanceof Error ? error.message : String(error) },
      { status: 500 }
    );
  }
}
