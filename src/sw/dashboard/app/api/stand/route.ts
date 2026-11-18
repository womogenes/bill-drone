import { standSince } from '@/lib/stand';

export const runtime = 'nodejs';
export const dynamic = 'force-dynamic';

// The stand's state and its readings after ?since=<seq> (lib/stand.ts).
export async function GET(request: Request) {
  const since = Number(new URL(request.url).searchParams.get('since') ?? 0);
  return Response.json(standSince(Number.isFinite(since) ? since : 0), {
    headers: { 'Cache-Control': 'no-store' },
  });
}
