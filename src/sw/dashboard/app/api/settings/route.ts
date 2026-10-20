import { report, settingsStatus, tune } from '@/lib/settings-bridge';

export const runtime = 'nodejs';

// Next sets x-forwarded-for from the socket unless the client sent one.
function local(request: Request) {
  const loopback = /^(127\.|::1$|::ffff:127\.|localhost$|\[::1\]$)/;
  const origin = request.headers.get('origin');
  return (
    loopback.test(request.headers.get('x-forwarded-for') ?? '') &&
    loopback.test(new URL(request.url).hostname) &&
    (!origin || loopback.test(new URL(origin).hostname))
  );
}

export function GET(request: Request) {
  if (!local(request)) return new Response(null, { status: 403 });
  return Response.json(settingsStatus());
}

export async function POST(request: Request) {
  if (!local(request)) return new Response(null, { status: 403 });
  const body = await request.json();
  if (body.control) return Response.json({ settings: report(body.control) });
  if (typeof body.settings !== 'object' || body.settings === null)
    return Response.json({ error: 'Expected settings' }, { status: 400 });
  return tune(body.settings);
}
