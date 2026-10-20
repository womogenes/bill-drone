import { connectRecorder, recordingStatus } from '@/lib/flight-recorder';

export const runtime = 'nodejs';

export async function POST(request: Request) {
  const { address } = await request.json();
  connectRecorder(address);
  return Response.json(recordingStatus());
}

export function GET() {
  return Response.json(recordingStatus());
}
