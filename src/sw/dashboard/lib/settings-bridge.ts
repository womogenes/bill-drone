import { CONTROL_KEYS } from './protocol';
import type { ControlSettings } from './telemetry';

// Lets local tools tune through the open dashboard page, which owns the drone connection.
type Bridge = {
  reported?: { settings: ControlSettings; time: number };
  pending: Record<string, number>;
};
const host = globalThis as typeof globalThis & { settingsBridge?: Bridge };
const state = (host.settingsBridge ??= { pending: {} });
const TUNABLE_KEYS: readonly string[] = CONTROL_KEYS.slice(5);

function connected() {
  return !!state.reported && Date.now() - state.reported.time < 2000;
}

export function settingsStatus() {
  return { connected: connected(), ...state.reported };
}

export function report(settings: ControlSettings) {
  state.reported = { settings, time: Date.now() };
  const pending = state.pending;
  state.pending = {};
  return pending;
}

export async function tune(settings: Record<string, unknown>) {
  if (!connected())
    return Response.json({ error: 'Dashboard not connected' }, { status: 503 });
  const reported = state.reported?.settings as Record<string, unknown>;
  for (const [key, value] of Object.entries(settings)) {
    if (!TUNABLE_KEYS.includes(key) || reported[key] === undefined)
      return Response.json(
        { error: `Unknown setting ${key}` },
        { status: 400 }
      );
    if (typeof value !== 'number' || !Number.isFinite(value))
      return Response.json({ error: `${key} must be finite` }, { status: 400 });
  }

  const requested = Object.entries(settings as Record<string, number>);
  const time = Date.now();
  Object.assign(state.pending, settings);
  while (Date.now() - time < 3000) {
    await new Promise((resolve) => setTimeout(resolve, 50));
    const current = state.reported;
    if (
      current &&
      current.time > time &&
      requested.every(
        ([key, value]) =>
          current.settings[key as keyof ControlSettings] === Math.fround(value)
      )
    )
      return Response.json(settingsStatus());
  }
  return Response.json(settingsStatus(), { status: 504 });
}
