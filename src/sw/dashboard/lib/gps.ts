import type { GpsReading } from '@/lib/telemetry';

const EARTH_RADIUS = 6_371_000;

type Place = { latitude: number; longitude: number };

// Without a fix the module still sends its filter's placeholders (accuracy 4294967 m, a position anywhere): a position counts only from a fresh 2D or 3D fix it trusts.
export function gpsSolved(
  gps: GpsReading | null | undefined
): gps is GpsReading {
  return (
    !!gps &&
    gps.age_ms !== null &&
    gps.age_ms < 1000 &&
    gps.ok &&
    gps.fix >= 2 &&
    gps.fix <= 4
  );
}

// Meters east and north of origin: flat over a flight's few hundred meters.
export function local(origin: Place, point: Place): [number, number] {
  const radians = Math.PI / 180;
  return [
    (point.longitude - origin.longitude) *
      radians *
      EARTH_RADIUS *
      Math.cos(origin.latitude * radians),
    (point.latitude - origin.latitude) * radians * EARTH_RADIUS,
  ];
}

// Meters between two fixes.
export function distance(a: Place, b: Place) {
  return Math.hypot(...local(a, b));
}
