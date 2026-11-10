export function signed(value: number, digits: number) {
  const text = value.toFixed(digits);
  return text.startsWith('-') ? text : `+${text}`;
}

export function relativeTime(time: number, end: number) {
  const seconds = Math.max(0, Math.round((end - time) / 1000));
  return `T-${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

// Seconds as m:ss.
export function clock(seconds: number) {
  const whole = Math.max(0, Math.round(seconds));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, '0')}`;
}

// A recording's start in this laptop's local time, which is how flights are named.
export function startTime(iso: string) {
  return new Date(iso).toLocaleString(undefined, {
    month: 'short',
    day: 'numeric',
    hour: 'numeric',
    minute: '2-digit',
    second: '2-digit',
  });
}
