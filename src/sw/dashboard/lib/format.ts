export function signed(value: number, digits: number) {
  const text = value.toFixed(digits);
  return text.startsWith('-') ? text : `+${text}`;
}

export function relativeTime(time: number, end: number) {
  const seconds = Math.max(0, Math.round((end - time) / 1000));
  return `T-${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}
