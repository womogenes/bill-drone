export function signed(value: number, digits: number) {
  const text = value.toFixed(digits);
  return text.startsWith('-') ? text : `+${text}`;
}
