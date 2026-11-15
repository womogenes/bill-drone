// The replay's colors (BRIEF.md): ink and its grays for data, one signal for now and what needs you. Values here, since the canvas strips and three.js read them as values; paletteStyle hands them to CSS as --replay-* for everything else.
export const PAPER = '#f3f2ee';
export const INK = '#1d1d1b';
export const GRAY = '#8b8983';
export const HAIRLINE = '#d9d7d0';
export const SIGNAL = '#e0561c';
// The strips' line shades, first channel darkest.
export const SHADES = [INK, '#6f6d68', '#a9a7a0'];

export const paletteStyle = {
  '--replay-paper': PAPER,
  '--replay-ink': INK,
  '--replay-gray': GRAY,
  '--replay-hairline': HAIRLINE,
  '--replay-signal': SIGNAL,
} as React.CSSProperties;
