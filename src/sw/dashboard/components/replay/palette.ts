'use client';

import { useSyncExternalStore } from 'react';

// The replay's colors: the shadcn theme's tokens (light and dark follow the dashboard's theme) and Tailwind's orange as the one signal color, for now and what needs you (BRIEF.md). HTML parts use the token classes directly; the canvas strips and three.js need values, so the tokens are resolved here from probe elements to opaque hex over the background, again whenever the page's theme class changes.
export type Palette = {
  paper: string;
  ink: string;
  gray: string;
  hairline: string;
  signal: string;
  // The strips' line shades, first channel most prominent.
  shades: string[];
};

const PROBES = {
  paper: 'text-background',
  ink: 'text-foreground',
  gray: 'text-muted-foreground',
  hairline: 'text-border',
  signal: 'text-orange-600 dark:text-orange-500',
};

const FALLBACK: Palette = { paper: '#ffffff', ink: '#0a0a0a', gray: '#737373', hairline: '#e5e5e5', signal: '#ea580c', shades: ['#0a0a0a', '#737373', '#a3a3a3'] };

function hex(red: number, green: number, blue: number) {
  return `#${[red, green, blue].map((value) => value.toString(16).padStart(2, '0')).join('')}`;
}

// Each probe's computed color painted over the background on a one-pixel canvas and read back, which flattens oklch and alpha into a hex three.js takes.
function resolve(): Palette {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 1;
  const context = canvas.getContext('2d', { willReadFrequently: true });
  if (!context) return FALLBACK;
  const span = document.createElement('span');
  document.body.appendChild(span);
  const read = (className: string, under: string) => {
    span.className = className;
    context.fillStyle = under;
    context.fillRect(0, 0, 1, 1);
    context.fillStyle = getComputedStyle(span).color;
    context.fillRect(0, 0, 1, 1);
    const [red, green, blue] = context.getImageData(0, 0, 1, 1).data;
    return [red, green, blue];
  };
  const paper = read(PROBES.paper, '#ffffff');
  const under = hex(paper[0], paper[1], paper[2]);
  const [ink, gray, hairline, signal] = [PROBES.ink, PROBES.gray, PROBES.hairline, PROBES.signal].map((className) => read(className, under));
  span.remove();
  const mix = (a: number[], b: number[], share: number) => hex(...(a.map((value, index) => Math.round(value * share + b[index] * (1 - share))) as [number, number, number]));
  return {
    paper: under,
    ink: mix(ink, ink, 1),
    gray: mix(gray, gray, 1),
    hairline: mix(hairline, hairline, 1),
    signal: mix(signal, signal, 1),
    shades: [mix(ink, ink, 1), mix(ink, gray, 0.25), mix(gray, paper, 0.55)],
  };
}

let cache: { key: string; palette: Palette } | null = null;

function snapshot() {
  const key = document.documentElement.className;
  if (cache?.key !== key) cache = { key, palette: resolve() };
  return cache.palette;
}

function subscribe(change: () => void) {
  const observer = new MutationObserver(change);
  observer.observe(document.documentElement, { attributes: true, attributeFilter: ['class'] });
  return () => observer.disconnect();
}

export function usePalette() {
  return useSyncExternalStore(subscribe, snapshot, () => FALLBACK);
}

export function paletteStyle(palette: Palette) {
  return { '--replay-signal': palette.signal } as React.CSSProperties;
}
