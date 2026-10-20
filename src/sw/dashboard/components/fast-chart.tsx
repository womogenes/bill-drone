'use client';

import { useEffect, useRef } from 'react';

export type ChartSeries<T> = {
  label: string;
  color: string;
  value: (point: T) => number | null;
};

export function FastChart<T>({
  history,
  series,
  time,
  start,
  end,
  formatTime,
  formatValue,
  gap,
  label,
}: {
  history: (start: number, end: number) => readonly T[];
  series: readonly ChartSeries<T>[];
  time: (point: T) => number;
  start: number;
  end: number;
  formatTime: (time: number, end: number) => string;
  formatValue: (value: number) => string;
  gap?: (previous: T, next: T) => boolean;
  label: string;
}) {
  const container = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const tooltip = useRef<HTMLDivElement>(null);
  const pointer = useRef<number | null>(null);

  const current = useRef({
    history,
    series,
    time,
    start,
    end,
    formatTime,
    formatValue,
    gap,
  });
  const redraw = useRef<() => void>(() => {});

  useEffect(() => {
    const host = container.current!;
    const surface = canvas.current!;
    const tip = tooltip.current!;
    const context = surface.getContext('2d')!;
    let frame = 0;
    let colors: string[] = [];
    let font = '';
    let styleDirty = true;

    function draw() {
      frame = 0;
      const {
        history,
        series,
        time,
        start,
        end,
        formatTime,
        formatValue,
        gap,
      } = current.current;
      const visible = history(start, end);
      const width = host.clientWidth;
      const height = host.clientHeight;
      const ratio = window.devicePixelRatio || 1;
      if (
        surface.width !== Math.round(width * ratio) ||
        surface.height !== Math.round(height * ratio)
      ) {
        surface.width = Math.round(width * ratio);
        surface.height = Math.round(height * ratio);
      }
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.clearRect(0, 0, width, height);
      if (styleDirty) {
        font = `12px ${getComputedStyle(host).fontFamily}`;
        colors = Array.from(
          host.querySelectorAll<HTMLElement>('[data-chart-color]'),
          (probe) => getComputedStyle(probe).color
        );
        styleDirty = false;
      }
      context.font = font;
      let low = Infinity;
      let high = -Infinity;
      for (const point of visible) {
        for (const line of series) {
          const value = line.value(point);
          if (value !== null && Number.isFinite(value)) {
            low = Math.min(low, value);
            high = Math.max(high, value);
          }
        }
      }
      if (low === Infinity) {
        low = -1;
        high = 1;
      }
      const padding =
        (high - low) * 0.08 || Math.max(Math.abs(low) * 0.05, 0.01);
      low -= padding;
      high += padding;
      const ticks = Array.from(
        { length: 5 },
        (_, index) => low + ((high - low) * index) / 4
      );
      const left = Math.max(
        48,
        ...ticks.map(
          (value) => context.measureText(formatValue(value)).width + 12
        )
      );
      const right = width - 12;
      const top = 8;
      const bottom = height - 36;
      const x = (value: number) =>
        left + ((value - start) / (end - start)) * (right - left);
      const y = (value: number) =>
        bottom - ((value - low) / (high - low)) * (bottom - top);
      context.lineWidth = 1;
      context.textAlign = 'right';
      context.textBaseline = 'middle';
      for (const value of ticks) {
        context.strokeStyle = colors[1];
        context.beginPath();
        context.moveTo(left, y(value));
        context.lineTo(right, y(value));
        context.stroke();
        context.fillStyle = colors[0];
        context.fillText(formatValue(value), left - 8, y(value));
      }
      context.textBaseline = 'top';
      for (const [index, value] of [start, (start + end) / 2, end].entries()) {
        context.textAlign =
          index === 0 ? 'left' : index === 2 ? 'right' : 'center';
        context.fillText(formatTime(value, end), x(value), bottom + 12);
      }
      context.save();
      context.beginPath();
      context.rect(left, top, Math.max(0, right - left), bottom - top);
      context.clip();
      series.forEach((line, index) => {
        context.strokeStyle = colors[index + 2];
        context.lineWidth = 1.5;
        context.beginPath();
        let previous: T | undefined;
        for (const point of visible) {
          const value = line.value(point);
          if (value === null || !Number.isFinite(value)) {
            previous = undefined;
            continue;
          }
          if (previous === undefined || gap?.(previous, point))
            context.moveTo(x(time(point)), y(value));
          else context.lineTo(x(time(point)), y(value));
          previous = point;
        }
        context.stroke();
      });
      tip.hidden = true;
      const position = pointer.current;
      if (
        position !== null &&
        position >= left &&
        position <= right &&
        visible.length
      ) {
        const target =
          start + ((position - left) / (right - left)) * (end - start);
        // Binary search the nearest timestamp in the sorted history.
        let lo = 0;
        let hi = visible.length - 1;
        while (lo < hi) {
          const mid = Math.floor((lo + hi) / 2);
          if (time(visible[mid]) < target) lo = mid + 1;
          else hi = mid;
        }
        const point =
          lo > 0 && target - time(visible[lo - 1]) < time(visible[lo]) - target
            ? visible[lo - 1]
            : visible[lo];
        const pointX = x(time(point));
        context.strokeStyle = colors[1];
        context.beginPath();
        context.moveTo(pointX, top);
        context.lineTo(pointX, bottom);
        context.stroke();
        tip.children[0].textContent = formatTime(time(point), end);
        series.forEach((line, index) => {
          const value = line.value(point);
          tip.children[index + 1].children[1].textContent =
            value === null ? '—' : formatValue(value);
          if (value !== null) {
            context.fillStyle = colors[index + 2];
            context.beginPath();
            context.arc(pointX, y(value), 3, 0, Math.PI * 2);
            context.fill();
          }
        });
        tip.hidden = false;
        tip.style.left = `${Math.max(0, Math.min(width - tip.offsetWidth, pointX + 12))}px`;
      }
      context.restore();
    }

    function schedule() {
      if (!frame) frame = requestAnimationFrame(draw);
    }
    function move(event: PointerEvent) {
      pointer.current = event.clientX - host.getBoundingClientRect().left;
      schedule();
    }
    function leave() {
      pointer.current = null;
      schedule();
    }
    redraw.current = schedule;
    function refreshStyle() {
      styleDirty = true;
      schedule();
    }
    const resize = new ResizeObserver(refreshStyle);
    resize.observe(host);
    const theme = new MutationObserver(refreshStyle);
    theme.observe(host.querySelector('[data-chart-palette]')!, {
      attributes: true,
      childList: true,
      subtree: true,
    });
    theme.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['class', 'style'],
    });
    surface.addEventListener('pointermove', move);
    surface.addEventListener('pointerleave', leave);
    schedule();
    return () => {
      cancelAnimationFrame(frame);
      redraw.current = () => {};
      resize.disconnect();
      theme.disconnect();
      surface.removeEventListener('pointermove', move);
      surface.removeEventListener('pointerleave', leave);
    };
  }, []);

  useEffect(() => {
    current.current = {
      history,
      series,
      time,
      start,
      end,
      formatTime,
      formatValue,
      gap,
    };
    redraw.current();
  }, [history, series, time, start, end, formatTime, formatValue, gap]);

  return (
    <div ref={container} className="relative h-40 w-full">
      <canvas
        ref={canvas}
        className="block h-full w-full"
        role="img"
        aria-label={label}
      />
      <div data-chart-palette className="hidden" aria-hidden="true">
        <span data-chart-color className="text-muted-foreground" />
        <span data-chart-color className="text-border" />
        {series.map((line) => (
          <span
            key={line.label}
            data-chart-color
            style={{ color: line.color }}
          />
        ))}
      </div>
      <div
        ref={tooltip}
        hidden
        className="pointer-events-none absolute top-4 z-10 min-w-32 rounded-lg border bg-background px-3 py-2 text-xs tabular-nums shadow-xl"
      >
        <div className="mb-1 font-medium" />
        {series.map((line) => (
          <div key={line.label} className="flex justify-between gap-4 py-0.5">
            <span style={{ color: line.color }}>{line.label}</span>
            <span />
          </div>
        ))}
      </div>
    </div>
  );
}
