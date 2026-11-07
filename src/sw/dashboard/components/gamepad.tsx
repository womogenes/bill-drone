'use client';

import { memo, useEffect, useRef, useState } from 'react';

// The browser's standard gamepad layout (https://w3c.github.io/gamepad/#remapping): the 8BitDo Ultimate 2 Wireless over Bluetooth reports it.
export const BUTTONS = {
  a: 0,
  b: 1,
  x: 2,
  y: 3,
  leftBumper: 4,
  rightBumper: 5,
  leftTrigger: 6,
  rightTrigger: 7,
  select: 8,
  start: 9,
  leftStick: 10,
  rightStick: 11,
  up: 12,
  down: 13,
  left: 14,
  right: 15,
  home: 16,
} as const;
const DEADZONE = 0.15;

// The first connected gamepad, or null; Chrome reports gamepads only to a focused, visible page, and only after a button press since the page loaded.
export function firstPad() {
  return (
    navigator.getGamepads?.().find((pad): pad is Gamepad => !!pad?.connected) ??
    null
  );
}

// A stick axis with the deadzone removed and the rest rescaled to -1..1.
export function stick(value: number) {
  const size = Math.abs(value);
  if (size < DEADZONE) return 0;
  return (Math.sign(value) * (Math.min(size, 1) - DEADZONE)) / (1 - DEADZONE);
}

const STICK_TRAVEL = 9;
const OFF = 'var(--muted)';
const ON = 'var(--primary)';

// A plain drawing of the controller that follows the first gamepad live. It polls on animation frames and writes the SVG's attributes directly, so the rest of the dashboard never re-renders for it.
export const GamepadView = memo(function GamepadView() {
  const svg = useRef<SVGSVGElement>(null);
  const [name, setName] = useState<string | null>(null);

  useEffect(() => {
    let frame = 0;
    let last = '';
    let shown: string | null = null;
    function draw() {
      frame = requestAnimationFrame(draw);
      const pad = firstPad();
      const id = pad
        ? `${pad.id}${pad.mapping === 'standard' ? '' : ' (not standard layout)'}`
        : null;
      if (id !== shown) {
        shown = id;
        setName(id);
      }
      const root = svg.current;
      if (!root) return;
      const buttons = pad?.buttons.map((button) => button.value) ?? [];
      const axes = pad?.axes.slice(0, 4) ?? [];
      const state = `${buttons.map((value) => value.toFixed(2))}|${axes.map((value) => value.toFixed(2))}`;
      if (state === last) return;
      last = state;
      for (const element of root.querySelectorAll<SVGElement>(
        '[data-button]'
      )) {
        const value = buttons[Number(element.dataset.button)] ?? 0;
        element.setAttribute('fill', value > 0.5 ? ON : OFF);
      }
      for (const element of root.querySelectorAll<SVGElement>(
        '[data-trigger]'
      )) {
        const value = buttons[Number(element.dataset.trigger)] ?? 0;
        element.setAttribute('height', String(12 * value));
        element.setAttribute('y', String(4 + 12 * (1 - value)));
      }
      for (const element of root.querySelectorAll<SVGElement>('[data-stick]')) {
        const index = Number(element.dataset.stick);
        const x = axes[index] ?? 0;
        const y = axes[index + 1] ?? 0;
        element.setAttribute(
          'transform',
          `translate(${x * STICK_TRAVEL} ${y * STICK_TRAVEL})`
        );
        element.setAttribute(
          'fill',
          buttons[index === 0 ? BUTTONS.leftStick : BUTTONS.rightStick] > 0.5
            ? ON
            : 'var(--foreground)'
        );
      }
    }
    frame = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(frame);
  }, []);

  const face = (cx: number, cy: number, button: number, label: string) => (
    <g key={label}>
      <circle
        cx={cx}
        cy={cy}
        r={6}
        data-button={button}
        fill={OFF}
        stroke="var(--border)"
      />
      <text
        x={cx}
        y={cy + 2.5}
        textAnchor="middle"
        fontSize={7}
        fill="var(--muted-foreground)"
      >
        {label}
      </text>
    </g>
  );
  const pad = (cx: number, cy: number, index: number) => (
    <g>
      <circle cx={cx} cy={cy} r={16} fill="none" stroke="var(--border)" />
      <circle
        cx={cx}
        cy={cy}
        r={8}
        data-stick={index}
        fill="var(--foreground)"
      />
    </g>
  );

  return (
    <div
      className={`flex items-center gap-3 text-sm text-muted-foreground transition-opacity ${name ? '' : 'opacity-40 grayscale'}`}
    >
      <svg
        ref={svg}
        viewBox="0 0 200 110"
        className="h-20 w-auto shrink-0"
        role="img"
        aria-label="Controller state"
      >
        {/* Triggers fill with their pull; bumpers below them. */}
        <rect
          x={38}
          y={4}
          width={22}
          height={12}
          rx={3}
          fill="none"
          stroke="var(--border)"
        />
        <rect
          x={38}
          y={16}
          width={22}
          height={0}
          rx={3}
          data-trigger={BUTTONS.leftTrigger}
          fill={ON}
        />
        <rect
          x={140}
          y={4}
          width={22}
          height={12}
          rx={3}
          fill="none"
          stroke="var(--border)"
        />
        <rect
          x={140}
          y={16}
          width={22}
          height={0}
          rx={3}
          data-trigger={BUTTONS.rightTrigger}
          fill={ON}
        />
        <rect
          x={32}
          y={19}
          width={34}
          height={7}
          rx={3}
          data-button={BUTTONS.leftBumper}
          fill={OFF}
          stroke="var(--border)"
        />
        <rect
          x={134}
          y={19}
          width={34}
          height={7}
          rx={3}
          data-button={BUTTONS.rightBumper}
          fill={OFF}
          stroke="var(--border)"
        />
        <path
          d="M40 28 H160 C182 28 194 50 196 78 C198 100 186 108 174 104 C160 100 150 84 134 84 H66 C50 84 40 100 26 104 C14 108 2 100 4 78 C6 50 18 28 40 28 Z"
          fill="var(--background)"
          stroke="var(--muted-foreground)"
          strokeOpacity={0.5}
        />
        {pad(48, 52, 0)}
        {pad(128, 72, 2)}
        {/* D-pad */}
        <rect
          x={68}
          y={58}
          width={8}
          height={9}
          data-button={BUTTONS.up}
          fill={OFF}
          stroke="var(--border)"
        />
        <rect
          x={68}
          y={75}
          width={8}
          height={9}
          data-button={BUTTONS.down}
          fill={OFF}
          stroke="var(--border)"
        />
        <rect
          x={59}
          y={67}
          width={9}
          height={8}
          data-button={BUTTONS.left}
          fill={OFF}
          stroke="var(--border)"
        />
        <rect
          x={76}
          y={67}
          width={9}
          height={8}
          data-button={BUTTONS.right}
          fill={OFF}
          stroke="var(--border)"
        />
        <rect
          x={86}
          y={42}
          width={9}
          height={5}
          rx={2}
          data-button={BUTTONS.select}
          fill={OFF}
          stroke="var(--border)"
        />
        <rect
          x={105}
          y={42}
          width={9}
          height={5}
          rx={2}
          data-button={BUTTONS.start}
          fill={OFF}
          stroke="var(--border)"
        />
        <circle
          cx={100}
          cy={56}
          r={4}
          data-button={BUTTONS.home}
          fill={OFF}
          stroke="var(--border)"
        />
        {face(152, 40, BUTTONS.y, 'Y')}
        {face(140, 52, BUTTONS.x, 'X')}
        {face(164, 52, BUTTONS.b, 'B')}
        {face(152, 64, BUTTONS.a, 'A')}
      </svg>
      <div className="min-w-0 flex-1 space-y-1">
        <p className="truncate font-medium text-foreground">
          {name?.replace(/\s*\(.*vendor.*\)/i, '') ?? 'No controller'}
        </p>
        <dl className="grid grid-cols-[auto_1fr] gap-x-2 text-xs">
          {[
            ['L stick', 'Tilt'],
            ['R stick', 'Climb / yaw'],
            ['A · B', 'Hold · land'],
            ['LB+RB', 'Stop'],
          ].map(([control, action]) => (
            <div key={control} className="contents">
              <dt className="font-mono">{control}</dt>
              <dd>{action}</dd>
            </div>
          ))}
        </dl>
      </div>
    </div>
  );
});
