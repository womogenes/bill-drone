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

const STICK_TRAVEL = 8;
const TRIGGER_TOP = 6;
const TRIGGER_HEIGHT = 14;
const OFF = 'var(--background)';
const ON = 'var(--primary)';
const KNOB = 'var(--muted-foreground)';
const LABEL = 'var(--muted-foreground)';

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
      for (const element of root.querySelectorAll<SVGElement>('[data-label]')) {
        const value = buttons[Number(element.dataset.label)] ?? 0;
        element.setAttribute(
          'fill',
          value > 0.5 ? 'var(--primary-foreground)' : LABEL
        );
      }
      for (const element of root.querySelectorAll<SVGElement>(
        '[data-trigger]'
      )) {
        const value = buttons[Number(element.dataset.trigger)] ?? 0;
        element.setAttribute('height', String(TRIGGER_HEIGHT * value));
        element.setAttribute(
          'y',
          String(TRIGGER_TOP + TRIGGER_HEIGHT * (1 - value))
        );
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
            : KNOB
        );
      }
    }
    frame = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(frame);
  }, []);

  const line = { stroke: 'var(--border)', strokeWidth: 1.5 };
  const key = (button: number) => ({
    'data-button': button,
    fill: OFF,
    ...line,
  });
  const face = (cx: number, cy: number, button: number, label: string) => (
    <g key={label}>
      <circle cx={cx} cy={cy} r={7.5} {...key(button)} />
      <text
        x={cx}
        y={cy + 3}
        textAnchor="middle"
        fontSize={8.5}
        fontWeight={600}
        data-label={button}
        fill={LABEL}
      >
        {label}
      </text>
    </g>
  );
  const pad = (cx: number, cy: number, index: number) => (
    <g>
      <circle cx={cx} cy={cy} r={17} fill={OFF} {...line} />
      <circle cx={cx} cy={cy} r={10.5} data-stick={index} fill={KNOB} />
    </g>
  );
  const trigger = (x: number, button: number) => (
    <g>
      <rect
        x={x}
        y={TRIGGER_TOP}
        width={30}
        height={TRIGGER_HEIGHT}
        rx={4}
        fill={OFF}
        {...line}
      />
      <rect
        x={x}
        y={TRIGGER_TOP + TRIGGER_HEIGHT}
        width={30}
        height={0}
        rx={4}
        data-trigger={button}
        fill={ON}
      />
    </g>
  );
  // The D-pad's arms around its center (94, 102): 9 wide, 10 long.
  const arm = (
    button: number,
    x: number,
    y: number,
    width: number,
    height: number
  ) => (
    <rect x={x} y={y} width={width} height={height} rx={2} {...key(button)} />
  );

  return (
    <div
      className={`flex items-center gap-4 text-sm text-muted-foreground transition-opacity ${name ? '' : 'opacity-40'}`}
    >
      <svg
        ref={svg}
        viewBox="0 0 240 156"
        className="h-24 w-auto shrink-0"
        role="img"
        aria-label="Controller state"
      >
        {trigger(46, BUTTONS.leftTrigger)}
        {trigger(164, BUTTONS.rightTrigger)}
        <rect
          x={36}
          y={22}
          width={52}
          height={9}
          rx={4.5}
          {...key(BUTTONS.leftBumper)}
        />
        <rect
          x={152}
          y={22}
          width={52}
          height={9}
          rx={4.5}
          {...key(BUTTONS.rightBumper)}
        />
        <path
          d="M62 34 H178 C206 34 224 54 231 90 C237 124 228 148 208 146 C194 145 184 136 170 130 H70 C56 136 46 145 32 146 C12 148 3 124 9 90 C16 54 34 34 62 34 Z"
          fill="var(--muted)"
          {...line}
        />
        {pad(64, 68, 0)}
        {pad(148, 102, 2)}
        {arm(BUTTONS.up, 89.5, 87.5, 9, 10)}
        {arm(BUTTONS.down, 89.5, 106.5, 9, 10)}
        {arm(BUTTONS.left, 79.5, 97.5, 10, 9)}
        {arm(BUTTONS.right, 98.5, 97.5, 10, 9)}
        <rect x={89.5} y={97.5} width={9} height={9} fill={OFF} />
        <rect
          x={101}
          y={50}
          width={11}
          height={6}
          rx={3}
          {...key(BUTTONS.select)}
        />
        <rect
          x={128}
          y={50}
          width={11}
          height={6}
          rx={3}
          {...key(BUTTONS.start)}
        />
        <circle cx={120} cy={68} r={5} {...key(BUTTONS.home)} />
        {face(178, 54, BUTTONS.y, 'Y')}
        {face(164, 68, BUTTONS.x, 'X')}
        {face(192, 68, BUTTONS.b, 'B')}
        {face(178, 82, BUTTONS.a, 'A')}
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
              <dt className="text-foreground">{control}</dt>
              <dd>{action}</dd>
            </div>
          ))}
        </dl>
      </div>
    </div>
  );
});
