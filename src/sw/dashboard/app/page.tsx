'use client';

import {
  Suspense,
  startTransition,
  useCallback,
  useEffect,
  useRef,
  useState,
} from 'react';
import { LoaderCircle, RefreshCw, Wifi, WifiOff } from 'lucide-react';

import { Button } from '@/components/ui/button';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { ImuPanel } from '@/components/imu-panel';
import { LoadCellPanel } from '@/components/load-cell-panel';
import type { LoadCellSample, Reading, Side } from '@/lib/telemetry';

const statuses = {
  offline: {
    label: 'Offline',
    icon: WifiOff,
    color: 'text-red-700',
  },
  connecting: {
    label: 'Connecting',
    icon: LoaderCircle,
    color: 'text-yellow-600/70 dark:text-yellow-400/70',
  },
  connected: {
    label: 'Connected',
    icon: Wifi,
    color: 'text-green-700',
  },
};

export default function Page() {
  const [status, setStatus] = useState<keyof typeof statuses>('offline');
  const [address, setAddress] = useState('');
  const [reading, setReading] = useState<Reading | null>(null);
  const [samples, setSamples] = useState<LoadCellSample[]>([]);
  const [stopEpoch, setStopEpoch] = useState(0);
  const heartbeat = useRef<ReturnType<typeof setInterval> | undefined>(
    undefined
  );
  const socket = useRef<WebSocket | null>(null);
  const watchdog = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

  const lookup = useRef<AbortController | null>(null);

  const connect = useCallback(async () => {
    lookup.current?.abort();
    const controller = new AbortController();
    lookup.current = controller;
    clearTimeout(watchdog.current);
    clearInterval(heartbeat.current);
    socket.current?.close();
    socket.current = null;
    setStatus('connecting');
    setReading(null);

    try {
      const response = await fetch(
        'https://kv.wfeng.dev/bill-esp32-drone-addr',
        {
          cache: 'no-store',
          signal: AbortSignal.any([
            controller.signal,
            AbortSignal.timeout(10000),
          ]),
        }
      );
      if (!response.ok)
        throw new Error(`Address lookup failed: ${response.status}`);
      const address = (await response.text()).trim();
      if (controller.signal.aborted) return;
      setAddress(address);
      const ws = new WebSocket(`ws://${address}/ws`);
      socket.current = ws;
      const session = Date.now();
      let lastSequence: number | undefined;
      function offline() {
        if (socket.current !== ws) return;
        clearTimeout(watchdog.current);
        socket.current = null;
        setStatus('offline');
        clearInterval(heartbeat.current);
        ws.close();
      }

      function received() {
        if (socket.current !== ws) return;
        clearTimeout(watchdog.current);
        // Silence means the telemetry link is stale.
        watchdog.current = setTimeout(offline, 3000);
      }

      watchdog.current = setTimeout(offline, 10000);

      ws.onopen = () => {
        if (socket.current !== ws) return;
        setStatus('connected');
        heartbeat.current = setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: 'heartbeat' }));
          }
        }, 200);
        received();
      };
      ws.onmessage = (event) => {
        if (socket.current !== ws) return;
        received();
        const data = JSON.parse(event.data);
        const time = Date.now();
        startTransition(() => setReading({ ...data, time }));
        if (typeof data.force !== 'number' || !Number.isFinite(data.force))
          return;
        if (data.force_age_ms === null || data.force_age_ms > 1000) return;
        if (
          data.force_sequence !== undefined &&
          data.force_sequence === lastSequence
        )
          return;
        lastSequence = data.force_sequence;
        startTransition(() =>
          setSamples((previous) =>
            [
              ...previous.filter((point) => point.time >= time - 300000),
              { time, force: data.force, session },
            ].slice(-15000)
          )
        );
      };
      ws.onerror = offline;
      ws.onclose = offline;
    } catch (error) {
      if (controller.signal.aborted) return;
      console.error(error);
      setStatus('offline');
    }
  }, []);

  useEffect(() => {
    const start = setTimeout(() => void connect(), 0);
    return () => {
      clearTimeout(start);
      lookup.current?.abort();
      clearTimeout(watchdog.current);
      clearInterval(heartbeat.current);
      socket.current?.close();
      socket.current = null;
    };
  }, [connect]);

  const sendActuator = useCallback(
    (command: 'servo' | 'throttle', side: Side, value: number) => {
      if (socket.current?.readyState === WebSocket.OPEN) {
        socket.current.send(JSON.stringify({ command, side, value }));
      }
    },
    []
  );

  useEffect(() => {
    function stop(event: KeyboardEvent) {
      if (event.code !== 'Space') return;
      event.preventDefault();
      event.stopPropagation();
      if (event.repeat) return;
      setStopEpoch((previous) => previous + 1);
      if (socket.current?.readyState === WebSocket.OPEN) {
        socket.current.send(JSON.stringify({ command: 'stop' }));
      }
    }
    function close() {
      socket.current?.close();
    }
    window.addEventListener('keydown', stop, true);
    window.addEventListener('pagehide', close);
    return () => {
      window.removeEventListener('keydown', stop, true);
      window.removeEventListener('pagehide', close);
    };
  }, []);

  const { label, icon: StatusIcon, color } = statuses[status];

  return (
    <>
      <header className="flex h-16 items-center justify-between border-b px-6">
        <h1 className="text-lg font-semibold">Drone dashboard</h1>
        <div className="flex items-center gap-3">
          {address && (
            <span className="text-sm text-muted-foreground tabular-nums">
              {address}
            </span>
          )}
          <span
            role="status"
            className={`flex items-center gap-2 text-sm ${color}`}
          >
            <StatusIcon
              aria-hidden="true"
              className={`size-4 ${status === 'connecting' ? 'animate-spin' : ''}`}
            />
            {label}
          </span>
          <Button
            variant="ghost"
            size="icon"
            aria-label="Refresh connection"
            title="Refresh connection"
            disabled={status === 'connecting'}
            onClick={connect}
          >
            <RefreshCw aria-hidden="true" />
          </Button>
        </div>
      </header>
      <main className="grid min-h-[calc(100dvh-4rem)] divide-y divide-border lg:h-[calc(100dvh-4rem)] lg:grid-cols-[repeat(3,minmax(0,28rem))] lg:divide-x lg:divide-y-0 lg:overflow-hidden">
        <div className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain">
          <Suspense>
            <LoadCellPanel
              samples={samples}
              reading={reading}
              connected={status === 'connected'}
              onClear={() => setSamples([])}
            />
          </Suspense>
          <ImuPanel reading={reading?.imu} connected={status === 'connected'} />
          <ADCPanel raw={reading?.adc_raw} connected={status === 'connected'} />
        </div>
        {(['left', 'right'] as const).map((side) => (
          <div
            key={side}
            className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain"
          >
            <ActuatorPanel
              key={`${side}-servo-${status}`}
              side={side}
              kind="servo"
              value={reading?.actuators?.[side].servo_angle}
              connected={status === 'connected'}
              onChange={sendActuator}
            />
            <ActuatorPanel
              key={`${side}-esc-${status}-${stopEpoch}`}
              side={side}
              kind="throttle"
              value={reading?.actuators?.[side].throttle}
              connected={status === 'connected'}
              onChange={sendActuator}
            />
          </div>
        ))}
      </main>
    </>
  );
}
