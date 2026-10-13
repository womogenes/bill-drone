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

import { ControlPanel } from '@/components/control-panel';
import { Button } from '@/components/ui/button';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { ImuPanel } from '@/components/imu-panel';
import type {
  ImuSample,
  Reading,
  Side,
  ControlSettings,
} from '@/lib/telemetry';

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

function reuseUnchanged<T extends object>(
  previous: T | undefined,
  next: T | undefined
): T | undefined {
  if (
    previous &&
    next &&
    Object.keys(next).length === Object.keys(previous).length &&
    (Object.keys(next) as (keyof T)[]).every(
      (key) => next[key] === previous[key]
    )
  ) {
    return previous;
  }
  return next;
}

export default function Page() {
  const [status, setStatus] = useState<keyof typeof statuses>('offline');
  const [address, setAddress] = useState('');
  const [reading, setReading] = useState<Reading | null>(null);
  const [samples, setSamples] = useState<ImuSample[]>([]);
  const latestReading = useRef<Reading | null>(null);
  const history = useRef<ImuSample[]>([]);
  const [stopEpoch, setStopEpoch] = useState(0);
  const heartbeat = useRef<ReturnType<typeof setInterval> | undefined>(
    undefined
  );
  const socket = useRef<WebSocket | null>(null);
  const watchdog = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

  const lookup = useRef<AbortController | null>(null);
  const pendingControl = useRef<Partial<ControlSettings>>({});
  const controlTimer = useRef<ReturnType<typeof setTimeout> | undefined>(
    undefined
  );
  const lastControlSent = useRef(0);

  const cancelControl = useCallback(() => {
    clearTimeout(controlTimer.current);
    controlTimer.current = undefined;
    pendingControl.current = {};
  }, []);

  const connect = useCallback(async () => {
    cancelControl();
    lookup.current?.abort();
    const controller = new AbortController();
    lookup.current = controller;
    clearTimeout(watchdog.current);
    clearInterval(heartbeat.current);
    socket.current?.close();
    socket.current = null;
    setStatus('connecting');
    setReading(null);
    latestReading.current = null;

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
      function offline() {
        if (socket.current !== ws) return;
        cancelControl();
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
        const data: Reading = JSON.parse(event.data);
        data.control = reuseUnchanged(
          latestReading.current?.control,
          data.control
        );
        data.control_status = reuseUnchanged(
          latestReading.current?.control_status,
          data.control_status
        );
        const time = Date.now();
        latestReading.current = { ...data, time };
        if (data.imu == null) return;
        history.current.push({ ...data.imu, time, session });
        while (
          history.current.length &&
          (history.current[0].time < time - 60000 ||
            history.current.length > 3000)
        ) {
          history.current.shift();
        }
      };
      ws.onerror = offline;
      ws.onclose = offline;
    } catch (error) {
      if (controller.signal.aborted) return;
      console.error(error);
      setStatus('offline');
    }
  }, [cancelControl]);

  useEffect(() => {
    const start = setTimeout(() => void connect(), 0);
    return () => {
      clearTimeout(start);
      cancelControl();
      lookup.current?.abort();
      clearTimeout(watchdog.current);
      clearInterval(heartbeat.current);
      socket.current?.close();
      socket.current = null;
    };
  }, [connect, cancelControl]);

  useEffect(() => {
    let frame = 0;
    let displayed: Reading | null = null;
    function update() {
      const current = latestReading.current;
      if (current !== displayed) {
        displayed = current;
        const points = [...history.current];
        startTransition(() => {
          setReading(current);
          setSamples(points);
        });
      }
      frame = requestAnimationFrame(update);
    }
    frame = requestAnimationFrame(update);
    return () => cancelAnimationFrame(frame);
  }, []);

  const calibrateImu = useCallback(() => {
    socket.current?.send(JSON.stringify({ command: 'calibrate_imu' }));
  }, []);

  const sendControl = useCallback((settings: Partial<ControlSettings>) => {
    pendingControl.current = { ...pendingControl.current, ...settings };
    if (controlTimer.current !== undefined) return;

    function flush() {
      controlTimer.current = undefined;
      const ws = socket.current;
      if (ws?.readyState !== WebSocket.OPEN) {
        pendingControl.current = {};
        return;
      }
      if (ws.bufferedAmount > 0) {
        controlTimer.current = setTimeout(flush, 20);
        return;
      }
      ws.send(
        JSON.stringify({ command: 'control', settings: pendingControl.current })
      );
      pendingControl.current = {};
      lastControlSent.current = performance.now();
    }

    const delay = 20 - (performance.now() - lastControlSent.current);
    if (delay <= 0) flush();
    else controlTimer.current = setTimeout(flush, delay);
  }, []);
  const stopMotors = useCallback(() => {
    cancelControl();
    setStopEpoch((previous) => previous + 1);
    if (socket.current?.readyState === WebSocket.OPEN) {
      socket.current.send(JSON.stringify({ command: 'stop' }));
    }
  }, [cancelControl]);

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
      stopMotors();
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
  }, [stopMotors]);

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
        <div className="min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
          <Suspense>
            <ImuPanel
              reading={reading?.imu}
              samples={samples}
              time={samples.at(-1)?.time ?? 0}
              onCalibrate={calibrateImu}
              connected={status === 'connected'}
            />
          </Suspense>
          {/* <LoadCellPanel reading={reading} connected={status === 'connected'} /> */}
          <ADCPanel
            raw={reading?.adc_raw}
            voltage={reading?.adc_voltage}
            connected={status === 'connected'}
          />
        </div>
        <div className="min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
          {(['left', 'right'] as const).flatMap((side) =>
            (['servo', 'throttle'] as const).map((kind) => (
              <ActuatorPanel
                key={`${side}-${kind}-${status}-${stopEpoch}-${reading?.control?.enabled}`}
                side={side}
                kind={kind}
                locked={reading?.control?.enabled}
                value={
                  kind === 'servo'
                    ? reading?.actuators &&
                      Math.round(reading.actuators[side].servo_angle * 10) / 10
                    : reading?.actuators?.[side].throttle
                }
                connected={status === 'connected'}
                onChange={sendActuator}
              />
            ))
          )}
        </div>
        <div className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain">
          {reading?.control && (
            <ControlPanel
              key={`${status}-${stopEpoch}-${reading.control.enabled}`}
              settings={reading.control}
              status={reading.control_status}
              connected={
                status === 'connected' &&
                reading.imu != null &&
                !reading.imu.calibrating
              }
              onChange={sendControl}
              onStop={stopMotors}
            />
          )}
        </div>
      </main>
    </>
  );
}
