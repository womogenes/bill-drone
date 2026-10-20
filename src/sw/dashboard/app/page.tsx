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
import Link from 'next/link';

import { ControlPanel } from '@/components/control-panel';
import { Button } from '@/components/ui/button';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { ImuPanel } from '@/components/imu-panel';
import { encodeCommand, decodeReading } from '@/lib/protocol';
import { EspConnection } from '@/lib/esp-connection';
import type {
  ImuSample,
  Reading,
  Side,
  ControlSettings,
  RecorderStatus,
} from '@/lib/telemetry';

const routes = [
  { value: 'router', label: 'Router' },
  { value: 'usb', label: 'USB controller' },
];

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
  const [recording, setRecording] = useState<RecorderStatus>({ ready: false });
  const [reading, setReading] = useState<Reading | null>(null);
  const [samples, setSamples] = useState<ImuSample[]>([]);
  const latestReading = useRef<Reading | null>(null);
  const history = useRef<ImuSample[]>([]);
  const displayFrame = useRef<number | undefined>(undefined);
  const publishReading = useCallback(() => {
    if (displayFrame.current !== undefined) return;
    displayFrame.current = requestAnimationFrame(() => {
      displayFrame.current = undefined;
      const current = latestReading.current;
      const points = [...history.current];
      startTransition(() => {
        setReading(current);
        setSamples(points);
      });
    });
  }, []);
  const [stopEpoch, setStopEpoch] = useState(0);
  const [fps, setFps] = useState<number | null>(null);
  const packetCount = useRef(0);
  const socket = useRef<EspConnection | null>(null);

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
    if (displayFrame.current !== undefined)
      cancelAnimationFrame(displayFrame.current);
    displayFrame.current = undefined;
    lookup.current?.abort();
    const controller = new AbortController();
    lookup.current = controller;
    socket.current?.close();
    socket.current = null;
    setStatus('connecting');
    setReading(null);
    latestReading.current = null;

    try {
      const direct = new URLSearchParams(window.location.search).get('controller') === 'usb';
      let address = '127.0.0.1:8765';
      if (!direct) {
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
        address = (await response.text()).trim();
      }
      if (controller.signal.aborted) return;
      setAddress(address);
      let reason = '';
      const session = Date.now();
      function offline() {
        if (socket.current !== ws) return;
        cancelControl();
        socket.current = null;
        setStatus('offline');
        ws.close();
      }

      const ws = new EspConnection(
        address,
        () => {
          if (socket.current === ws) setStatus('connected');
        },
        (packet) => {
          if (socket.current !== ws) return;
          if (new Uint8Array(packet)[0] === 82) {
            reason = new TextDecoder().decode(packet.slice(1));
            return;
          }
          const data = decodeReading(packet, reason);
          packetCount.current++;
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
          publishReading();
          if (data.imu == null) return;
          history.current.push({ ...data.imu, time, session });
          // Trim in batches: shifting re-indexes every sample, which makes React's dev profiler deep-diff the whole array each frame.
          if (
            history.current[0].time < time - 120000 ||
            history.current.length > 6000
          ) {
            history.current = history.current
              .filter((sample) => sample.time >= time - 60000)
              .slice(-3000);
          }
        },
        offline
      );
      socket.current = ws;
    } catch (error) {
      if (controller.signal.aborted) return;
      console.error(error);
      setStatus('offline');
    }
  }, [cancelControl, publishReading]);

  useEffect(() => {
    const start = setTimeout(() => void connect(), 0);
    return () => {
      clearTimeout(start);
      if (displayFrame.current !== undefined)
        cancelAnimationFrame(displayFrame.current);
      displayFrame.current = undefined;
      cancelControl();
      lookup.current?.abort();
      socket.current?.close();
      socket.current = null;
    };
  }, [connect, cancelControl]);

  useEffect(() => {
    if (!address || status !== 'connected') return;
    let cancelled = false;
    async function update(start = false) {
      try {
        const response = await fetch(
          '/api/recording',
          start
            ? {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ address }),
              }
            : { cache: 'no-store' }
        );
        if (!response.ok) throw new Error('Laptop recorder unavailable');
        const result: RecorderStatus = await response.json();
        if (!cancelled) setRecording(result);
      } catch (error) {
        if (!cancelled) setRecording({ ready: false, error: String(error) });
      }
    }
    void update(true);
    const timer = setInterval(() => void update(), 1000);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [address, status]);

  useEffect(() => {
    if (status !== 'connected') return;
    const marks = [{ time: performance.now(), count: packetCount.current }];
    const timer = setInterval(() => {
      const time = performance.now();
      marks.push({ time, count: packetCount.current });
      if (marks.length > 5) marks.shift();
      setFps(
        ((marks.at(-1)!.count - marks[0].count) * 1000) / (time - marks[0].time)
      );
    }, 500);
    return () => {
      clearInterval(timer);
      setFps(null);
    };
  }, [status]);

  useEffect(() => {
    if (status !== 'offline') return;
    const timer = setTimeout(() => void connect(), 1000);
    return () => clearTimeout(timer);
  }, [status, connect]);

  const calibrateImu = useCallback(() => {
    socket.current?.send(encodeCommand({ command: 'calibrate_imu' }));
  }, []);

  const sendControl = useCallback(
    (settings: Partial<ControlSettings>, immediate = false) => {
      pendingControl.current = { ...pendingControl.current, ...settings };
      if (controlTimer.current !== undefined) {
        if (!immediate) return;
        clearTimeout(controlTimer.current);
        controlTimer.current = undefined;
      }

      function flush() {
        controlTimer.current = undefined;
        const ws = socket.current;
        if (!ws?.connected) {
          pendingControl.current = {};
          return;
        }
        ws.send(
          encodeCommand({
            command: 'control',
            settings: pendingControl.current,
          })
        );
        pendingControl.current = {};
        lastControlSent.current = performance.now();
      }

      const delay = immediate
        ? 0
        : 10 - (performance.now() - lastControlSent.current);
      if (delay <= 0) flush();
      else controlTimer.current = setTimeout(flush, delay);
    },
    []
  );

  // Local tools tune gains and model parameters through this page's connection.
  useEffect(() => {
    if (status !== 'connected') return;
    const timer = setInterval(async () => {
      const control = latestReading.current?.control;
      if (!control) return;
      const response = await fetch('/api/settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ control }),
      });
      const { settings } = await response.json();
      if (Object.keys(settings).length) sendControl(settings, true);
    }, 300);
    return () => clearInterval(timer);
  }, [status, sendControl]);

  const stopMotors = useCallback(() => {
    cancelControl();
    setStopEpoch((previous) => previous + 1);
    if (socket.current?.connected) {
      socket.current.send(encodeCommand({ command: 'stop' }));
    }
  }, [cancelControl]);

  const sendActuator = useCallback(
    (command: 'servo' | 'throttle', side: Side, value: number) => {
      if (socket.current?.connected) {
        socket.current.send(encodeCommand({ command, side, value }));
      }
    },
    []
  );

  const zeroServo = useCallback((side: Side) => {
    if (socket.current?.connected) {
      socket.current.send(encodeCommand({ command: 'zero_servo', side }));
    }
  }, []);

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
          <Link href="/camera" className="text-sm underline">
            Camera
          </Link>
          <Select
            value={address === '127.0.0.1:8765' ? 'usb' : 'router'}
            items={routes}
            disabled={status === 'connecting' || reading?.control?.enabled}
            onValueChange={(value) => {
              const url = new URL(window.location.href);
              if (value === 'usb') url.searchParams.set('controller', 'usb');
              else url.searchParams.delete('controller');
              window.history.replaceState(null, '', url);
              void connect();
            }}
          >
            <SelectTrigger aria-label="Connection route">
              <SelectValue />
            </SelectTrigger>
            <SelectContent align="start" alignItemWithTrigger={false}>
              <SelectGroup>
                {routes.map((route) => (
                  <SelectItem key={route.value} value={route.value}>
                    {route.label}
                  </SelectItem>
                ))}
              </SelectGroup>
            </SelectContent>
          </Select>
          {address && (
            <span className="text-sm text-muted-foreground tabular-nums">
              {address}
            </span>
          )}
          {fps !== null && (
            <span className="text-sm text-muted-foreground tabular-nums">
              {Math.round(fps)} FPS
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
                onZero={zeroServo}
                zeroDisabled={
                  !!(
                    reading?.actuators?.left.throttle ||
                    reading?.actuators?.right.throttle
                  )
                }
              />
            ))
          )}
        </div>
        <div className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain">
          <p
            className={`px-4 pt-4 ${recording.error || (reading?.logging?.dropped ?? 0) > 0 ? 'text-red-700' : 'text-muted-foreground'}`}
          >
            {recording.error ??
              (reading?.logging?.active
                ? `Recording · ${reading.logging.samples} samples · ${reading.logging.dropped} dropped`
                : recording.ready && reading?.logging?.connected
                  ? recording.writing
                    ? 'Saving recording'
                    : recording.file
                      ? 'Recording saved'
                      : 'Recorder ready'
                  : 'Recorder connecting')}
          </p>
          {reading?.control && (
            <ControlPanel
              key={`${status}-${stopEpoch}-${reading.control.enabled}`}
              settings={reading.control}
              recorderReady={
                recording.ready &&
                recording.address === address &&
                !!reading.logging?.ready
              }
              recorderError={recording.error}
              status={reading.control_status}
              connected={status === 'connected'}
              imu={
                !reading.imu
                  ? 'offline'
                  : reading.imu.calibrating
                    ? 'calibrating'
                    : reading.imu.calibrated
                      ? 'calibrated'
                      : 'uncalibrated'
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
