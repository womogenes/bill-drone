import { spawn, type ChildProcess } from 'node:child_process';
import { resolve } from 'node:path';

// Every FPV frame the dashboard delivers while the laptop service records a flight log (armed, or disarmed after request O) is saved beside that log: sw/recordings/<log stem>_video/<capture_us>.jpg, byte for byte as reassembled, and frames.csv (capture_us, frame, received_unix_s, file) as sw/vio/record_camera.py writes them, so sw/vio/recorded.py reads either. Nothing here asks the camera for anything: with no FPV coming, nothing is recorded.
// Where the bridge records (src/fw/controller/bridge.py RECORDINGS), beside this dashboard; FPV_RECORDINGS and FPV_RECORDER_STATUS stand in for them in an offline replay.
const RECORDINGS = resolve(
  process.cwd(),
  process.env.FPV_RECORDINGS ?? '../recordings'
);
const STATUS_URL =
  process.env.FPV_RECORDER_STATUS ?? 'http://127.0.0.1:8765/status';
// Frames handed to the writer and not yet in its pipe, at most: about 5 s of VGA frames. Past it frames are dropped and counted, so a slow disk never holds the stream back or grows this server's memory.
const QUEUE_BYTES = 2 * 1024 * 1024;
// FPV gone for this long ends the recording and the polling, and the bridge unheard for this long counts as not recording.
const QUIET_MS = 3000;
const BRIDGE_MS = 5000;

// The writer, its own process so file writes and fsyncs never wait on this event loop nor it on them. Each frame comes as "F capture_us frame received_unix_s length\n" and the JPEG, drops as "D count\n". Rows end in CRLF as Python's csv module ends them in record_camera.py. Each frame's file is written before its row, and each row in one append, so frames.csv keeps whole lines naming whole files however the writer ends; once a second the files, frames.csv and summary.json (written and dropped, kept across reopenings of the folder) are synced and the counts printed.
const WRITER = `import json, os, sys, time
folder = sys.argv[1]
os.makedirs(folder, exist_ok=True)
summary = os.path.join(folder, "summary.json")
try:
    with open(summary) as file:
        base = json.load(file)
except (OSError, ValueError):
    base = {}
dropped_before, dropped = base.get("dropped", 0), 0
rows = os.path.join(folder, "frames.csv")
index = os.open(rows, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
if os.fstat(index).st_size == 0:
    os.write(index, b"capture_us,frame,received_unix_s,file\\r\\n")
# Counted from the rows, which a writer that was killed may have added after its last summary.
with open(rows, "rb") as file:
    written = file.read().count(b"\\n") - 1
directory = os.open(folder, os.O_RDONLY)
unsynced = []
def sync():
    for fd in unsynced:
        os.fsync(fd)
        os.close(fd)
    unsynced.clear()
    os.fsync(index)
    with open(summary + ".tmp", "w") as file:
        json.dump({"written": written, "dropped": dropped_before + dropped}, file)
        file.flush()
        os.fsync(file.fileno())
    os.replace(summary + ".tmp", summary)
    os.fsync(directory)
    print(written, dropped_before, flush=True)
sync()
synced = time.monotonic()
stdin = sys.stdin.buffer
while True:
    line = stdin.readline().split()
    if not line:
        break
    if line[0] == b"D":
        dropped = int(line[1])
    else:
        capture, frame, received, length = (part.decode() for part in line[1:])
        data = stdin.read(int(length))
        if len(data) < int(length):
            break
        name = f"{capture}.jpg"
        fd = os.open(os.path.join(folder, name), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view):]
        unsynced.append(fd)
        os.write(index, f"{capture},{frame},{received},{name}\\r\\n".encode())
        written += 1
    if time.monotonic() - synced >= 1:
        sync()
        synced = time.monotonic()
sync()`;

type Writer = {
  process: ChildProcess;
  directory: string;
  // From the writer's last report: frames in the folder and those dropped before this writer opened it; then those this writer dropped, and the count last sent to it.
  written: number;
  droppedBefore: number;
  dropped: number;
  droppedSent: number;
};
type Recorder = {
  // The writer of the recording running, and of the last one, for the status after it.
  writer?: Writer;
  last?: Writer;
  // The bridge's open flight log (its .bin's name), and when the bridge last answered.
  log?: string;
  answered: number;
  asking: boolean;
  lastFrame: number;
  poll?: ReturnType<typeof setInterval>;
  error?: string;
};
const store = globalThis as { fpvRecorder?: Recorder };
const recorder = (store.fpvRecorder ??= {
  answered: 0,
  asking: false,
  lastFrame: 0,
});

function open(directory: string) {
  const child = spawn('python3', ['-c', WRITER, directory], {
    stdio: ['pipe', 'pipe', 'pipe'],
  });
  const writer: Writer = {
    process: child,
    directory,
    written: 0,
    droppedBefore: 0,
    dropped: 0,
    droppedSent: 0,
  };
  let errors = '';
  child.stdin!.on('error', () => {});
  child.stderr!.on('data', (data: Buffer) => (errors += data));
  child.stdout!.on('data', (data: Buffer) => {
    const [written, droppedBefore] = data
      .toString()
      .trim()
      .split('\n')
      .pop()!
      .split(' ')
      .map(Number);
    writer.written = written;
    writer.droppedBefore = droppedBefore;
  });
  child.on('exit', (code) => {
    if (code) {
      recorder.error =
        errors.trim().split('\n').pop() || `Writer exited ${code}`;
      console.error(`FPV recorder ${directory}: ${errors}`);
    }
    if (recorder.writer === writer) recorder.writer = undefined;
  });
  recorder.error = undefined;
  recorder.writer = recorder.last = writer;
}

function close() {
  const writer = recorder.writer;
  if (!writer) return;
  recorder.writer = undefined;
  writer.process.stdin!.end(`D ${writer.dropped}\n`);
}

// Once a second while FPV comes: the bridge's open flight log decides whether and where to record.
async function check() {
  if (Date.now() - recorder.lastFrame > QUIET_MS) {
    close();
    clearInterval(recorder.poll);
    recorder.poll = undefined;
    return;
  }
  if (!recorder.asking) {
    recorder.asking = true;
    try {
      const status = await (
        await fetch(STATUS_URL, { signal: AbortSignal.timeout(900) })
      ).json();
      const file = status.reading?.recorder;
      recorder.log = file?.writing && file.file ? file.file : undefined;
      recorder.answered = Date.now();
    } catch {
    } finally {
      recorder.asking = false;
    }
  }
  if (Date.now() - recorder.answered > BRIDGE_MS) recorder.log = undefined;
  const directory =
    recorder.log &&
    resolve(RECORDINGS, `${recorder.log.replace(/\.bin$/, '')}_video`);
  if (recorder.writer && recorder.writer.directory !== directory) close();
  if (directory && !recorder.writer) open(directory);
}

// Each reassembled FPV frame, on the stream's own path: at most one pipe write, which never waits.
export function recordFpvFrame(
  jpeg: Uint8Array,
  captureUs: number,
  frame: number
) {
  const received = Date.now() / 1000;
  recorder.lastFrame = Date.now();
  if (!recorder.poll) {
    recorder.poll = setInterval(check, 1000);
    void check();
  }
  const writer = recorder.writer;
  if (!writer) return;
  const stdin = writer.process.stdin!;
  if (stdin.writableLength > QUEUE_BYTES) {
    writer.dropped++;
    return;
  }
  if (writer.dropped !== writer.droppedSent) {
    stdin.write(`D ${writer.dropped}\n`);
    writer.droppedSent = writer.dropped;
  }
  stdin.write(
    `F ${captureUs} ${frame} ${received.toFixed(6)} ${jpeg.length}\n`
  );
  stdin.write(jpeg);
}

// For the camera's status: whether frames are being recorded, where, how many are in the folder and how many were dropped for a slow disk.
export function fpvRecording() {
  const writer = recorder.writer ?? recorder.last;
  return {
    active: Boolean(recorder.writer),
    directory: writer?.directory,
    written: writer?.written ?? 0,
    dropped: writer ? writer.droppedBefore + writer.dropped : 0,
    error: recorder.error,
  };
}
