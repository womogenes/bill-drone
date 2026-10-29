// Keep chart history outside React state: only the latest reading needs a render.
export function createHistory<T extends { time: number }>(
  duration = 60_000,
  capacity = 6000
) {
  const points: (T | undefined)[] = new Array(capacity);
  let first = 0;
  let count = 0;

  function discard() {
    points[first] = undefined;
    first = (first + 1) % capacity;
    count--;
  }

  function append(point: T) {
    while (count && points[first]!.time < point.time - duration) discard();
    if (count === capacity) discard();
    points[(first + count) % capacity] = point;
    count++;
  }

  function read(start: number, end: number): T[] {
    const visible: T[] = [];
    for (let index = 0; index < count; index++) {
      const point = points[(first + index) % capacity]!;
      if (point.time > end) break;
      if (point.time >= start) visible.push(point);
    }
    return visible;
  }

  return { append, read };
}
