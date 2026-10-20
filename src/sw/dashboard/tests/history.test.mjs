import assert from 'node:assert/strict';
import { test } from 'node:test';
import { createHistory } from '../lib/history.ts';

test('history expires old samples and returns chronological ranges after wraparound', () => {
  const history = createHistory(100, 4);
  for (let time = 0; time <= 250; time += 25) history.append({ time });
  assert.deepEqual(
    history.read(0, 1000).map(({ time }) => time),
    [175, 200, 225, 250]
  );
  assert.deepEqual(
    history.read(200, 225).map(({ time }) => time),
    [200, 225]
  );
  history.append({ time: 1000 });
  assert.deepEqual(history.read(0, 1000), [{ time: 1000 }]);
});

test('one hour of 100 Hz samples retains only the last minute, capped at 6000', () => {
  const history = createHistory();
  for (let time = 0; time < 3_600_000; time += 10) history.append({ time });
  const samples = history.read(0, Infinity);
  assert.equal(samples.length, 6000);
  assert.equal(samples[0].time, 3_540_000);
  assert.equal(samples.at(-1).time, 3_599_990);
  assert.equal(history.read(3_594_990, 3_599_990).length, 501);
});
