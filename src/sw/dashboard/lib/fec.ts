// Decodes the camera's erasure-coded FPV frames (fw/esp32cam/src/fec.h): a JPEG of length bytes split evenly into data shards, then parity shards from a Cauchy matrix over GF(256) (polynomial 0x11d), parity shard j's coefficient for data shard i being 1 / ((data + j) xor i). Any data of the shards rebuild it.

const exps = new Uint8Array(510);
const logs = new Uint8Array(256);
for (let i = 0, x = 1; i < 255; i++) {
  exps[i] = exps[i + 255] = x;
  logs[x] = i;
  x <<= 1;
  if (x & 0x100) x ^= 0x11d;
}

const mul = (a: number, b: number) => (a && b ? exps[logs[a] + logs[b]] : 0);
const inverse = (a: number) => exps[255 - logs[a]];

// The JPEG from at least data of its shards (index to payload; the last data shard may be short), or null when there are too few.
export function decode(
  shards: Map<number, Buffer>,
  data: number,
  length: number
): Buffer | null {
  const size = Math.ceil(length / data);
  const out = Buffer.alloc(size * data);
  const missing: number[] = [];
  for (let i = 0; i < data; i++) {
    const shard = shards.get(i);
    if (shard) shard.copy(out, i * size);
    else missing.push(i);
  }
  if (missing.length) {
    // Rows of the code for the shards used: the data shards present, then enough parity shards.
    const used = [
      ...[...shards.keys()].filter((index) => index < data),
      ...[...shards.keys()].filter((index) => index >= data).slice(0, missing.length),
    ];
    if (used.length < data) return null;
    const rows = used.map((index) =>
      Array.from({ length: data }, (_, i) =>
        index < data ? Number(index === i) : inverse(index ^ i)
      )
    );
    // Gauss-Jordan elimination over GF(256): rows becomes the identity and solution its inverse.
    const solution = rows.map((_, r) =>
      Array.from({ length: data }, (_, c) => Number(r === c))
    );
    for (let column = 0; column < data; column++) {
      const pivot = rows.findIndex((row, r) => r >= column && row[column]);
      if (pivot < 0) return null;
      [rows[column], rows[pivot]] = [rows[pivot], rows[column]];
      [solution[column], solution[pivot]] = [solution[pivot], solution[column]];
      const scale = inverse(rows[column][column]);
      rows[column] = rows[column].map((v) => mul(v, scale));
      solution[column] = solution[column].map((v) => mul(v, scale));
      for (let r = 0; r < data; r++) {
        const factor = rows[r][column];
        if (r === column || !factor) continue;
        rows[r] = rows[r].map((v, c) => v ^ mul(factor, rows[column][c]));
        solution[r] = solution[r].map((v, c) => v ^ mul(factor, solution[column][c]));
      }
    }
    const inputs = used.map((index) => {
      const shard = Buffer.alloc(size);
      shards.get(index)!.copy(shard);
      return shard;
    });
    for (const i of missing) {
      const target = out.subarray(i * size, (i + 1) * size);
      solution[i].forEach((factor, r) => {
        if (!factor) return;
        const input = inputs[r];
        for (let b = 0; b < size; b++) target[b] ^= mul(factor, input[b]);
      });
    }
  }
  return out.subarray(0, length);
}
