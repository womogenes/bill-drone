#pragma once
#include <stdint.h>
#include <string.h>

// Systematic Reed-Solomon erasure code over GF(256) (polynomial 0x11d) for FPV's raw frames (fpv.h): a message goes out as data shards, its bytes split evenly, and parity shards from a Cauchy matrix, parity shard j's coefficient for data shard i being 1 / ((data + j) xor i). Every square block of that matrix is invertible, so any data of the shards rebuild the message; the dashboard decodes it (sw/dashboard/lib/fec.ts).
namespace fec {
uint8_t exps[510], logs[256];

inline void init() {
    unsigned x = 1;
    for (int i = 0; i < 255; ++i) {
        exps[i] = exps[i + 255] = uint8_t(x);
        logs[x] = uint8_t(i);
        x <<= 1;
        if (x & 0x100)
            x ^= 0x11d;
    }
}

inline uint8_t coefficient(int data, int parity, int shard) { return exps[255 - logs[(data + parity) ^ shard]]; }

// Parity shard parity (from 0) of a message of length bytes in data shards of size bytes, the last one zero-padded, into out (size bytes).
inline void parity(const uint8_t *message, size_t length, int data, size_t size, int parity, uint8_t *out) {
    if (!exps[0])
        init();
    memset(out, 0, size);
    uint8_t row[256];
    for (int i = 0; i < data; ++i) {
        const int scale = logs[coefficient(data, parity, i)];
        row[0] = 0;
        for (int v = 1; v < 256; ++v)
            row[v] = exps[scale + logs[v]];
        const size_t start = i * size, n = start >= length ? 0 : length - start < size ? length - start : size;
        const uint8_t *in = message + start;
        for (size_t b = 0; b < n; ++b)
            out[b] ^= row[in[b]];
    }
}
} // namespace fec
