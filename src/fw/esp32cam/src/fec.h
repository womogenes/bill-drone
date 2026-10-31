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

inline uint8_t coefficient(int data, int parity, int shard) {
    if (!exps[0])
        init();
    return exps[255 - logs[(data + parity) ^ shard]];
}

// Adds coefficient (from coefficient()) times in (n bytes) into out.
inline void accumulate(uint8_t *out, const uint8_t *in, size_t n, uint8_t coefficient) {
    uint8_t row[256];
    row[0] = 0;
    for (int v = 1; v < 256; ++v)
        row[v] = exps[logs[coefficient] + logs[v]];
    for (size_t b = 0; b < n; ++b)
        out[b] ^= row[in[b]];
}
} // namespace fec
