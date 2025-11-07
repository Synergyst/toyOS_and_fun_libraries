#pragma once
#include <stdint.h>
#include <stddef.h>

namespace shsha256 {

struct Ctx {
  uint32_t state[8];
  uint64_t bitlen;
  uint8_t data[64];
  uint32_t datalen;
};

inline uint32_t rotr(uint32_t x, uint32_t n) {
  return (x >> n) | (x << (32 - n));
}
inline uint32_t Ch(uint32_t x, uint32_t y, uint32_t z) {
  return (x & y) ^ (~x & z);
}
inline uint32_t Maj(uint32_t x, uint32_t y, uint32_t z) {
  return (x & y) ^ (x & z) ^ (y & z);
}
inline uint32_t EP0(uint32_t x) {
  return rotr(x, 2) ^ rotr(x, 13) ^ rotr(x, 22);
}
inline uint32_t EP1(uint32_t x) {
  return rotr(x, 6) ^ rotr(x, 11) ^ rotr(x, 25);
}
inline uint32_t SIG0(uint32_t x) {
  return rotr(x, 7) ^ rotr(x, 18) ^ (x >> 3);
}
inline uint32_t SIG1(uint32_t x) {
  return rotr(x, 17) ^ rotr(x, 19) ^ (x >> 10);
}

static const uint32_t K[64] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

inline void transform(Ctx* ctx, const uint8_t data[64]) {
  uint32_t m[64];
  for (int i = 0; i < 16; ++i) {
    int j = i * 4;
    m[i] = (uint32_t)data[j + 0] << 24 | (uint32_t)data[j + 1] << 16 | (uint32_t)data[j + 2] << 8 | (uint32_t)data[j + 3];
  }
  for (int i = 16; i < 64; ++i)
    m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];

  uint32_t a = ctx->state[0], b = ctx->state[1], c = ctx->state[2], d = ctx->state[3];
  uint32_t e = ctx->state[4], f = ctx->state[5], g = ctx->state[6], h = ctx->state[7];

  for (int i = 0; i < 64; ++i) {
    uint32_t t1 = h + EP1(e) + Ch(e, f, g) + K[i] + m[i];
    uint32_t t2 = EP0(a) + Maj(a, b, c);
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }

  ctx->state[0] += a;
  ctx->state[1] += b;
  ctx->state[2] += c;
  ctx->state[3] += d;
  ctx->state[4] += e;
  ctx->state[5] += f;
  ctx->state[6] += g;
  ctx->state[7] += h;
}

inline void init(Ctx* ctx) {
  ctx->datalen = 0;
  ctx->bitlen = 0;
  ctx->state[0] = 0x6a09e667;
  ctx->state[1] = 0xbb67ae85;
  ctx->state[2] = 0x3c6ef372;
  ctx->state[3] = 0xa54ff53a;
  ctx->state[4] = 0x510e527f;
  ctx->state[5] = 0x9b05688c;
  ctx->state[6] = 0x1f83d9ab;
  ctx->state[7] = 0x5be0cd19;
}

inline void update(Ctx* ctx, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    ctx->data[ctx->datalen++] = data[i];
    if (ctx->datalen == 64) {
      transform(ctx, ctx->data);
      ctx->bitlen += 512;
      ctx->datalen = 0;
    }
  }
}

inline void final(Ctx* ctx, uint8_t out[32]) {
  uint32_t i = ctx->datalen;
  ctx->data[i++] = 0x80;
  if (i > 56) {
    while (i < 64) ctx->data[i++] = 0;
    transform(ctx, ctx->data);
    i = 0;
  }
  while (i < 56) ctx->data[i++] = 0;
  ctx->bitlen += (uint64_t)ctx->datalen * 8ull;
  uint64_t bl = ctx->bitlen;
  for (int b = 0; b < 8; ++b) ctx->data[63 - b] = (uint8_t)(bl >> (8 * b));
  transform(ctx, ctx->data);
  for (int j = 0; j < 8; ++j) {
    uint32_t s = ctx->state[j];
    out[j * 4 + 0] = (uint8_t)(s >> 24);
    out[j * 4 + 1] = (uint8_t)(s >> 16);
    out[j * 4 + 2] = (uint8_t)(s >> 8);
    out[j * 4 + 3] = (uint8_t)(s);
  }
}

// Optional helper for chunked I/O
template<typename Reader>
inline bool hashStream(Reader rd, uint8_t out[32], size_t chunk = 1024) {
  Ctx ctx;
  init(&ctx);
  uint8_t* buf = new (std::nothrow) uint8_t[chunk];
  if (!buf) return false;
  for (;;) {
    size_t got = rd(buf, chunk);
    if (got == 0) break;
    update(&ctx, buf, got);
  }
  delete[] buf;
  final(&ctx, out);
  return true;
}

}  // namespace shsha256