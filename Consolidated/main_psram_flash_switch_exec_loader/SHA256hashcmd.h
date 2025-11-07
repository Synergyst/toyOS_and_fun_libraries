// =================== SHA-256 (public domain-style, compact) ===================
struct SHA256_CTX {
  uint32_t state[8];
  uint64_t bitlen;
  uint8_t data[64];
  uint32_t datalen;
};

static inline uint32_t SHR(uint32_t x, uint32_t n) {
  return x >> n;
}
static inline uint32_t ROTR(uint32_t x, uint32_t n) {
  return (x >> n) | (x << (32 - n));
}
static inline uint32_t Ch(uint32_t x, uint32_t y, uint32_t z) {
  return (x & y) ^ (~x & z);
}
static inline uint32_t Maj(uint32_t x, uint32_t y, uint32_t z) {
  return (x & y) ^ (x & z) ^ (y & z);
}
static inline uint32_t EP0(uint32_t x) {
  return ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22);
}
static inline uint32_t EP1(uint32_t x) {
  return ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25);
}
static inline uint32_t SIG0(uint32_t x) {
  return ROTR(x, 7) ^ ROTR(x, 18) ^ SHR(x, 3);
}
static inline uint32_t SIG1(uint32_t x) {
  return ROTR(x, 17) ^ ROTR(x, 19) ^ SHR(x, 10);
}

static const uint32_t k256[64] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

static void sha256_transform(SHA256_CTX* ctx, const uint8_t data[64]) {
  uint32_t m[64];
  for (int i = 0; i < 16; ++i) {
    m[i] = (uint32_t)data[i * 4 + 0] << 24 | (uint32_t)data[i * 4 + 1] << 16 | (uint32_t)data[i * 4 + 2] << 8 | (uint32_t)data[i * 4 + 3];
  }
  for (int i = 16; i < 64; ++i)
    m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];

  uint32_t a = ctx->state[0], b = ctx->state[1], c = ctx->state[2], d = ctx->state[3];
  uint32_t e = ctx->state[4], f = ctx->state[5], g = ctx->state[6], h = ctx->state[7];

  for (int i = 0; i < 64; ++i) {
    uint32_t t1 = h + EP1(e) + Ch(e, f, g) + k256[i] + m[i];
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

static void sha256_init(SHA256_CTX* ctx) {
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

static void sha256_update(SHA256_CTX* ctx, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    ctx->data[ctx->datalen++] = data[i];
    if (ctx->datalen == 64) {
      sha256_transform(ctx, ctx->data);
      ctx->bitlen += 512;
      ctx->datalen = 0;
    }
  }
}

static void sha256_final(SHA256_CTX* ctx, uint8_t hash[32]) {
  uint32_t i = ctx->datalen;

  // pad current block
  ctx->data[i++] = 0x80;
  if (i > 56) {
    while (i < 64) ctx->data[i++] = 0;
    sha256_transform(ctx, ctx->data);
    i = 0;
  }
  while (i < 56) ctx->data[i++] = 0;

  // append total bits, big-endian
  ctx->bitlen += ctx->datalen * 8ull;
  uint64_t bl = ctx->bitlen;
  ctx->data[63] = (uint8_t)(bl);
  ctx->data[62] = (uint8_t)(bl >> 8);
  ctx->data[61] = (uint8_t)(bl >> 16);
  ctx->data[60] = (uint8_t)(bl >> 24);
  ctx->data[59] = (uint8_t)(bl >> 32);
  ctx->data[58] = (uint8_t)(bl >> 40);
  ctx->data[57] = (uint8_t)(bl >> 48);
  ctx->data[56] = (uint8_t)(bl >> 56);
  sha256_transform(ctx, ctx->data);

  // output big-endian
  for (int j = 0; j < 8; ++j) {
    uint32_t s = ctx->state[j];
    hash[j * 4 + 0] = (uint8_t)(s >> 24);
    hash[j * 4 + 1] = (uint8_t)(s >> 16);
    hash[j * 4 + 2] = (uint8_t)(s >> 8);
    hash[j * 4 + 3] = (uint8_t)(s);
  }
}

// Helper: compute SHA-256 of a file using readFileRange()
static bool sha256File(const char* fname, uint8_t out[32]) {
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz)) {
    Console.println("hash: not found");
    return false;
  }
  const size_t CHUNK = 1024;
  uint8_t buf[CHUNK];
  SHA256_CTX ctx;
  sha256_init(&ctx);

  uint32_t off = 0;
  while (off < sz) {
    size_t n = (sz - off > CHUNK) ? CHUNK : (sz - off);
    uint32_t got = activeFs.readFileRange(fname, off, buf, n);
    if (got != n) {
      Console.println("hash: read error");
      return false;
    }
    sha256_update(&ctx, buf, n);
    off += n;
    yield();
  }
  sha256_final(&ctx, out);
  return true;
}

static void printHexLower(const uint8_t* d, size_t n) {
  static const char* hexd = "0123456789abcdef";
  for (size_t i = 0; i < n; ++i) {
    Serial.write(hexd[d[i] >> 4]);
    Serial.write(hexd[d[i] & 0xF]);
  }
}