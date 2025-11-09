#pragma once
// shrxbin.h - Header-only raw framed binary receiver over Stream
// Frame (LE): MAGIC(4)=A5 5A 4B 52, u32 offset, u32 len, u32 crc32(payload), payload[len]
// Commit: offset=0xFFFFFFFF, len=0, crc=0

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#ifndef SHRXBIN_MAX_FRAME
#define SHRXBIN_MAX_FRAME 32768
#endif

namespace shrxbin {

static inline uint32_t crc32_ieee(const uint8_t* p, size_t n) {
  static bool init = false;
  static uint32_t T[256];
  if (!init) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int j = 0; j < 8; ++j)
        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      T[i] = c;
    }
    init = true;
  }
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i)
    c = T[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

struct Writer {
  // Must write 'len' bytes at absolute device address 'absAddr'
  bool (*writeAbs)(void* ctx, uint32_t absAddr, const uint8_t* data, uint32_t len) = nullptr;
  // Optional finalize: set logical file size in FS metadata
  bool (*finalizeSize)(void* ctx, const char* name, uint32_t size, uint32_t baseAddr, uint32_t cap) = nullptr;

  void* ctx = nullptr;
  uint32_t baseAddr = 0;
  uint32_t cap = 0;
};

struct State {
  Stream* io = nullptr;
  Writer wr{};
  // File meta
  char name[48];
  uint32_t total = 0;
  uint32_t received = 0;
  bool active = false;
  // Frame parsing
  uint8_t hdr[16];
  uint32_t hdrGot = 0;
  uint32_t frameOff = 0;
  uint32_t frameLen = 0;
  uint32_t frameCRC = 0;
  uint32_t payGot = 0;
  uint8_t pay[SHRXBIN_MAX_FRAME];
};

inline bool active(const State& st) {
  return st.active;
}

inline void end(State& st, bool ok, const char* msg = nullptr) {
  st.active = false;
  st.hdrGot = st.payGot = 0;
  st.frameLen = 0;
  if (st.io) {
    if (ok) st.io->println("OK");
    else {
      st.io->print("ERR");
      if (msg) {
        st.io->print(' ');
        st.io->print(msg);
      }
      st.io->println();
    }
  }
}

inline bool begin(State& st, Stream& io, const char* fname, uint32_t total, const Writer& wr) {
  if (!wr.writeAbs) return false;
  memset(&st, 0, sizeof(st));
  st.io = &io;
  st.wr = wr;
  st.total = total;
  st.received = 0;
  st.active = true;
  if (fname) {
    strncpy(st.name, fname, sizeof(st.name) - 1);
    st.name[sizeof(st.name) - 1] = 0;
  }
  st.io->println("READY");
  return true;
}

inline void pump(State& st) {
  if (!st.active || !st.io) return;
  while (st.io->available()) {
    if (st.hdrGot < 16) {
      st.hdr[st.hdrGot++] = (uint8_t)st.io->read();
      if (st.hdrGot < 16) continue;

      if (!(st.hdr[0] == 0xA5 && st.hdr[1] == 0x5A && st.hdr[2] == 0x4B && st.hdr[3] == 0x52)) {
        end(st, false, "bad-magic");
        return;
      }
      st.frameOff = (uint32_t)st.hdr[4] | ((uint32_t)st.hdr[5] << 8) | ((uint32_t)st.hdr[6] << 16) | ((uint32_t)st.hdr[7] << 24);
      st.frameLen = (uint32_t)st.hdr[8] | ((uint32_t)st.hdr[9] << 8) | ((uint32_t)st.hdr[10] << 16) | ((uint32_t)st.hdr[11] << 24);
      st.frameCRC = (uint32_t)st.hdr[12] | ((uint32_t)st.hdr[13] << 8) | ((uint32_t)st.hdr[14] << 16) | ((uint32_t)st.hdr[15] << 24);

      if (st.frameOff == 0xFFFFFFFFu && st.frameLen == 0) {
        if (st.received != st.total) {
          end(st, false, "size-mismatch");
          return;
        }
        // finalize size if provided
        if (st.wr.finalizeSize) {
          if (!st.wr.finalizeSize(st.wr.ctx, st.name, st.total, st.wr.baseAddr, st.wr.cap)) {
            end(st, false, "finalize");
            return;
          }
        }
        end(st, true, nullptr);
        return;
      }

      if (st.frameLen == 0 || st.frameLen > SHRXBIN_MAX_FRAME) {
        end(st, false, "bad-len");
        return;
      }
      if (st.frameOff != st.received) {
        end(st, false, "bad-off");
        return;
      }
      if (st.wr.cap && (st.frameOff + st.frameLen > st.wr.cap)) {
        end(st, false, "cap");
        return;
      }

      st.payGot = 0;
    }

    while (st.io->available() && st.payGot < st.frameLen) {
      st.pay[st.payGot++] = (uint8_t)st.io->read();
    }
    if (st.payGot < st.frameLen) return;

    uint32_t c = crc32_ieee(st.pay, st.frameLen);
    if (c != st.frameCRC) {
      end(st, false, "crc");
      return;
    }

    uint32_t abs = st.wr.baseAddr + st.frameOff;
    if (!st.wr.writeAbs(st.wr.ctx, abs, st.pay, st.frameLen)) {
      end(st, false, "write");
      return;
    }

    st.received += st.frameLen;
    st.hdrGot = 0;
    st.payGot = 0;
  }
}

}  // namespace shrxbin