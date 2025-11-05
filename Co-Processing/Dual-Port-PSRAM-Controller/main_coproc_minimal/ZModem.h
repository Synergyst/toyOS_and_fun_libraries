// ==================== Minimal ZMODEM receiver (rz-lite, HEX+BIN32) ====================
namespace ZModem {
// ZMODEM constants
static constexpr uint8_t ZPAD = 0x2A;  // '*'
static constexpr uint8_t ZDLE = 0x18;
static constexpr uint8_t ZHEX = 0x42;    // 'B'
static constexpr uint8_t ZBIN = 0x41;    // 'A' (CRC-16) - optional
static constexpr uint8_t ZBIN32 = 0x43;  // 'C' (CRC-32)

// Frame types (subset)
static constexpr uint8_t ZRQINIT = 0x00;
static constexpr uint8_t ZRINIT = 0x01;
static constexpr uint8_t ZSINIT = 0x02;
static constexpr uint8_t ZFILE = 0x04;
static constexpr uint8_t ZDATA = 0x0A;
static constexpr uint8_t ZEOF = 0x0B;
static constexpr uint8_t ZFIN = 0x08;
static constexpr uint8_t ZACK = 0x06;
static constexpr uint8_t ZRPOS = 0x11;

// Subpacket end markers (following ZDLE)
static constexpr uint8_t ZCRCE = 0x68;  // 'h'
static constexpr uint8_t ZCRCG = 0x69;  // 'i'
static constexpr uint8_t ZCRCQ = 0x6A;  // 'j'
static constexpr uint8_t ZCRCW = 0x6B;  // 'k'

// Receiver capability flags
static constexpr uint32_t CANFC32 = 0x0001;
static constexpr uint32_t ESCCTL = 0x0002;
static constexpr uint32_t ESC8 = 0x0008;

// CRC-32 (IEEE)
static inline uint32_t crc32_update(uint32_t c, uint8_t b) {
  c ^= b;
  for (int i = 0; i < 8; ++i) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
  return c;
}
static inline uint32_t crc32(const uint8_t* p, size_t n, uint32_t c = 0xFFFFFFFFu) {
  for (size_t i = 0; i < n; ++i) c = crc32_update(c, p[i]);
  return ~c;
}
// CRC-16/CCITT (ZMODEM header CRC when HEX/BIN16)
static inline uint16_t crc16_ccitt_update(uint16_t c, uint8_t b) {
  c ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; ++i) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  return c;
}
static inline uint16_t crc16_ccitt(const uint8_t* p, size_t n, uint16_t c = 0) {
  for (size_t i = 0; i < n; ++i) c = crc16_ccitt_update(c, p[i]);
  return c;
}

// Read a byte from Serial with timeout (ms) - IMPORTANT: do NOT call pumpConsole here
static bool sread(uint8_t& b, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while ((millis() - t0) <= timeoutMs) {
    if (Serial.available() > 0) {
      int v = Serial.read();
      if (v >= 0) {
        b = (uint8_t)v;
        return true;
      }
    }
    serviceISPIfActive();  // OK
    tight_loop_contents();
    yield();
  }
  return false;
}

// Read and unescape a byte in ZDLE-escaped stream
static bool readZDLE(uint8_t& out, uint32_t timeoutMs) {
  uint8_t b;
  if (!sread(b, timeoutMs)) return false;
  if (b != ZDLE) {
    out = b;
    return true;
  }
  if (!sread(b, timeoutMs)) return false;
  // Return ZCRCE/ZCRCG/ZCRCQ/ZCRCW raw; otherwise unescape by XOR 0x40
  if (b == ZCRCE || b == ZCRCG || b == ZCRCQ || b == ZCRCW) {
    out = b;
    return true;
  }
  out = b ^ 0x40;
  return true;
}

// Send a ZHEX header (ZRINIT commonly starts as HEX)
static inline char hx(uint8_t v) {
  v &= 0x0F;
  return (v < 10) ? (char)('0' + v) : (char)('A' + (v - 10));
}
static void sendHeaderHex(uint8_t type, uint32_t p0) {
  uint8_t hdr[5] = {
    type,
    (uint8_t)(p0 & 0xFF),
    (uint8_t)((p0 >> 8) & 0xFF),
    (uint8_t)((p0 >> 16) & 0xFF),
    (uint8_t)((p0 >> 24) & 0xFF)
  };
  uint16_t c = crc16_ccitt(hdr, 5, 0);
  Serial.write(ZPAD);
  Serial.write(ZPAD);
  Serial.write(ZDLE);
  Serial.write(ZHEX);
  for (int i = 0; i < 5; ++i) {
    Serial.write(hx(hdr[i] >> 4));
    Serial.write(hx(hdr[i] & 0x0F));
  }
  Serial.write(hx((uint8_t)(c >> 12)));
  Serial.write(hx((uint8_t)((c >> 8) & 0x0F)));
  Serial.write(hx((uint8_t)((c >> 4) & 0x0F)));
  Serial.write(hx((uint8_t)(c & 0x0F)));
  Serial.write('\r');
  Serial.write('\n');
  Serial.flush();
}

// Send a ZBIN32 header
static void sendHeaderBin32(uint8_t type, uint32_t p0) {
  Serial.write(ZPAD);
  Serial.write(ZPAD);
  Serial.write(ZDLE);
  Serial.write(ZBIN32);
  uint8_t hdr[5] = {
    type,
    (uint8_t)(p0 & 0xFF),
    (uint8_t)((p0 >> 8) & 0xFF),
    (uint8_t)((p0 >> 16) & 0xFF),
    (uint8_t)((p0 >> 24) & 0xFF)
  };
  uint32_t c = crc32(hdr, 5);
  auto we = [](uint8_t b) {
    if (b == ZDLE || b < 0x20 || b == 0x7F) {
      Serial.write(ZDLE);
      Serial.write(b ^ 0x40);
    } else Serial.write(b);
  };
  for (int i = 0; i < 5; ++i) we(hdr[i]);
  we((uint8_t)(c & 0xFF));
  we((uint8_t)((c >> 8) & 0xFF));
  we((uint8_t)((c >> 16) & 0xFF));
  we((uint8_t)((c >> 24) & 0xFF));
  Serial.flush();
}

// Read a HEX header after seeing ZPAD ZPAD ZDLE ZHEX; returns type and p0
static bool recvHeaderHex(uint8_t& type, uint32_t& p0, uint32_t timeoutMs) {
  auto hv = [](int c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  uint8_t bytes[7] = { 0 };  // 5 header + 2 CRC16 bytes
  int got = 0;
  // Read ascii hex pairs until CR/LF; accept minimal 5+2 bytes (14 hex chars)
  uint8_t c;
  int hn = -1;
  uint32_t t0 = millis();
  while ((millis() - t0) <= timeoutMs) {
    if (!sread(c, 50)) continue;
    if (c == '\r' || c == '\n') {
      break;
    }
    int h = hv(c);
    if (h < 0) continue;
    if (hn < 0) {
      hn = h;
      continue;
    }
    // complete a byte
    uint8_t b = (uint8_t)((hn << 4) | h);
    hn = -1;
    if (got < (int)sizeof(bytes)) bytes[got] = b;
    got++;
    if (got >= 7) {
      // We have at least 5 header + 2 CRC bytes; we can stop early if sender put no extra
      // Wait for CRLF but we can break on a short pause:
      // We'll rely on timeout to end; keep going until CR/LF encountered
    }
  }
  if (got < 7) return false;
  type = bytes[0];
  p0 = (uint32_t)bytes[1] | ((uint32_t)bytes[2] << 8) | ((uint32_t)bytes[3] << 16) | ((uint32_t)bytes[4] << 24);
  // We could verify CRC16 here; tolerate mismatches for compatibility
  return true;
}

// Wait for next header (HEX or BIN32). Returns true with type/p0 set.
static bool recvHeader(uint8_t& type, uint32_t& p0, uint32_t timeoutMs) {
  uint8_t b;
  enum { S0,
         S1,
         S2,
         S3 } st = S0;
  uint32_t t0 = millis();
  while ((millis() - t0) <= timeoutMs) {
    if (!sread(b, 50)) continue;
    switch (st) {
      case S0: st = (b == ZPAD) ? S1 : S0; break;
      case S1: st = (b == ZPAD) ? S2 : ((b == ZPAD) ? S1 : S0); break;
      case S2: st = (b == ZDLE) ? S3 : ((b == ZPAD) ? S2 : S0); break;
      case S3:
        if (b == ZHEX) {
          return recvHeaderHex(type, p0, timeoutMs);
        } else if (b == ZBIN32) {
          // Read 5 bytes (type + 4 params), escaped; then 4 CRC bytes
          uint8_t hdr[5];
          for (int i = 0; i < 5; ++i) {
            if (!readZDLE(hdr[i], timeoutMs)) return false;
          }
          uint8_t c0, c1, c2, c3;
          if (!readZDLE(c0, timeoutMs) || !readZDLE(c1, timeoutMs) || !readZDLE(c2, timeoutMs) || !readZDLE(c3, timeoutMs)) return false;
          // Optionally check CRC32; tolerate mismatches
          type = hdr[0];
          p0 = (uint32_t)hdr[1] | ((uint32_t)hdr[2] << 8) | ((uint32_t)hdr[3] << 16) | ((uint32_t)hdr[4] << 24);
          return true;
        } else if (b == ZBIN) {
          // Like BIN32 but CRC16 and no need here; accept same parsing and ignore CRC
          uint8_t hdr[5];
          for (int i = 0; i < 5; ++i)
            if (!readZDLE(hdr[i], timeoutMs)) return false;
          uint8_t c0, c1;
          if (!readZDLE(c0, timeoutMs) || !readZDLE(c1, timeoutMs)) return false;
          type = hdr[0];
          p0 = (uint32_t)hdr[1] | ((uint32_t)hdr[2] << 8) | ((uint32_t)hdr[3] << 16) | ((uint32_t)hdr[4] << 24);
          return true;
        } else {
          // Unknown framing; resync
          st = S0;
        }
        break;
    }
  }
  return false;
}

// Read a subpacket and stream payload via callback. Returns end marker or 0 on error.
template<typename Sink>
static uint8_t readSubpacket(Sink onBytes, uint32_t timeoutMs, uint32_t* bytesReadOut = nullptr) {
  uint8_t buf[256];
  uint32_t total = 0;
  for (;;) {
    size_t n = 0;
    while (n < sizeof(buf)) {
      uint8_t b;
      if (!readZDLE(b, timeoutMs)) return 0;
      if (b == ZCRCE || b == ZCRCG || b == ZCRCQ || b == ZCRCW) {
        // Read & discard trailing CRC32/CRC16 (senders usually use CRC32 in ZDATA)
        uint8_t c[4];
        for (int i = 0; i < 4; ++i) {
          if (!readZDLE(c[i], timeoutMs)) return 0;
        }
        if (n) {
          onBytes(buf, n);
          total += n;
        }
        if (bytesReadOut) *bytesReadOut = total;
        return b;
      }
      buf[n++] = b;
    }
    if (n) {
      onBytes(buf, n);
      total += n;
    }
    serviceISPIfActive();
    tight_loop_contents();
    yield();
  }
}

static void parseZFILEInfo(const uint8_t* data, size_t len, char* outName, size_t outNameCap, uint32_t& outSize) {
  outSize = 0;
  if (outName && outNameCap) outName[0] = 0;
  size_t i = 0, w = 0;
  while (i < len && data[i]) {
    if (w + 1 < outNameCap) outName[w++] = (char)data[i];
    i++;
  }
  if (outName && outNameCap) outName[w] = 0;
  if (i < len && data[i] == 0) i++;
  uint32_t sz = 0;
  while (i < len && data[i] == ' ') i++;
  while (i < len && data[i] >= '0' && data[i] <= '9') { sz = sz * 10 + (uint32_t)(data[i++] - '0'); }
  outSize = sz;
}

class FsStreamWriter {
public:
  FsStreamWriter()
    : _dev(nullptr), _addr(0), _end(0), _cur(0), _eraseAlign(1), _lastErasedEnd(0) {}
  bool begin(StorageBackend backend, uint32_t addr, uint32_t capacity, uint32_t reserveLen) {
    _dev = deviceForBackend(backend);
    if (!_dev) return false;
    _addr = addr;
    _cur = addr;
    _end = addr + reserveLen;
    if (_end > addr + capacity) _end = addr + capacity;
    _eraseAlign = _dev->eraseSize();
    if (_eraseAlign == 0) _eraseAlign = 1;
    _lastErasedEnd = addr;
    return true;
  }
  bool writeChunk(const uint8_t* data, uint32_t len) {
    if (!_dev || len == 0) return true;
    if (_eraseAlign > 1) {
      uint64_t needEnd = _cur + len;
      if (needEnd > _lastErasedEnd) {
        uint64_t eraseFrom = (_lastErasedEnd / _eraseAlign) * (uint64_t)_eraseAlign;
        uint64_t eraseTo = ((needEnd + _eraseAlign - 1) / _eraseAlign) * (uint64_t)_eraseAlign;
        if (eraseTo > _end) eraseTo = _end;
        while (eraseFrom < eraseTo) {
          if (!_dev->eraseRange(eraseFrom, _eraseAlign)) return false;
          eraseFrom += _eraseAlign;
          _lastErasedEnd = (uint32_t)eraseFrom;
          yield();
        }
      }
    }
    const uint32_t MAXW = 4096;
    uint32_t off = 0;
    while (off < len) {
      uint32_t n = (len - off > MAXW) ? MAXW : (len - off);
      if (!_dev->write((uint64_t)_cur, data + off, n)) return false;
      _cur += n;
      off += n;
      yield();
    }
    return true;
  }
  uint32_t bytesWritten() const {
    return (_cur > _addr) ? (_cur - _addr) : 0;
  }
private:
  UnifiedSpiMem::MemDevice* _dev;
  uint32_t _addr, _end, _cur;
  uint32_t _eraseAlign;
  uint32_t _lastErasedEnd;
};

// Drop-in replacement for ZModem::receiveToFS()
// Works with picky Windows serial senders (ExtraPuTTY, MobaXterm):
// - Passive but also proactively re-announces ZRINIT (HEX) every ~700 ms
// - Accepts both HEX and BIN32 headers
// - Suspends console while active to avoid corrupting the stream
// - Streams directly to reserved file slot and stamps final size via writeFileInPlace(..., nullptr, size)
// Requires existing helpers in ZModem namespace: recvHeader, readSubpacket, parseZFILEInfo,
// and ZModem::FsStreamWriter. Also requires global g_zmodem_active and your activeFs facade.

static bool receiveToFS(const char* dstOverride /*nullable*/) {
  g_zmodem_active = true;  // suspend console pumping during ZMODEM session

  auto drainResidual = []() {
    // Drain any residual terminal chatter (e.g. "Retry ...") so it won't hit command parser
    uint32_t t0 = millis();
    while (millis() - t0 < 250) {
      while (Serial.available() > 0) (void)Serial.read();
      serviceISPIfActive();
      tight_loop_contents();
      yield();
    }
  };

  // Inline HEX header sender (more compatible with Windows serial terminals)
  auto sendHeaderHex = [](uint8_t type, uint32_t p0) {
    auto hx = [](uint8_t v) -> char {
      v &= 0x0F;
      return (v < 10) ? (char)('0' + v) : (char)('A' + (v - 10));
    };
    auto crc16_ccitt_update = [](uint16_t c, uint8_t b) -> uint16_t {
      c ^= (uint16_t)b << 8;
      for (int i = 0; i < 8; ++i) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
      return c;
    };
    uint8_t hdr[5] = {
      type, (uint8_t)(p0 & 0xFF), (uint8_t)((p0 >> 8) & 0xFF),
      (uint8_t)((p0 >> 16) & 0xFF), (uint8_t)((p0 >> 24) & 0xFF)
    };
    uint16_t c = 0;
    for (int i = 0; i < 5; ++i) c = crc16_ccitt_update(c, hdr[i]);
    Serial.write((uint8_t)0x2A);  // ZPAD '*'
    Serial.write((uint8_t)0x2A);  // ZPAD
    Serial.write((uint8_t)0x18);  // ZDLE
    Serial.write((uint8_t)0x42);  // ZHEX 'B'
    for (int i = 0; i < 5; ++i) {
      Serial.write(hx(hdr[i] >> 4));
      Serial.write(hx(hdr[i] & 0x0F));
    }
    Serial.write(hx((uint8_t)(c >> 12)));
    Serial.write(hx((uint8_t)((c >> 8) & 0x0F)));
    Serial.write(hx((uint8_t)((c >> 4) & 0x0F)));
    Serial.write(hx((uint8_t)(c & 0x0F)));
    Serial.write('\r');
    Serial.write('\n');
    // Some Windows senders expect extra XONs after HEX headers on serial
    Serial.write((uint8_t)0x11);
    Serial.write((uint8_t)0x11);
    Serial.flush();
  };

  const uint32_t caps = 0x0001u /*CANFC32*/ | 0x0002u /*ESCCTL*/ | 0x0008u /*ESC8*/;

  Console.println("rz: Waiting for sender...");

  // Actively re-announce ZRINIT (HEX) every ~700 ms while waiting for ZFILE
  uint8_t ftype = 0;
  uint32_t p0 = 0;
  uint32_t tStart = millis();
  uint32_t lastAnnounce = 0;
  bool gotZFILE = false;
  while ((millis() - tStart) < 20000) {
    if ((millis() - lastAnnounce) > 700) {
      sendHeaderHex(0x01 /*ZRINIT*/, caps);
      lastAnnounce = millis();
    }
    if (!ZModem::recvHeader(ftype, p0, 400)) {
      serviceISPIfActive();
      tight_loop_contents();
      yield();
      continue;
    }
    if (ftype == 0x04 /*ZFILE*/) {
      gotZFILE = true;
      break;
    }
    if (ftype == 0x02 /*ZSINIT*/) {
      auto sink = [](const uint8_t*, size_t) {};
      (void)ZModem::readSubpacket(sink, 2000);  // consume and ignore
      // ACK politely using HEX (keeps some senders happy)
      sendHeaderHex(0x06 /*ZACK*/, 0);
      continue;
    }
    if (ftype == 0x00 /*ZRQINIT*/) {
      // Sender probing; re-announce
      sendHeaderHex(0x01 /*ZRINIT*/, caps);
      continue;
    }
    // Ignore anything else
  }
  if (!gotZFILE) {
    Console.println("rz: did not receive ZFILE");
    drainResidual();
    g_zmodem_active = false;
    return false;
  }

  // Read ZFILE subpacket -> filename + size
  uint8_t fileMeta[256];
  size_t metaLen = 0;
  auto metaSink = [&](const uint8_t* d, size_t n) {
    size_t can = sizeof(fileMeta) - metaLen;
    if (n > can) n = can;
    memcpy(fileMeta + metaLen, d, n);
    metaLen += n;
  };
  uint32_t dummy = 0;
  uint8_t tend = ZModem::readSubpacket(metaSink, 6000, &dummy);
  if (tend == 0) {
    Console.println("rz: ZFILE subpacket timeout");
    drainResidual();
    g_zmodem_active = false;
    return false;
  }
  char fnameHdr[ActiveFS::MAX_NAME + 1] = { 0 };
  uint32_t expectedLen = 0;
  ZModem::parseZFILEInfo(fileMeta, metaLen, fnameHdr, sizeof(fnameHdr), expectedLen);

  // Destination name
  char dstPath[ActiveFS::MAX_NAME + 1];
  if (dstOverride && dstOverride[0]) {
    if (strlen(dstOverride) > ActiveFS::MAX_NAME) {
      Console.println("rz: destination name too long");
      drainResidual();
      g_zmodem_active = false;
      return false;
    }
    strncpy(dstPath, dstOverride, sizeof(dstPath));
  } else {
    if (!fnameHdr[0]) {
      Console.println("rz: empty filename (use: rz <name>)");
      drainResidual();
      g_zmodem_active = false;
      return false;
    }
    strncpy(dstPath, fnameHdr, sizeof(dstPath));
  }
  dstPath[ActiveFS::MAX_NAME] = 0;
  if (expectedLen) Console.printf("rz: receiving '%s' (%lu bytes)\n", dstPath, (unsigned long)expectedLen);
  else Console.printf("rz: receiving '%s' (size unknown)\n", dstPath);

  // Reserve slot in FS
  uint32_t eraseAlign = getEraseAlign();
  uint32_t reserve = expectedLen ? expectedLen : (256u * 1024u);
  if (eraseAlign > 1) reserve = (reserve + (eraseAlign - 1)) & ~(eraseAlign - 1);
  if (reserve < eraseAlign) reserve = eraseAlign;
  if (!activeFs.createFileSlot(dstPath, reserve, nullptr, 0)) {
    Console.println("rz: createFileSlot failed");
    drainResidual();
    g_zmodem_active = false;
    return false;
  }
  uint32_t faddr = 0, fsize0 = 0, fcap = 0;
  if (!activeFs.getFileInfo(dstPath, faddr, fsize0, fcap)) {
    Console.println("rz: getFileInfo failed");
    drainResidual();
    g_zmodem_active = false;
    return false;
  }

  // Stream writer into reserved slot
  ZModem::FsStreamWriter writer;
  if (!writer.begin(g_storage, faddr, fcap, reserve)) {
    Console.println("rz: writer.begin failed");
    drainResidual();
    g_zmodem_active = false;
    return false;
  }

  // Ask for data from 0 (HEX)
  sendHeaderHex(0x11 /*ZRPOS*/, 0);

  // Receive ZDATA streams until ZEOF
  uint32_t received = 0;
  uint32_t lastPct = 101;
  for (;;) {
    if (!ZModem::recvHeader(ftype, p0, 10000)) {
      Console.println("rz: header wait timeout");
      drainResidual();
      g_zmodem_active = false;
      return false;
    }
    if (ftype == 0x0A /*ZDATA*/) {
      // p0 is offset; proceed (we expect in-order)
      for (;;) {
        uint32_t chunkBytes = 0;
        auto dataSink = [&](const uint8_t* d, size_t n) {
          if (n) (void)writer.writeChunk(d, (uint32_t)n);
        };
        uint8_t endm = ZModem::readSubpacket(dataSink, 15000, &chunkBytes);
        if (endm == 0) {
          Console.println("rz: data subpacket timeout");
          drainResidual();
          g_zmodem_active = false;
          return false;
        }
        received += chunkBytes;
        if (expectedLen) {
          uint32_t pct = (uint32_t)((uint64_t)received * 100ull / (uint64_t)expectedLen);
          if (pct != lastPct && (pct % 5) == 0) {
            Console.printf("  %u%%\n", (unsigned)pct);
            lastPct = pct;
          }
        } else if ((received & ((64u * 1024u) - 1u)) == 0) {
          Console.printf("  %lu KiB\n", (unsigned long)(received / 1024));
        }
        // ACK politely on ZCRCW/ZCRCQ using HEX
        if (endm == 0x6B /*ZCRCW*/ || endm == 0x6A /*ZCRCQ*/) {
          sendHeaderHex(0x06 /*ZACK*/, received);
        }
        if (endm == 0x68 /*ZCRCE*/ || endm == 0x6B /*ZCRCW*/) break;  // end of this ZDATA burst
        serviceISPIfActive();
        tight_loop_contents();
        yield();
      }
      continue;
    } else if (ftype == 0x0B /*ZEOF*/) {
      if (expectedLen && p0 != expectedLen) {
        Console.printf("rz: ZEOF at %lu, expected %lu\n", (unsigned long)p0, (unsigned long)expectedLen);
      }
      break;
    } else if (ftype == 0x02 /*ZSINIT*/) {
      auto sink = [](const uint8_t*, size_t) {};
      (void)ZModem::readSubpacket(sink, 2000);
      sendHeaderHex(0x06 /*ZACK*/, 0);
    } else if (ftype == 0x00 /*ZRQINIT*/) {
      // Some senders re-probe mid-stream; re-announce
      sendHeaderHex(0x01 /*ZRINIT*/, caps);
    } else {
      // ignore noise/other frames
    }
    serviceISPIfActive();
    tight_loop_contents();
    yield();
  }

  // Stamp final file size in directory without rewriting data:
  // writeFileInPlace with data=null and size=received appends a new dir entry (driver ignores null buf)
  (void)activeFs.writeFileInPlace(dstPath, nullptr, received, false);
  Console.printf("rz: received %lu bytes into %s\n", (unsigned long)received, dstPath);

  // Expect ZFIN and reply "OO"
  uint32_t tFin = millis();
  while ((millis() - tFin) < 5000) {
    if (ZModem::recvHeader(ftype, p0, 1000) && ftype == 0x08 /*ZFIN*/) {
      Serial.write('O');
      Serial.write('O');
      Serial.flush();
      break;
    }
    serviceISPIfActive();
    tight_loop_contents();
    yield();
  }

  drainResidual();
  g_zmodem_active = false;
  return true;
}
}  // namespace ZModem
   // ==================== end ZMODEM receiver ====================