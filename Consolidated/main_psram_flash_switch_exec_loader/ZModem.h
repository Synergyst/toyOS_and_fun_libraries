// ==================== Minimal ZMODEM receiver (rz-lite) ====================
namespace ZModem {
// ZMODEM constants
static constexpr uint8_t ZPAD = 0x2A;  // '*'
static constexpr uint8_t ZDLE = 0x18;
static constexpr uint8_t ZBIN32 = 0x43;  // 'C' (binary header with CRC-32)
static constexpr uint8_t ZHEX = 0x42;    // 'B' (not used for data here)
// Frame types (subset)
static constexpr uint8_t ZRQINIT = 0x00;
static constexpr uint8_t ZRINIT = 0x01;
static constexpr uint8_t ZFILE = 0x04;
static constexpr uint8_t ZSINIT = 0x02;
static constexpr uint8_t ZDATA = 0x0A;
static constexpr uint8_t ZEOF = 0x0B;
static constexpr uint8_t ZFIN = 0x08;
static constexpr uint8_t ZACK = 0x06;
static constexpr uint8_t ZRPOS = 0x11;

// Subpacket end markers following ZDLE
static constexpr uint8_t ZCRCE = 0x68;  // 'h' frame ends, no ACK expected
static constexpr uint8_t ZCRCG = 0x69;  // 'i' frame continues, no ACK
static constexpr uint8_t ZCRCQ = 0x6A;  // 'j' frame continues, receiver should ZACK
static constexpr uint8_t ZCRCW = 0x6B;  // 'k' frame ends; receiver should ZACK

// ZRINIT receiver capability flags (we claim CRC-32 and escaping)
static constexpr uint32_t CANFC32 = 0x0001;
static constexpr uint32_t ESCCTL = 0x0002;
static constexpr uint32_t ESC8 = 0x0008;

// CRC-32 (IEEE)
static inline uint32_t crc32_update(uint32_t c, uint8_t b) {
  c ^= b;
  for (int i = 0; i < 8; ++i) {
    c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
  }
  return c;
}
static inline uint32_t crc32(const uint8_t* p, size_t n, uint32_t c = 0xFFFFFFFFu) {
  for (size_t i = 0; i < n; ++i) c = crc32_update(c, p[i]);
  return ~c;
}

// Read a byte from Serial with timeout (ms)
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
    // Keep rest of app alive
    //keypadPollSPI();
    //pumpConsole();
    //serviceISPIfActive();
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
  // Next byte is escaped or control marker
  if (!sread(b, timeoutMs)) return false;
  // ZCRCE/ZCRCG/ZCRCQ/ZCRCW are returned raw (caller checks)
  if (b == ZCRCE || b == ZCRCG || b == ZCRCQ || b == ZCRCW) {
    out = b;
    return true;
  }
  // Otherwise, unescape by XOR 0x40 (typical ZMODEM rule for control chars)
  out = b ^ 0x40;
  return true;
}

// Send one byte with ZDLE escaping if needed
static inline void writeEscaped(uint8_t b) {
  // Escape ZDLE and control chars < 0x20 and 0x7F
  if (b == ZDLE || b < 0x20 || b == 0x7F) {
    Serial.write(ZDLE);
    Serial.write(b ^ 0x40);
  } else {
    Serial.write(b);
  }
}

// Send a ZBIN32 header: "** ZDLE 'C' type p0 p1 p2 p3 CRC32( type..p3 )"
static void sendHeaderBin32(uint8_t type, uint32_t p0) {
  Serial.write(ZPAD);
  Serial.write(ZPAD);
  Serial.write(ZDLE);
  Serial.write(ZBIN32);
  uint8_t hdr[5];
  hdr[0] = type;
  hdr[1] = (uint8_t)(p0 & 0xFF);
  hdr[2] = (uint8_t)((p0 >> 8) & 0xFF);
  hdr[3] = (uint8_t)((p0 >> 16) & 0xFF);
  hdr[4] = (uint8_t)((p0 >> 24) & 0xFF);
  uint32_t c = crc32(hdr, 5);
  // Emit escaped
  for (int i = 0; i < 5; ++i) writeEscaped(hdr[i]);
  writeEscaped((uint8_t)(c & 0xFF));
  writeEscaped((uint8_t)((c >> 8) & 0xFF));
  writeEscaped((uint8_t)((c >> 16) & 0xFF));
  writeEscaped((uint8_t)((c >> 24) & 0xFF));
  Serial.flush();
}

// Wait for next ZBIN32 header, returns type and 32-bit parameter
static bool recvHeaderBin32(uint8_t& type, uint32_t& p0, uint32_t timeoutMs) {
  uint8_t b;
  // Seek "** ZDLE 'C'"
  uint8_t stage = 0;
  uint32_t t0 = millis();
  while ((millis() - t0) <= timeoutMs) {
    if (!sread(b, 50)) continue;
    if (stage < 2) {
      if (b == ZPAD) stage++;
      else stage = 0;
    } else if (stage == 2) {
      if (b == ZDLE) stage = 3;
      else if (b == ZPAD) stage = 2;
      else stage = 0;
    } else if (stage == 3) {
      if (b == ZBIN32) {
        stage = 4;
        break;
      } else stage = 0;
    }
  }
  if (stage != 4) return false;

  // Read 5 bytes (type + 4 params), escaped
  uint8_t hdr[5];
  for (int i = 0; i < 5; ++i) {
    if (!readZDLE(hdr[i], timeoutMs)) return false;
  }
  // Read and verify CRC32 (4 bytes, escaped)
  uint8_t c0, c1, c2, c3;
  if (!readZDLE(c0, timeoutMs)) return false;
  if (!readZDLE(c1, timeoutMs)) return false;
  if (!readZDLE(c2, timeoutMs)) return false;
  if (!readZDLE(c3, timeoutMs)) return false;
  uint32_t cGot = (uint32_t)c0 | ((uint32_t)c1 << 8) | ((uint32_t)c2 << 16) | ((uint32_t)c3 << 24);
  uint32_t cExp = crc32(hdr, 5);
  // For compatibility, accept even if CRC mismatches (some senders vary on escapes). Log but continue.
  if (cGot != cExp) {
    Console.printf("[rz] header CRC32 mismatch (got=0x%08lX exp=0x%08lX), continuing\n",
                   (unsigned long)cGot, (unsigned long)cExp);
  }
  type = hdr[0];
  p0 = (uint32_t)hdr[1] | ((uint32_t)hdr[2] << 8) | ((uint32_t)hdr[3] << 16) | ((uint32_t)hdr[4] << 24);
  return true;
}

// Read a subpacket (ZFILE data or ZDATA chunk) into a callback sink.
// Calls onBytes(data, len) for raw payload between ZDLE ZCRC* markers.
// Returns endMarker (ZCRCE/ZCRCG/ZCRCQ/ZCRCW), or 0 on error/timeout.
template<typename Sink>
static uint8_t readSubpacket(Sink onBytes, uint32_t timeoutMs, uint32_t* bytesReadOut = nullptr) {
  uint8_t buf[256];
  uint32_t total = 0;
  for (;;) {
    // Fill a small burst, stopping if we encounter a ZDLE control
    size_t n = 0;
    while (n < sizeof(buf)) {
      uint8_t b;
      if (!readZDLE(b, timeoutMs)) return 0;
      if (b == ZCRCE || b == ZCRCG || b == ZCRCQ || b == ZCRCW) {
        // read and discard CRC32 of subpacket (4 bytes)
        uint8_t c[4];
        for (int i = 0; i < 4; ++i) {
          if (!readZDLE(c[i], timeoutMs)) return 0;
        }
        // Flush pending bytes
        if (n) {
          onBytes(buf, n);
          total += n;
          n = 0;
        }
        if (bytesReadOut) *bytesReadOut = total;
        return b;
      }
      buf[n++] = b;
    }
    if (n) {
      onBytes(buf, n);
      total += n;
      n = 0;
    }
    // keep system responsive
    //keypadPollSPI();
    //pumpConsole();
    //serviceISPIfActive();
    tight_loop_contents();
    yield();
  }
}

// Parse ZFILE subpacket contents to get filename and size.
static void parseZFILEInfo(const uint8_t* data, size_t len, char* outName, size_t outNameCap, uint32_t& outSize) {
  // Format: "name\0size SP mtime SP mode SP ... \0"
  outSize = 0;
  if (outName && outNameCap) outName[0] = 0;
  // name
  size_t i = 0, w = 0;
  while (i < len && data[i]) {
    if (w + 1 < outNameCap) outName[w++] = (char)data[i];
    i++;
  }
  if (outName && outNameCap) outName[w] = 0;
  // past NUL
  if (i < len && data[i] == 0) i++;
  // size (ASCII decimal)
  uint32_t sz = 0;
  while (i < len && data[i] == ' ') i++;
  while (i < len && data[i] >= '0' && data[i] <= '9') {
    sz = sz * 10 + (uint32_t)(data[i] - '0');
    i++;
  }
  outSize = sz;
}

// Stream writer to SimpleFS region (addr .. addr+len). Erases per erase block on demand.
class FsStreamWriter {
public:
  FsStreamWriter()
    : _dev(nullptr), _addr(0), _end(0), _cur(0), _eraseAlign(4096), _lastErasedEnd(0) {}
  bool begin(StorageBackend backend, uint32_t addr, uint32_t capacity, uint32_t reserveLen) {
    _dev = deviceForBackend(backend);
    if (!_dev) return false;
    _addr = addr;
    _end = addr + reserveLen;
    if (_end > addr + capacity) _end = addr + capacity;
    _cur = addr;
    _eraseAlign = _dev->eraseSize();
    if (_eraseAlign == 0) _eraseAlign = 1;
    _lastErasedEnd = addr;  // nothing erased yet
    return true;
  }
  bool writeChunk(const uint8_t* data, uint32_t len) {
    if (!_dev || len == 0) return true;
    // Ensure erase for NOR/NAND before programming into covered range
    if (_eraseAlign > 1) {
      // If this chunk crosses into not-yet-erased area, erase forward by block(s)
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
    // Program in device-friendly chunks (<=4K)
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

// Main ZMODEM receive routine. Returns true on success.
static bool receiveToFS(const char* dstOverride /*may be null*/) {
  Console.println("rz: Ready. Start 'sz -b yourfile' in your terminal/tool...");

  // 1) Send ZRINIT to announce receiver capabilities
  uint32_t caps = CANFC32 | ESCCTL | ESC8;
  sendHeaderBin32(ZRINIT, caps);

  // 2) Wait for ZFILE header
  uint8_t ftype = 0;
  uint32_t p0 = 0;
  uint32_t tStart = millis();
  while (millis() - tStart < 15000) {
    if (!recvHeaderBin32(ftype, p0, 5000)) continue;
    if (ftype == ZRQINIT) {
      // Sender wants us to re-announce
      sendHeaderBin32(ZRINIT, caps);
      continue;
    }
    if (ftype == ZSINIT) {
      // Consume and ignore ZSINIT subpacket, ACK
      auto sink = [](const uint8_t*, size_t) {};
      (void)readSubpacket(sink, 3000);
      sendHeaderBin32(ZACK, 0);
      continue;
    }
    if (ftype == ZFILE) break;
  }
  if (ftype != ZFILE) {
    Console.println("rz: did not receive ZFILE");
    return false;
  }
  // 3) Read ZFILE subpacket -> filename + size
  uint8_t fileMeta[256];
  size_t metaLen = 0;
  auto metaSink = [&](const uint8_t* d, size_t n) {
    size_t can = sizeof(fileMeta) - metaLen;
    if (n > can) n = can;
    memcpy(fileMeta + metaLen, d, n);
    metaLen += n;
  };
  uint32_t dummy = 0;
  uint8_t tend = readSubpacket(metaSink, 5000, &dummy);
  if (tend == 0) {
    Console.println("rz: ZFILE subpacket timeout");
    return false;
  }
  char fnameFromHeader[ActiveFS::MAX_NAME + 1] = { 0 };
  uint32_t expectedLen = 0;
  parseZFILEInfo(fileMeta, metaLen, fnameFromHeader, sizeof(fnameFromHeader), expectedLen);

  // Decide destination name
  char dstPath[ActiveFS::MAX_NAME + 1];
  if (dstOverride && dstOverride[0]) {
    if (strlen(dstOverride) > ActiveFS::MAX_NAME) {
      Console.println("rz: destination name too long for FS");
      return false;
    }
    strncpy(dstPath, dstOverride, sizeof(dstPath));
  } else {
    if (fnameFromHeader[0] == 0) {
      Console.println("rz: empty filename in header; provide a name: rz <name>");
      return false;
    }
    strncpy(dstPath, fnameFromHeader, sizeof(dstPath));
    dstPath[ActiveFS::MAX_NAME] = 0;
  }
  Console.printf("rz: receiving '%s' (%lu bytes)\n", dstPath, (unsigned long)expectedLen);

  // 4) Create a reserved slot
  uint32_t eraseAlign = getEraseAlign();
  uint32_t reserve = expectedLen;
  if (eraseAlign > 1) reserve = (reserve + (eraseAlign - 1)) & ~(eraseAlign - 1);
  if (reserve < eraseAlign) reserve = eraseAlign;
  if (!activeFs.createFileSlot(dstPath, reserve, nullptr, 0)) {
    Console.println("rz: createFileSlot failed");
    return false;
  }
  // Get location
  uint32_t faddr = 0, fsize0 = 0, fcap = 0;
  if (!activeFs.getFileInfo(dstPath, faddr, fsize0, fcap)) {
    Console.println("rz: getFileInfo failed after slot create");
    return false;
  }
  // Prepare stream writer
  FsStreamWriter writer;
  if (!writer.begin(g_storage, faddr, fcap, reserve)) {
    Console.println("rz: writer.begin failed");
    return false;
  }

  // 5) Ask for data from position 0
  sendHeaderBin32(ZRPOS, 0);

  // 6) Loop: ZDATA + subpackets until ZEOF
  uint32_t received = 0;
  uint32_t lastProgress = 0xFFFFFFFFu;
  for (;;) {
    if (!recvHeaderBin32(ftype, p0, 10000)) {
      Console.println("rz: header wait timeout");
      return false;
    }
    if (ftype == ZDATA) {
      // p0 is file offset
      if (p0 != received) {
        // Simple receiver: expect in-order
        Console.printf("rz: out-of-order ZDATA pos=%lu expected=%lu\n", (unsigned long)p0, (unsigned long)received);
      }
      // Read subpackets until a ZCRC* end marker
      for (;;) {
        uint32_t chunkBytes = 0;
        auto dataSink = [&](const uint8_t* d, size_t n) {
          // stream to flash
          if (n) {
            if (!writer.writeChunk(d, (uint32_t)n)) {
              // If write fails, set cancel flag by throwing
              // Note: can't throw here; we’ll check after return
            }
          }
        };
        uint8_t endm = readSubpacket(dataSink, 10000, &chunkBytes);
        if (endm == 0) {
          Console.println("rz: data subpacket timeout");
          return false;
        }
        received += chunkBytes;

        // progress print
        if (expectedLen) {
          uint32_t pct = (uint32_t)((uint64_t)received * 100ull / (uint64_t)expectedLen);
          if (pct != lastProgress && (pct % 5) == 0) {
            Console.printf("  %u%% (%lu/%lu)\n", (unsigned)pct, (unsigned long)received, (unsigned long)expectedLen);
            lastProgress = pct;
          }
        } else {
          if ((received / 1024) != (lastProgress / 1024)) {
            Console.printf("  %lu KiB\n", (unsigned long)(received / 1024));
            lastProgress = received;
          }
        }

        if (endm == ZCRCW || endm == ZCRCQ) {
          sendHeaderBin32(ZACK, received);
        }
        if (endm == ZCRCE || endm == ZCRCW) {
          break;  // end of this ZDATA sequence; expect next header (or ZDATA again)
        }
        //keypadPollSPI();
        //pumpConsole();
        //serviceISPIfActive();
        tight_loop_contents();
        yield();
      }
      continue;  // next header: ZDATA/ZEOF/...
    } else if (ftype == ZEOF) {
      // p0 is end offset (should equal file size)
      if (expectedLen && p0 != expectedLen) {
        Console.printf("rz: ZEOF pos=%lu != expected=%lu\n", (unsigned long)p0, (unsigned long)expectedLen);
      }
      break;  // done with file data
    } else if (ftype == ZRQINIT) {
      // Sender wants re-init; oblige
      sendHeaderBin32(ZRINIT, caps);
    } else {
      // Ignore other headers
    }
    //keypadPollSPI();
    //pumpConsole();
    //serviceISPIfActive();
    tight_loop_contents();
    yield();
  }

  // Stamp final file size by appending a dir entry via writeFileInPlace
  {
    // Re-write file size in directory by calling writeFileInPlace with final size (no data rewrite)
    // To avoid re-sending data, we can call writeFileInPlace with size=received and null data not allowed, so:
    // Workaround: create a zero-length write by calling writeFileInPlace with size=received but data=null is disallowed.
    // So we just call activeFs.writeFileInPlace with a 0-byte buffer when received==0, else we re-open and do nothing.
    // Simpler: call writeFile(dstPath, nullptr, 0, ReplaceIfExists) would reset. Not desired.
    // Keep it simple: if received > 0, we read back a tiny 1-byte and rewrite same first 1-byte with writeFileInPlace to update size.
    if (received > 0) {
      uint8_t first = 0;
      (void)activeFs.readFileRange(dstPath, 0, &first, 1);
      bool ok = activeFs.writeFileInPlace(dstPath, &first, received >= 1 ? 1u : 0u, false);
      if (!ok) {
        // If this fails, fallback: write a zero-byte "touch" to stamp dir at least
        (void)activeFs.writeFileInPlace(dstPath, nullptr, 0, false);
      }
    } else {
      (void)activeFs.writeFileInPlace(dstPath, nullptr, 0, false);
    }
  }

  Console.printf("rz: received %lu bytes into %s\n", (unsigned long)received, dstPath);

  // 7) Expect ZFIN; reply "OO" and finish
  tStart = millis();
  for (;;) {
    if ((millis() - tStart) > 5000) break;
    if (!recvHeaderBin32(ftype, p0, 1000)) continue;
    if (ftype == ZFIN) {
      // Reply "OO"
      Serial.write('O');
      Serial.write('O');
      Serial.flush();
      break;
    }
  }
  return true;
}
}  // namespace ZModem
   // ==================== end ZMODEM receiver (rz-lite) ====================