// ========== Binary upload helpers (single-line puthex/putb64) ==========
static inline int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static bool decodeHexString(const char* hex, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!hex) return false;
  size_t n = strlen(hex);
  if (n == 0) return false;
  if (n & 1) {
    Console.println("puthex: error: hex string length is odd");
    return false;
  }
  uint32_t bytes = (uint32_t)(n / 2);
  uint8_t* buf = (uint8_t*)malloc(bytes);
  if (!buf) {
    Console.println("puthex: malloc failed");
    return false;
  }
  for (uint32_t i = 0; i < bytes; ++i) {
    int hi = hexVal(hex[2 * i + 0]), lo = hexVal(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) {
      Console.println("puthex: invalid hex character");
      free(buf);
      return false;
    }
    buf[i] = (uint8_t)((hi << 4) | lo);
  }
  out = buf;
  outLen = bytes;
  return true;
}
static int8_t b64Map[256];
static void initB64MapOnce() {
  static bool inited = false;
  if (inited) return;
  for (int i = 0; i < 256; ++i) b64Map[i] = -1;
  for (char c = 'A'; c <= 'Z'; ++c) b64Map[(uint8_t)c] = (int8_t)(c - 'A');
  for (char c = 'a'; c <= 'z'; c++) b64Map[(uint8_t)c] = (int8_t)(26 + (c - 'a'));
  for (char c = '0'; c <= '9'; c++) b64Map[(uint8_t)c] = (int8_t)(52 + (c - '0'));
  b64Map[(uint8_t)'+'] = 62;
  b64Map[(uint8_t)'/'] = 63;
}
static bool decodeBase64String(const char* s, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!s) return false;
  initB64MapOnce();
  size_t inLen = strlen(s);
  uint32_t outCap = (uint32_t)(((inLen + 3) / 4) * 3);
  uint8_t* buf = (uint8_t*)malloc(outCap ? outCap : 1);
  if (!buf) {
    Console.println("putb64: malloc failed");
    return false;
  }
  uint32_t o = 0;
  int vals[4];
  int vCount = 0;
  int pad = 0;
  for (size_t i = 0; i < inLen; ++i) {
    unsigned char c = (unsigned char)s[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    if (c == '=') {
      vals[vCount++] = 0;
      pad++;
    } else {
      int8_t v = b64Map[c];
      if (v < 0) {
        Console.println("putb64: invalid base64 character");
        free(buf);
        return false;
      }
      vals[vCount++] = v;
    }
    if (vCount == 4) {
      uint32_t v0 = (uint32_t)vals[0], v1 = (uint32_t)vals[1], v2 = (uint32_t)vals[2], v3 = (uint32_t)vals[3];
      uint8_t b0 = (uint8_t)((v0 << 2) | (v1 >> 4));
      uint8_t b1 = (uint8_t)(((v1 & 0x0F) << 4) | (v2 >> 2));
      uint8_t b2 = (uint8_t)(((v2 & 0x03) << 6) | v3);
      if (pad == 0) {
        buf[o++] = b0;
        buf[o++] = b1;
        buf[o++] = b2;
      } else if (pad == 1) {
        buf[o++] = b0;
        buf[o++] = b1;
      } else if (pad == 2) {
        buf[o++] = b0;
      } else {
        free(buf);
        Console.println("putb64: invalid padding");
        return false;
      }
      vCount = 0;
      pad = 0;
    }
  }
  if (vCount != 0) {
    free(buf);
    Console.println("putb64: truncated input");
    return false;
  }
  out = buf;
  outLen = o;
  return true;
}
static bool writeBinaryToFS(const char* fname, const uint8_t* data, uint32_t len) {
  if (!checkNameLen(fname)) return false;
  bool ok = activeFs.writeFile(fname, data, len, fsReplaceMode());
  return ok;
}

// ========== Streaming Base64 upload (clipboard-friendly, non-blocking, bracketed paste aware) ==========
enum EscMode : uint8_t { EM_None = 0,
                         EM_Esc,
                         EM_CSI,
                         EM_OSC,
                         EM_OSC_Esc };

struct B64UploadState {
  bool active = false;
  char fname[ActiveFS::MAX_NAME + 1] = { 0 };

  // Decoded data buffer (grow as needed; for really large files you could add chunked FS writes)
  uint8_t* buf = nullptr;
  uint32_t size = 0;
  uint32_t cap = 0;
  uint32_t expected = 0;

  // Base64 quartet state
  int8_t q[4];
  uint8_t qn = 0;
  uint8_t pad = 0;

  // Line terminator detection (only when not in bracketed paste)
  bool lineAtStart = true;
  bool lineOnlyDot = true;
  bool hadAnyData = false;

  // Escape / terminal control parsing (non-blocking)
  EscMode escMode = EM_None;
  int csiNum = -1;              // parsed CSI numeric parameter (e.g., 200, 201)
  bool bracketEnabled = false;  // we asked terminal to enable bracketed paste
  bool bracketActive = false;   // we observed ESC[200~; will expect ESC[201~ to finish
};

static B64UploadState g_b64u;

static bool b64uEnsure(uint32_t need) {
  if (g_b64u.cap >= need) return true;
  uint32_t newCap = g_b64u.cap ? g_b64u.cap : 4096;
  while (newCap < need) {
    uint32_t next = newCap * 2;
    if (next <= newCap) {
      newCap = need;
      break;
    }  // clamp on overflow
    newCap = next;
  }
  uint8_t* nb = (uint8_t*)realloc(g_b64u.buf, newCap);
  if (!nb) return false;
  g_b64u.buf = nb;
  g_b64u.cap = newCap;
  return true;
}
static bool b64uEmit(uint8_t b) {
  if (!b64uEnsure(g_b64u.size + 1)) return false;
  g_b64u.buf[g_b64u.size++] = b;
  return true;
}
static bool b64uEmit3(uint8_t b0, uint8_t b1, uint8_t b2, int emitCount) {
  if (!b64uEnsure(g_b64u.size + (uint32_t)emitCount)) return false;
  if (emitCount >= 1) g_b64u.buf[g_b64u.size++] = b0;
  if (emitCount >= 2) g_b64u.buf[g_b64u.size++] = b1;
  if (emitCount >= 3) g_b64u.buf[g_b64u.size++] = b2;
  return true;
}
static bool b64uFlushQuartet(bool finalFlush, const char*& err) {
  if (g_b64u.qn == 0) return true;
  if (!finalFlush && g_b64u.qn < 4) return true;

  if (finalFlush && (g_b64u.qn == 2 || g_b64u.qn == 3)) {
    uint32_t v0 = (uint32_t)g_b64u.q[0], v1 = (uint32_t)g_b64u.q[1];
    uint8_t b0 = (uint8_t)((v0 << 2) | (v1 >> 4));
    if (g_b64u.qn == 2) {
      g_b64u.qn = 0;
      g_b64u.pad = 0;
      return b64uEmit(b0) ? true : (err = "putb64s: OOM", false);
    }
    uint32_t v2 = (uint32_t)g_b64u.q[2];
    uint8_t b1 = (uint8_t)(((v1 & 0x0F) << 4) | (v2 >> 2));
    g_b64u.qn = 0;
    g_b64u.pad = 0;
    return b64uEmit3(b0, b1, 0, 2) ? true : (err = "putb64s: OOM", false);
  }

  if (g_b64u.qn != 4) {
    err = "putb64s: truncated input";
    return false;
  }
  uint32_t v0 = (uint32_t)g_b64u.q[0], v1 = (uint32_t)g_b64u.q[1];
  uint32_t v2 = (uint32_t)g_b64u.q[2], v3 = (uint32_t)g_b64u.q[3];
  uint8_t b0 = (uint8_t)((v0 << 2) | (v1 >> 4));
  uint8_t b1 = (uint8_t)(((v1 & 0x0F) << 4) | (v2 >> 2));
  uint8_t b2 = (uint8_t)(((v2 & 0x03) << 6) | v3);
  int emit = 3;
  if (g_b64u.pad == 1) emit = 2;
  else if (g_b64u.pad == 2) emit = 1;
  else if (g_b64u.pad > 2) {
    err = "putb64s: invalid padding";
    return false;
  }
  g_b64u.qn = 0;
  g_b64u.pad = 0;
  if (!b64uEmit3(b0, b1, b2, emit)) {
    err = "putb64s: OOM";
    return false;
  }
  return true;
}

static void b64uDisableBracketedPaste() {
  if (g_b64u.bracketEnabled) {
    // disable bracketed paste in terminal
    Serial.print("\x1b[?2004l");
    g_b64u.bracketEnabled = false;
  }
}

static void b64uAbort(const char* why) {
  b64uDisableBracketedPaste();
  Console.print("putb64s: aborted: ");
  Console.println(why ? why : "error");
  if (g_b64u.buf) {
    free(g_b64u.buf);
    g_b64u.buf = nullptr;
  }
  g_b64u = B64UploadState{};
  Console.print("> ");
}

static void b64uComplete() {
  const char* err = nullptr;
  if (!b64uFlushQuartet(/*final*/ true, err)) {
    b64uAbort(err);
    return;
  }
  b64uDisableBracketedPaste();
  // Optional size check
  if (g_b64u.expected && g_b64u.size != g_b64u.expected) {
    Console.print("putb64s: warning: decoded ");
    Console.print(g_b64u.size);
    Console.print(" bytes, expected ");
    Console.println(g_b64u.expected);
  }
  bool ok = writeBinaryToFS(g_b64u.fname, g_b64u.buf, g_b64u.size);
  if (g_b64u.buf) {
    free(g_b64u.buf);
    g_b64u.buf = nullptr;
  }
  if (ok) {
    Console.print("putb64s: wrote ");
    Console.print(g_b64u.size);
    Console.print(" bytes to ");
    Console.println(g_b64u.fname);
  } else {
    Console.println("putb64s: write failed");
  }
  g_b64u = B64UploadState{};
  Console.print("> ");
}

static void b64uStart(const char* fname, uint32_t expected) {
  memset(&g_b64u, 0, sizeof(g_b64u));
  g_b64u.active = true;
  strncpy(g_b64u.fname, fname, ActiveFS::MAX_NAME);
  g_b64u.fname[ActiveFS::MAX_NAME] = 0;
  g_b64u.expected = expected;
  if (expected) (void)b64uEnsure(expected);
  initB64MapOnce();

  // Enable bracketed paste so terminals wrap clipboard paste with ESC[200~ ... ESC[201~.
  // This won’t affect simple serial monitors, but it helps modern terminals.
  Serial.print("\x1b[?2004h");
  g_b64u.bracketEnabled = true;

  Console.println("putb64s: paste base64 now.");
  Console.println(" - If your terminal supports bracketed paste, just paste; it will auto-detect end.");
  Console.println(" - Otherwise, finish with Ctrl-D (EOT) or with a line containing only a single dot '.'");
}

// Non-blocking pump: consumes available bytes, handles bracketed paste, CSI/OSC, Ctrl-D, and '.' line
static void b64uPump() {
  const char* err = nullptr;
  while (Serial.available()) {
    int iv = Serial.read();
    if (iv < 0) break;
    unsigned char c = (unsigned char)iv;

    // Ctrl-D (EOT) always ends upload (works in many terminals)
    if (c == 0x04) {
      b64uComplete();
      return;
    }

    // CR/LF for line handling only if not inside bracketed paste
    if (!g_b64u.bracketActive) {
      if (c == '\r') continue;
      if (c == '\n') {
        if (g_b64u.lineOnlyDot) {
          // dot on its own line ends upload
          b64uComplete();
          return;
        }
        g_b64u.lineAtStart = true;
        g_b64u.lineOnlyDot = true;
        continue;
      }
    }

    // Software flow control noise
    if (c == 0x11 /*XON*/ || c == 0x13 /*XOFF*/) continue;

    // Escape/terminal control parsing (non-blocking)
    if (g_b64u.escMode != EM_None) {
      // We are inside some escape handling
      if (g_b64u.escMode == EM_Esc) {
        if (c == '[') {
          g_b64u.escMode = EM_CSI;
          g_b64u.csiNum = -1;
          continue;
        }
        if (c == ']') {
          g_b64u.escMode = EM_OSC;
          continue;
        }
        // Unknown ESC + X: ignore
        g_b64u.escMode = EM_None;
        continue;
      } else if (g_b64u.escMode == EM_CSI) {
        if (c >= '0' && c <= '9') {
          if (g_b64u.csiNum < 0) g_b64u.csiNum = 0;
          g_b64u.csiNum = g_b64u.csiNum * 10 + (c - '0');
          continue;
        }
        if (c == '~') {
          if (g_b64u.csiNum == 200) {
            g_b64u.bracketActive = true;  // start-of-bracketed paste
          } else if (g_b64u.csiNum == 201) {
            // end-of-bracketed paste: complete immediately
            b64uComplete();
            return;
          }
          g_b64u.escMode = EM_None;
          g_b64u.csiNum = -1;
          continue;
        }
        // Any other final byte ends CSI
        if (c >= 0x40 && c <= 0x7E) {
          g_b64u.escMode = EM_None;
          g_b64u.csiNum = -1;
          continue;
        }
        // Else keep consuming CSI parameters
        continue;
      } else if (g_b64u.escMode == EM_OSC) {
        if (c == 0x07) {
          g_b64u.escMode = EM_None;
          continue;
        }  // BEL ends OSC
        if (c == 0x1B) {
          g_b64u.escMode = EM_OSC_Esc;
          continue;
        }          // ESC \ ends OSC
        continue;  // swallow OSC content
      } else if (g_b64u.escMode == EM_OSC_Esc) {
        if (c == '\\') {
          g_b64u.escMode = EM_None;
          continue;
        }  // ST = ESC \
        g_b64u.escMode = EM_OSC;                                     // otherwise keep swallowing
        continue;
      }
    } else {
      if (c == 0x1B) {
        g_b64u.escMode = EM_Esc;
        continue;
      }
    }

    // Only track dot-line termination when not in bracketed paste
    if (!g_b64u.bracketActive) {
      if (g_b64u.lineAtStart) {
        if (c == '.') {
          // keep lineOnlyDot = true unless other non-space follows
        } else if (c != ' ' && c != '\t') {
          g_b64u.lineOnlyDot = false;
        }
        g_b64u.lineAtStart = false;
      } else {
        if (c != ' ' && c != '\t') g_b64u.lineOnlyDot = false;
      }
    }

    // Ignore whitespace (safe for both wrapped and unwrapped base64)
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;

    // Base64
    if (c == '=') {
      if (g_b64u.qn >= 4) {
        b64uAbort("putb64s: quartet overflow");
        return;
      }
      g_b64u.q[g_b64u.qn++] = 0;
      g_b64u.pad++;
    } else {
      int8_t v = b64Map[c];
      if (v < 0) {
        // Ignore any other non-base64 glyphs (robust against stray terminal noise)
        continue;
      }
      g_b64u.q[g_b64u.qn++] = v;
    }
    if (g_b64u.qn == 4) {
      if (!b64uFlushQuartet(/*final*/ false, err)) {
        b64uAbort(err);
        return;
      }
      g_b64u.hadAnyData = true;
    }
  }
}
