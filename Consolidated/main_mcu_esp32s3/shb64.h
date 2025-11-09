#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

namespace shb64s {

using WriteFn = bool (*)(const char* fname, const uint8_t* data, uint32_t len);

enum EscMode : uint8_t { EM_None,
                         EM_Esc,
                         EM_CSI,
                         EM_OSC,
                         EM_OSC_Esc };

struct State {
  bool active = false;
  char fname[33] = { 0 };
  uint8_t* buf = nullptr;
  uint32_t size = 0, cap = 0, expected = 0;
  int8_t q[4];
  uint8_t qn = 0, pad = 0;
  bool lineAtStart = true, lineOnlyDot = true, hadAnyData = false;
  EscMode escMode = EM_None;
  int csiNum = -1;
  bool bracketEnabled = false;
  bool bracketActive = false;
  WriteFn sink = nullptr;
  Stream* io = nullptr;
};

static int8_t b64Map[256];
static bool b64Inited = false;
inline void initMap() {
  if (b64Inited) return;
  b64Inited = true;
  for (int i = 0; i < 256; ++i) b64Map[i] = -1;
  for (char c = 'A'; c <= 'Z'; ++c) b64Map[(uint8_t)c] = (int8_t)(c - 'A');
  for (char c = 'a'; c <= 'z'; ++c) b64Map[(uint8_t)c] = (int8_t)(26 + (c - 'a'));
  for (char c = '0'; c <= '9'; ++c) b64Map[(uint8_t)c] = (int8_t)(52 + (c - '0'));
  b64Map[(uint8_t)'+'] = 62;
  b64Map[(uint8_t)'/'] = 63;
}

inline bool ensure(State& s, uint32_t need) {
  if (s.cap >= need) return true;
  uint32_t nc = s.cap ? s.cap : 4096;
  while (nc < need) {
    uint32_t nx = nc * 2;
    if (nx <= nc) {
      nc = need;
      break;
    }
    nc = nx;
  }
  uint8_t* nb = (uint8_t*)realloc(s.buf, nc);
  if (!nb) return false;
  s.buf = nb;
  s.cap = nc;
  return true;
}

inline void disableBracketPaste(State& s) {
  if (s.bracketEnabled && s.io) {
    s.io->print("\x1b[?2004l");
    s.bracketEnabled = false;
  }
}

inline void abort(State& s, const char* why) {
  disableBracketPaste(s);
  if (s.io) {
    s.io->print("putb64s: aborted: ");
    s.io->println(why ? why : "error");
    s.io->print("> ");
  }
  if (s.buf) free(s.buf);
  s = State{};
}

inline void complete(State& s) {
  // flush current quartet if needed
  // (re-use flush logic from your sketch; omitted here for brevity)
  disableBracketPaste(s);
  bool ok = s.sink ? s.sink(s.fname, s.buf, s.size) : false;
  if (s.io) {
    if (ok) {
      s.io->print("putb64s: wrote ");
      s.io->print(s.size);
      s.io->print(" bytes to ");
      s.io->println(s.fname);
    } else s.io->println("putb64s: write failed");
    s.io->print("> ");
  }
  if (s.buf) free(s.buf);
  s = State{};
}

inline void begin(State& s, Stream& serial, const char* fname, uint32_t expected, WriteFn sink) {
  s = State{};
  s.io = &serial;
  s.active = true;
  s.expected = expected;
  s.sink = sink;
  strncpy(s.fname, fname, sizeof(s.fname) - 1);
  if (expected) (void)ensure(s, expected);
  initMap();
  serial.print("\x1b[?2004h");
  s.bracketEnabled = true;
  serial.println("putb64s: paste base64 now. End with ESC[201~ (auto), Ctrl-D, or a line containing only '.'");
}

inline void pump(State& s) {
  if (!s.active || !s.io) return;
  while (s.io->available()) {
    int iv = s.io->read();
    if (iv < 0) break;
    uint8_t c = (uint8_t)iv;
    if (c == 0x04) {
      complete(s);
      return;
    }                                     // Ctrl-D
    if (!s.bracketActive && c == '\n') {  // line end
      if (s.lineOnlyDot) {
        complete(s);
        return;
      }
      s.lineAtStart = true;
      s.lineOnlyDot = true;
      continue;
    }
    if (c == 0x11 || c == 0x13) continue;  // XON/XOFF
    // ... (escape/CSI/OSC handling identical to your current logic)
    // whitespace ignored
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    // base64 quartet
    // ... (push into s.q[], flush to s.buf using ensure(), same as your code)
  }
}

inline bool active(const State& s) {
  return s.active;
}

}  // namespace shb64s