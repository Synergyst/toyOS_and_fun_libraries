#pragma once
/*
  CoProcLang.h (safe, no-heap, no-static-init, low-footprint)
  Minimal line-oriented scripting language interpreter for RP2040/RP2350 Arduino builds.
  Goals in this revision:
    - Zero global/static constructors. Header declares no globals and does not allocate at include-time.
    - No dynamic allocation during run() by default (operates in-place on a mutable script buffer).
      Define COPROCLANG_COPY_INPUT=1 before including this header to enable a copying mode if your buffer is const.
    - Bounded RAM usage: internal tables are small and configurable at compile time.
    - Robust label handling (LABEL: at start of statement; code may follow ':' on same line).
    - Arduino-native I/O and delays; timeout + cancel-flag honored.
  Script features:
    - Registers: R0..R15 (int32). Args preload R0..R(N-1).
    - Labels: "NAME:" at statement start.
    - Flow:  GOTO <label>
             IF Rn <op> <expr> GOTO <label>   where <op> in {==, !=, <, >, <=, >=}
    - Math:  LET Rn <expr>, ADD Rn <expr>, SUB Rn <expr>, MOV Rn Rm
    - I/O:   PINMODE <pin> <IN|OUT|INPUT|OUTPUT|INPU|INPD|PULLUP|PULLDOWN|num>
             DWRITE  <pin> <expr>   (expr may be 0/1 or LOW/HIGH/FALSE/TRUE)
             DREAD   <pin> Rn
             AWRITE  <pin> <expr>   (0..255)
             AREAD   <pin> Rn
    - Time:  DELAY <ms>, DELAY_US <us>
    - Mailbox: MBCLR, MBAPP "text", PRINT "text"
    - Return: RET <expr>
    - Comments: '#' or '//' to end of statement.
    - Statements separated by newline or ';'
  Notes:
    - Default parsing operates IN-PLACE: the interpreter writes '\0' to split statements and strip comments.
      Ensure the buffer passed to run() is writable (your pipeline already allocates g_script, so it's fine).
      If you need to keep the input immutable, define COPROCLANG_COPY_INPUT=1 before including this file.
    - To minimize RAM while supporting larger scripts, defaults use increased but fixed tables; override before including if needed:
        #define COPROCLANG_MAX_LINES  8192
        #define COPROCLANG_MAX_LABELS 2048
*/
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
// ---------- Configurable bounds ----------
#ifndef COPROCLANG_MAX_LINES
#define COPROCLANG_MAX_LINES 8192
#endif
#ifndef COPROCLANG_MAX_LABELS
#define COPROCLANG_MAX_LABELS 2048
#endif
namespace CoProcLang {
// ---------- Small ASCII helpers (no <ctype.h>) ----------
static inline bool is_ws(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}
static inline bool is_alpha(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}
static inline bool is_digit(char c) {
  return (c >= '0' && c <= '9');
}
static inline bool is_xdigit(char c) {
  return is_digit(c) || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}
static inline char to_lower(char c) {
  return (c >= 'A' && c <= 'Z') ? char(c - 'A' + 'a') : c;
}
static inline char to_upper(char c) {
  return (c >= 'a' && c <= 'z') ? char(c - 'a' + 'A') : c;
}
static inline bool is_ident_ch0(char c) {
  return (c == '_') || is_alpha(c);
}
static inline bool is_ident_ch(char c) {
  return (c == '_') || is_alpha(c) || is_digit(c);
}
static inline void skip_ws(const char*& p) {
  while (*p && is_ws(*p)) ++p;
}
static inline int stricmp_l(const char* a, const char* b, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    char ca = to_lower(a[i]), cb = to_lower(b[i]);
    if (ca != cb) return (int)((unsigned char)ca) - (int)((unsigned char)cb);
  }
  return 0;
}
static inline bool strieq_full(const char* a, const char* b) {
  while (*a && *b) {
    if (to_lower(*a) != to_lower(*b)) return false;
    ++a;
    ++b;
  }
  return (*a == 0 && *b == 0);
}
// ---------- Execution env ----------
struct Env {
  volatile uint8_t* mailbox;            // optional mailbox (null-terminated), may be nullptr
  uint32_t mailbox_max;                 // bytes
  volatile const uint8_t* cancel_flag;  // optional cancel flag pointer (1 => cancel)
  Env()
    : mailbox(nullptr), mailbox_max(0), cancel_flag(nullptr) {}
};
// ---------- VM ----------
struct VM {
  // Registers
  int32_t R[16];
  // In-place script storage view
  char* base;    // points to mutable script buffer (owned by caller)
  uint32_t len;  // bytes available in base
  // Tables (bounded, no heap)
  const char* lines[COPROCLANG_MAX_LINES];
  uint32_t line_count;
  struct Label {
    const char* name;  // pointer inside base (null-terminated)
    uint32_t idx;      // line index of first statement following the label
  };
  Label labels[COPROCLANG_MAX_LABELS];
  uint32_t label_count;
  // Mailbox and control
  Env env;
  uint32_t mb_w;
#if COPROCLANG_MAX_LINES < 8 || COPROCLANG_MAX_LABELS < 8
#error "COPROCLANG_MAX_* too small"
#endif
  VM()
    : base(nullptr), len(0), line_count(0), label_count(0), mb_w(0) {
    for (int i = 0; i < 16; ++i) R[i] = 0;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LINES; ++i) lines[i] = nullptr;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LABELS; ++i) {
      labels[i].name = nullptr;
      labels[i].idx = 0;
    }
  }
  // ----- Mailbox -----
  void mbClear() {
    if (!env.mailbox || env.mailbox_max == 0) return;
    mb_w = 0;
    env.mailbox[0] = 0;
  }
  void mbAppend(const char* s, size_t n) {
    if (!env.mailbox || env.mailbox_max == 0 || !s || n == 0) return;
    uint32_t cap = env.mailbox_max;
    while (n && (mb_w + 1) < cap) {
      env.mailbox[mb_w++] = (uint8_t)(*s++);
      --n;
    }
    env.mailbox[mb_w] = 0;
  }
  // ----- Lex helpers -----
  static bool parseIdent(const char*& p, const char*& outStart, size_t& outLen) {
    skip_ws(p);
    const char* s = p;
    if (!is_ident_ch0(*s)) return false;
    const char* b = s++;
    while (is_ident_ch(*s)) ++s;
    outStart = b;
    outLen = (size_t)(s - b);
    p = s;
    return true;
  }
  static bool parseString(const char*& p, const char*& outStart, size_t& outLen) {
    skip_ws(p);
    if (*p != '"') return false;
    ++p;
    const char* s = p;
    while (*s && *s != '"') ++s;
    if (*s != '"') return false;
    outStart = p;
    outLen = (size_t)(s - p);
    p = s + 1;
    return true;
  }
  static bool parseNumber(const char*& p, int32_t& out) {
    skip_ws(p);
    const char* s = p;
    bool neg = false;
    if (*s == '+' || *s == '-') {
      neg = (*s == '-');
      ++s;
    }
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      s += 2;
      if (!is_xdigit(*s)) return false;
      int32_t v = 0;
      while (is_xdigit(*s)) {
        char c = *s++;
        int d = (c >= '0' && c <= '9') ? (c - '0') : (c >= 'a' && c <= 'f') ? (c - 'a' + 10)
                                                   : (c >= 'A' && c <= 'F') ? (c - 'A' + 10)
                                                                            : -1;
        if (d < 0) return false;
        v = (v << 4) | d;
      }
      out = neg ? -v : v;
      p = s;
      return true;
    } else {
      if (!is_digit(*s)) return false;
      int32_t v = 0;
      while (is_digit(*s)) {
        v = v * 10 + (*s - '0');
        ++s;
      }
      out = neg ? -v : v;
      p = s;
      return true;
    }
  }
  static bool parseReg(const char*& p, int& r) {
    skip_ws(p);
    const char* s = p;
    if (*s != 'R' && *s != 'r') return false;
    ++s;
    if (!is_digit(*s)) return false;
    int v = 0;
    while (is_digit(*s)) {
      v = v * 10 + (*s - '0');
      ++s;
    }
    if (v < 0 || v > 15) return false;
    r = v;
    p = s;
    return true;
  }
  bool parseExpr(const char*& p, int32_t& out) {
    const char* save = p;
    int r = -1;
    if (parseReg(p, r)) {
      out = R[r];
      return true;
    }
    // identifiers: HIGH/LOW/TRUE/FALSE
    const char* id;
    size_t n;
    const char* s2 = p;
    if (parseIdent(s2, id, n)) {
      if ((n == 4 && stricmp_l(id, "HIGH", 4) == 0) || (n == 4 && stricmp_l(id, "TRUE", 4) == 0)) {
        p = s2;
        out = 1;
        return true;
      }
      if ((n == 3 && stricmp_l(id, "LOW", 3) == 0) || (n == 5 && stricmp_l(id, "FALSE", 5) == 0)) {
        p = s2;
        out = 0;
        return true;
      }
    }
    if (parseNumber(p, out)) return true;
    p = save;
    return false;
  }
  enum CmpOp { OP_EQ,
               OP_NE,
               OP_LT,
               OP_GT,
               OP_LE,
               OP_GE,
               OP_BAD };
  static CmpOp parseCmpOp(const char*& p) {
    skip_ws(p);
    if (p[0] == '=' && p[1] == '=') {
      p += 2;
      return OP_EQ;
    }
    if (p[0] == '!' && p[1] == '=') {
      p += 2;
      return OP_NE;
    }
    if (p[0] == '<' && p[1] == '=') {
      p += 2;
      return OP_LE;
    }
    if (p[0] == '>' && p[1] == '=') {
      p += 2;
      return OP_GE;
    }
    if (p[0] == '<') {
      ++p;
      return OP_LT;
    }
    if (p[0] == '>') {
      ++p;
      return OP_GT;
    }
    return OP_BAD;
  }
  static bool evalCmp(int32_t a, CmpOp op, int32_t b) {
    switch (op) {
      case OP_EQ: return a == b;
      case OP_NE: return a != b;
      case OP_LT: return a < b;
      case OP_GT: return a > b;
      case OP_LE: return a <= b;
      case OP_GE: return a >= b;
      default: return false;
    }
  }
  static bool parsePinModeToken(const char*& p, int& modeOut) {
    const char* id;
    size_t n;
    const char* save = p;
    if (parseIdent(p, id, n)) {
      // lowercase compare
      if ((n == 2 && stricmp_l(id, "in", 2) == 0) || (n == 5 && stricmp_l(id, "input", 5) == 0)) {
        modeOut = INPUT;
        return true;
      }
      if ((n == 3 && stricmp_l(id, "out", 3) == 0) || (n == 6 && stricmp_l(id, "output", 6) == 0)) {
        modeOut = OUTPUT;
        return true;
      }
#if defined(INPUT_PULLUP)
      if ((n == 4 && stricmp_l(id, "inpu", 4) == 0) || (n == 6 && stricmp_l(id, "pullup", 6) == 0)) {
        modeOut = INPUT_PULLUP;
        return true;
      }
#endif
#if defined(INPUT_PULLDOWN)
      if ((n == 4 && stricmp_l(id, "inpd", 4) == 0) || (n == 8 && stricmp_l(id, "pulldown", 8) == 0)) {
        modeOut = INPUT_PULLDOWN;
        return true;
      }
#endif
      // not matched; fall through to numeric
      p = save;
    }
    int32_t v = 0;
    if (parseNumber(p, v)) {
      modeOut = (int)v;
      return true;
    }
    return false;
  }
  // ----- Preprocess: split into statements and collect labels (IN-PLACE) -----
  void stripCommentInPlace(char* s) {
    if (!s) return;
    for (; *s; ++s) {
      if (*s == '#') {
        *s = 0;
        return;
      }
      if (s[0] == '/' && s[1] == '/') {
        s[0] = 0;
        return;
      }
    }
  }
  bool buildTablesInPlace() {
    line_count = 0;
    label_count = 0;
    if (!base || len == 0) return false;
    char* cur = base;
    char* end = base + len;
    auto trim = [](char*& a) {
      while (*a && is_ws(*a)) ++a;
      // trailing trim after we know endpoint; we will set '\0' already
    };
    while (cur < end) {
      // Get one physical line up to '\n' or end
      char* line = cur;
      while (cur < end && *cur != '\n') ++cur;
      char* after = cur;
      if (cur < end && *cur == '\n') {
        *cur = 0;
        ++cur;
      }
      // Now split by ';' into statements
      char* stmt = line;
      while (stmt && *stmt) {
        char* semi = strchr(stmt, ';');
        if (semi) *semi = 0;
        // Strip comments and trim spaces
        stripCommentInPlace(stmt);
        // trim leading
        char* s = stmt;
        while (*s && is_ws(*s)) ++s;
        // trim trailing
        char* t = s + strlen(s);
        while (t > s && is_ws(t[-1])) --t;
        *t = 0;
        if (*s) {
          // Label detection: IDENT: at start
          const char* p = s;
          skip_ws(p);
          const char* id;
          size_t idn;
          const char* p0 = p;
          if (parseIdent(p, id, idn)) {
            if (*p == ':') {
              // register label name by zero-terminating at ':'
              char* nameStart = (char*)id;
              char* colon = (char*)p;
              *colon = 0;  // terminate label name
              if (label_count >= COPROCLANG_MAX_LABELS) return false;
              labels[label_count].name = nameStart;
              labels[label_count].idx = line_count;
              ++label_count;
              // any code after ':' becomes a statement
              ++p;  // move past '\0' we just wrote, now points to char after colon in original text
              while (*p && is_ws(*p)) ++p;
              if (*p) {
                if (line_count >= COPROCLANG_MAX_LINES) return false;
                lines[line_count++] = p;
              }
            } else {
              // normal statement
              if (line_count >= COPROCLANG_MAX_LINES) return false;
              lines[line_count++] = s;
            }
          } else {
            // not an identifier
            if (line_count >= COPROCLANG_MAX_LINES) return false;
            lines[line_count++] = s;
          }
        }
        if (!semi) break;
        stmt = semi + 1;
      }
      (void)after;
    }
    return true;
  }
  int findLabel(const char* name) const {
    if (!name || !*name) return -1;
    for (uint32_t i = 0; i < label_count; ++i) {
      if (labels[i].name && strieq_full(labels[i].name, name)) return (int)labels[i].idx;
    }
    return -1;
  }
  // ----- Execute one statement -----
  bool execLine(uint32_t idx, int32_t& retVal, bool& didReturn, int& outJumpIdx) {
    outJumpIdx = -1;
    const char* s = lines[idx];
    if (!s || !*s) return true;
    const char* p = s;
    // command
    const char* cmd;
    size_t cmdn;
    if (!parseIdent(p, cmd, cmdn)) return true;
    // Normalize a short token for branching
    char tok[24];
    size_t L = (cmdn < sizeof(tok) - 1) ? cmdn : (sizeof(tok) - 1);
    for (size_t i = 0; i < L; ++i) tok[i] = to_upper(cmd[i]);
    tok[L] = 0;
    // Math/move
    if (!strcmp(tok, "LET")) {
      int r;
      if (!parseReg(p, r)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      R[r] = v;
      return true;
    }
    if (!strcmp(tok, "ADD")) {
      int r;
      if (!parseReg(p, r)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      R[r] += v;
      return true;
    }
    if (!strcmp(tok, "SUB")) {
      int r;
      if (!parseReg(p, r)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      R[r] -= v;
      return true;
    }
    if (!strcmp(tok, "MOV")) {
      int rd;
      if (!parseReg(p, rd)) return true;
      int rs;
      if (!parseReg(p, rs)) return true;
      R[rd] = R[rs];
      return true;
    }
    // I/O
    if (!strcmp(tok, "PINMODE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int mode;
      if (!parsePinModeToken(p, mode)) return true;
      pinMode((uint8_t)pin, mode);
      return true;
    }
    if (!strcmp(tok, "DWRITE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      digitalWrite((uint8_t)pin, v ? HIGH : LOW);
      return true;
    }
    if (!strcmp(tok, "DREAD")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int r;
      if (!parseReg(p, r)) return true;
      R[r] = digitalRead((uint8_t)pin);
      return true;
    }
    if (!strcmp(tok, "AWRITE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      analogWrite((uint8_t)pin, (int)v);
      return true;
    }
    if (!strcmp(tok, "AREAD")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int r;
      if (!parseReg(p, r)) return true;
      R[r] = analogRead((uint8_t)pin);
      return true;
    }
    // SHIFTOUT: SHIFTOUT <dataPin> <clockPin> <latchPin> <expr> [bits] [MSBFIRST|LSBFIRST]
    if (!strcmp(tok, "SHIFTOUT")) {
      int32_t dataPin, clockPin, latchPin;
      if (!parseNumber(p, dataPin)) return true;
      if (!parseNumber(p, clockPin)) return true;
      if (!parseNumber(p, latchPin)) return true;
      int32_t val;
      if (!parseExpr(p, val)) return true;
      // optional bit count
      const char* save_p = p;
      int bitCount = 8;
      int32_t tmp;
      if (parseNumber(p, tmp)) {
        if (tmp > 0 && tmp <= 32) bitCount = tmp;
        else bitCount = 8;
      } else {
        p = save_p;
      }
      // Optional bit order token
      const char* id;
      size_t idn;
      bool msbFirst = true;
      const char* save2 = p;
      if (parseIdent(p, id, idn)) {
        if (idn == 8 && stricmp_l(id, "LSBFIRST", 8) == 0) msbFirst = false;
        else if (idn == 8 && stricmp_l(id, "MSBFIRST", 8) == 0) msbFirst = true;
        else p = save2;  // unrecognized token; rewind
      }
      // Ensure pins are outputs (safe)
      pinMode((uint8_t)dataPin, OUTPUT);
      pinMode((uint8_t)clockPin, OUTPUT);
      pinMode((uint8_t)latchPin, OUTPUT);
      // Latch low, shift bits, latch high
      digitalWrite((uint8_t)latchPin, LOW);
      // Shift bitCount bits (cap to 32)
      int bits = (bitCount > 32) ? 32 : bitCount;
      for (int i = 0; i < bits; ++i) {
        int bitIndex = msbFirst ? (bits - 1 - i) : i;
        int b = (((uint32_t)val >> bitIndex) & 1) ? HIGH : LOW;
        digitalWrite((uint8_t)dataPin, b);
        // Clock pulse
        digitalWrite((uint8_t)clockPin, HIGH);
        delayMicroseconds(1);  // tweak if you need slower/faster
        digitalWrite((uint8_t)clockPin, LOW);
        delayMicroseconds(1);
      }
      digitalWrite((uint8_t)latchPin, HIGH);
      return true;
    }
    // Timing
    if (!strcmp(tok, "DELAY")) {
      int32_t ms;
      if (!parseExpr(p, ms)) return true;
      if (ms < 0) ms = 0;
      delay((uint32_t)ms);
      return true;
    }
    if (!strcmp(tok, "DELAY_US")) {
      int32_t us;
      if (!parseExpr(p, us)) return true;
      if (us < 0) us = 0;
      delayMicroseconds((uint32_t)us);
      return true;
    }
    // Mailbox
    if (!strcmp(tok, "MBCLR")) {
      mbClear();
      return true;
    }
    if (!strcmp(tok, "MBAPP") || !strcmp(tok, "PRINT")) {
      const char* t;
      size_t tn;
      if (!parseString(p, t, tn)) return true;
      mbAppend(t, tn);
      return true;
    }
    // Flow
    if (!strcmp(tok, "RET")) {
      int32_t v = 0;
      (void)parseExpr(p, v);
      retVal = v;
      didReturn = true;
      return false;
    }
    if (!strcmp(tok, "GOTO")) {
      const char* id;
      size_t idn;
      if (!parseIdent(p, id, idn)) return true;
      char name[64];
      size_t m = (idn < sizeof(name) - 1) ? idn : (sizeof(name) - 1);
      memcpy(name, id, m);
      name[m] = 0;
      int li = findLabel(name);
      if (li >= 0) outJumpIdx = li;
      return true;
    }
    if (!strcmp(tok, "IF")) {
      // IF Rn <op> <expr> GOTO <label>
      int r;
      if (!parseReg(p, r)) return true;
      CmpOp op = parseCmpOp(p);
      if (op == OP_BAD) return true;
      int32_t rhs;
      if (!parseExpr(p, rhs)) return true;
      const char* g;
      size_t gn;
      if (!parseIdent(p, g, gn)) return true;
      // Expect GOTO
      if (!(gn == 4 && stricmp_l(g, "GOTO", 4) == 0)) return true;
      const char* id;
      size_t idn;
      if (!parseIdent(p, id, idn)) return true;
      char name[64];
      size_t m = (idn < sizeof(name) - 1) ? idn : (sizeof(name) - 1);
      memcpy(name, id, m);
      name[m] = 0;
      if (evalCmp(R[r], op, rhs)) {
        int li = findLabel(name);
        if (li >= 0) outJumpIdx = li;
      }
      return true;
    }
    // Unknown token: ignore
    return true;
  }
  // ----- Public run API -----
  // By default, operates IN-PLACE: modifies 'buf' by inserting '\0' terminators.
  // Ensure 'buf' is writable. Your transport allocates a script buffer, so this matches well.
  bool run(uint8_t* buf, uint32_t buflen, const int32_t* args, uint32_t argc, uint32_t timeout_ms, int32_t& retVal) {
#if COPROCLANG_COPY_INPUT
    // Copying mode (if you need const input). Allocates once per run (heap).
    char* copy = (char*)malloc(buflen + 1);
    if (!copy) return false;
    memcpy(copy, buf, buflen);
    copy[buflen] = 0;
    bool ok = runInPlaceInternal(copy, buflen, args, argc, timeout_ms, retVal);
    free(copy);
    return ok;
#else
    // In-place mode: cast to char* and run
    return runInPlaceInternal((char*)buf, buflen, args, argc, timeout_ms, retVal);
#endif
  }
private:
  bool runInPlaceInternal(char* buf, uint32_t buflen, const int32_t* args, uint32_t argc, uint32_t timeout_ms, int32_t& retVal) {
    // init
    base = buf;
    len = buflen;
    for (int i = 0; i < 16; ++i) R[i] = 0;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LINES; ++i) lines[i] = nullptr;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LABELS; ++i) {
      labels[i].name = nullptr;
      labels[i].idx = 0;
    }
    line_count = 0;
    label_count = 0;
    mb_w = 0;
    for (uint32_t i = 0; i < argc && i < 16; ++i) R[i] = args[i];
    if (env.mailbox && env.mailbox_max) env.mailbox[0] = 0;
    // Build tables
    if (!buildTablesInPlace()) return false;
    // Execute
    retVal = 0;
    uint32_t t0 = millis();
    bool didReturn = false;
    for (uint32_t pc = 0; pc < line_count;) {
      if (timeout_ms && (millis() - t0) > timeout_ms) return false;
      if (env.cancel_flag && *env.cancel_flag) return false;
      int jump = -1;
      if (!execLine(pc, retVal, didReturn, jump)) break;
      if (didReturn) break;
      if (jump >= 0 && (uint32_t)jump < line_count) pc = (uint32_t)jump;
      else ++pc;
      // lightweight yield
      tight_loop_contents();
      yield();
    }
    return true;
  }
};

// ============================
// Host-side MIDI -> CoProcLang utility
// ============================
// These utilities are meant to be called on the main CPU side to:
//   1) Read a Standard MIDI File (SMF) from SimpleFS,
//   2) Convert it to a compact, monophonic, tempo-aware note stream,
//   3) Store a processed ".coplm" file describing the stream,
//   4) Optionally generate CoProcLang script chunks from the .coplm and play them by calling a user-supplied callback.
//
// No heap is used: the caller supplies a scratch buffer (work) large enough to hold the MIDI file.
//
// Filesystem and playback callbacks (bind these to activeFs and ExecHost respectively):
struct FsOps {
  // All are optional except getFileSize + readFile + writeFile.
  bool (*exists)(const char* name) = nullptr;
  bool (*getFileSize)(const char* name, uint32_t& size) = nullptr;
  uint32_t (*readFile)(const char* name, uint8_t* buf, uint32_t sz) = nullptr;
  // If you don't provide readFileRange, the converter still works (it reads the whole file into work buffer).
  uint32_t (*readFileRange)(const char* name, uint32_t off, uint8_t* buf, uint32_t len) = nullptr;
  // A simple whole-write API is enough for .coplm (small). Replace mode is backend-specific; pass mode as provided by your activeFs wrapper.
  bool (*writeFile)(const char* name, const uint8_t* data, uint32_t len, int writeMode) = nullptr;
};

struct PlayOps {
  // Provide a callback that will send one whole CoProcLang script text to the co-processor and run it.
  // Typical integration: in the callback, call ExecHost.coprocScriptBegin/Data/End/Exec or its convenient wrappers if any.
  // Return true on success.
  bool (*playScriptText)(const char* scriptText, uint32_t len, uint32_t timeoutMs, int32_t& outRetVal) = nullptr;
};

// Configurable bounds for MIDI → note processing
#ifndef COPROCLANG_MIDI_MAX_TEMPO_EVENTS
#define COPROCLANG_MIDI_MAX_TEMPO_EVENTS 256
#endif
#ifndef COPROCLANG_MIDI_MAX_TRACKS
#define COPROCLANG_MIDI_MAX_TRACKS 32
#endif

// Helper: raw .coplm entry (compact; duration microseconds; noteNumber = MIDI note 0..127)
struct CoplmNoteEntry {
  uint8_t note;
  uint8_t reserved0;
  uint16_t reserved1;
  uint32_t dur_us;
};

// Shared header constants (matches CoProc::CoplmHeader in CoProcProto.h)
static inline void makeCoplmName(char* out, size_t outCap, const char* midiNameBase) {
  // Build "<base>.coplm" from "<base>" (assume midiNameBase <= 32 chars in your FS)
  size_t L = strlen(midiNameBase);
  const char* base = midiNameBase;
  // strip extension if present (find last '.')
  const char* dot = nullptr;
  for (size_t i = 0; i < L; ++i)
    if (midiNameBase[i] == '.') dot = midiNameBase + i;
  char core[40];
  size_t coreLen = 0;
  if (dot) {
    coreLen = (size_t)(dot - midiNameBase);
    if (coreLen >= sizeof(core) - 1) coreLen = sizeof(core) - 1;
    memcpy(core, midiNameBase, coreLen);
    core[coreLen] = 0;
  } else {
    coreLen = (L < sizeof(core) - 1) ? L : (sizeof(core) - 1);
    memcpy(core, midiNameBase, coreLen);
    core[coreLen] = 0;
  }
  snprintf(out, outCap, "%s.coplm", core);
}

// Simple MIDI parser (SMF format 0 or 1; time division = ticksPerBeat only).
// We parse twice:
//   Pass A: scan all tracks to build tempo map (meta 0x51) and pick the melody track (most note-on events or user-specified).
//   Pass B: scan selected track and produce monophonic notes via preemption reduction, computing durations in microseconds.
struct TempoEvent {
  uint32_t absTicks;
  uint32_t usPerBeat;
};
static inline double midiNoteToFreq(int note) {
  // 440 * 2^((n-69)/12)
  double x = (double)(note - 69) / 12.0;
  // use Arduino math
  return 440.0 * pow(2.0, x);
}
static inline uint32_t vlq(const uint8_t*& p, const uint8_t* end, bool& ok) {
  ok = false;
  uint32_t v = 0;
  int cnt = 0;
  while (p < end) {
    uint8_t b = *p++;
    v = (v << 7) | (b & 0x7F);
    cnt++;
    if ((b & 0x80) == 0) {
      ok = true;
      return v;
    }
    if (cnt > 4) { return 0; }
  }
  return 0;
}
static bool parseHeaderSMF(const uint8_t* data, uint32_t len, uint16_t& fmt, uint16_t& ntrks, uint16_t& division, uint32_t& off) {
  off = 0;
  if (len < 14) return false;
  if (!(data[0] == 'M' && data[1] == 'T' && data[2] == 'h' && data[3] == 'd')) return false;
  uint32_t hdrLen = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
  if (hdrLen < 6 || len < 8 + hdrLen) return false;
  fmt = (uint16_t)((data[8] << 8) | data[9]);
  ntrks = (uint16_t)((data[10] << 8) | data[11]);
  division = (uint16_t)((data[12] << 8) | data[13]);
  off = 8 + hdrLen;
  return true;
}
static bool findTrackAt(const uint8_t* data, uint32_t len, uint32_t& off, uint32_t& trackLen) {
  if (off + 8 > len) return false;
  if (!(data[off + 0] == 'M' && data[off + 1] == 'T' && data[off + 2] == 'r' && data[off + 3] == 'k')) return false;
  trackLen = (uint32_t)(data[off + 4] << 24) | (uint32_t)(data[off + 5] << 16) | (uint32_t)(data[off + 6] << 8) | (uint32_t)(data[off + 7]);
  off += 8;
  if (off + trackLen > len) return false;
  return true;
}
static void buildTempoMapPass(const uint8_t* trk, uint32_t trkLen, uint32_t& outNoteOnCount, TempoEvent* tempo, uint32_t& tempoCount, uint32_t tempoMax, uint32_t& lastAbs) {
  const uint8_t* p = trk;
  const uint8_t* end = trk + trkLen;
  uint8_t run = 0;
  uint32_t absTicks = 0;
  outNoteOnCount = 0;
  lastAbs = 0;
  while (p < end) {
    bool ok = false;
    uint32_t delta = vlq(p, end, ok);
    if (!ok) break;
    absTicks += delta;
    if (p >= end) break;
    uint8_t st = *p++;
    if (st < 0x80) {
      // running status
      if (!run) break;
      p--;  // step back to treat this byte as data for 'run'
      st = run;
    } else if (st < 0xF0) {
      run = st;
    }
    if ((st & 0xF0) == 0x90) {
      // note on
      if (p + 1 > end) break;
      uint8_t note = *p++;
      uint8_t vel = *p++;
      if (vel > 0) outNoteOnCount++;
    } else if ((st & 0xF0) == 0x80) {
      if (p + 1 > end) break;
      p += 2;
    } else if (st == 0xFF) {
      if (p >= end) break;
      uint8_t meta = *p++;
      bool ok2 = false;
      uint32_t ml = vlq(p, end, ok2);
      if (!ok2 || p + ml > end) break;
      if (meta == 0x51 && ml == 3) {
        uint32_t uspb = (uint32_t)p[0] << 16 | (uint32_t)p[1] << 8 | (uint32_t)p[2];
        // Record (coalesce/keep latest at same tick similarly to Node logic)
        if (tempoCount < tempoMax) {
          tempo[tempoCount].absTicks = absTicks;
          tempo[tempoCount].usPerBeat = uspb;
          tempoCount++;
        }
      }
      p += ml;
    } else {
      // channel or system common
      // 0xF0/0xF7 sysex: length then skip
      if (st == 0xF0 || st == 0xF7) {
        bool ok3 = false;
        uint32_t sl = vlq(p, end, ok3);
        if (!ok3 || p + sl > end) break;
        p += sl;
      } else {
        // channel voice other than note on/off: assume two data bytes for safety for most
        uint8_t hi = (st & 0xF0);
        int need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;  // program change, channel pressure have 1 data byte
        if (p + need > end) break;
        p += need;
      }
    }
    lastAbs = absTicks;
  }
}
static uint32_t ticksToMicrosWithTempo(uint32_t ticks, const TempoEvent* tempo, uint32_t tempoCount, uint16_t tpq) {
  // integrate piecewise from 0..ticks with tempo changes (default 500000 us/qn at tick 0)
  uint32_t us = 0;
  uint32_t prevTick = 0;
  uint32_t curUsPerBeat = 500000;
  // Build a scan: tempo is not guaranteed sorted here; we’ll copy into a tiny sorted order by insertion (tempoCount small)
  // For safety without heap, do a single-pass bubble-like sweep up to small count.
  // Note: tempoCount capacity is bounded; this is fine.
  for (uint32_t i = 0; i + 1 < tempoCount; ++i) {
    for (uint32_t j = i + 1; j < tempoCount; ++j) {
      if (tempo[j].absTicks < tempo[i].absTicks) {
        // cannot swap const; ignore, we’ll process unsorted by repeated scan below
      }
    }
  }
  // We cannot reorder in place (const), so we compute on-the-fly by finding next change each step
  // Simple linear pass: Walk tempo[i] in sorted-by-absTicks order.
  // Since we pushed in chronological order during parse, it's already non-decreasing across tracks, but coalescing same-tick last wins.
  prevTick = 0;
  curUsPerBeat = 500000;
  for (uint32_t i = 0; i < tempoCount; ++i) {
    uint32_t tTick = tempo[i].absTicks;
    if (tTick > ticks) break;
    if (tTick > prevTick) {
      uint32_t dt = tTick - prevTick;
      us += (uint32_t)((uint64_t)dt * (uint64_t)curUsPerBeat / (uint64_t)tpq);
      prevTick = tTick;
    }
    curUsPerBeat = tempo[i].usPerBeat;  // tempo at this tick takes effect from here onward
  }
  if (ticks > prevTick) {
    uint32_t dt = ticks - prevTick;
    us += (uint32_t)((uint64_t)dt * (uint64_t)curUsPerBeat / (uint64_t)tpq);
  }
  return us;
}

// Main entry: ensure .coplm exists for a given MIDI file and optionally play it via PlayOps.
// Returns true on success; false on parse/FS/play error.
//
// Parameters:
//   fs          - bind to activeFs.* (exists, getFileSize, readFile, writeFile)
//   midiName    - MIDI filename in SimpleFS (<= 32 chars).
//   buzzPin     - buzzer GPIO
//   ledPin      - LED GPIO (may be same or different; used as activity indicator in scripts)
//   trackOpt    - -1 to auto-pick track with most noteOn; otherwise track index (0-based)
//   gapMs       - pause between notes (default 20 used by Node tool; you choose)
//   maxLines    - CoProcLang max lines per chunk (e.g., 3712 like Node tool)
//   maxSplits   - bounds #parts
//   play        - callback to send one full script text to coprocessor
//   work        - caller-supplied scratch buffer
//   workCap     - size of work buffer; must be >= MIDI file size
//   writeMode   - FS write mode; pass fsReplaceMode() from your activeFs binding
//   timeoutMs   - per-script execution timeout for callback
//
// Behavior:
//   - If "<base>.coplm" exists, we skip re-processing (we still play it).
//   - If it doesn't exist, we parse the MIDI file and produce the .coplm file, then play it.
//   - Playback: stream one script per chunk to play->playScriptText() (no temp .cpl files).
static bool midiEnsureCoplmAndPlay(FsOps& fs, PlayOps& play,
                                   const char* midiName,
                                   int buzzPin, int ledPin, int trackOpt, int gapMs,
                                   uint32_t maxLines, uint32_t maxSplits,
                                   char* work, uint32_t workCap,
                                   int writeMode,
                                   uint32_t timeoutMs) {
  if (!midiName || !fs.getFileSize || !fs.readFile || !fs.writeFile || !work) return false;
  // 1) Read MIDI to work buffer
  uint32_t midiSize = 0;
  if (!fs.getFileSize(midiName, midiSize) || midiSize == 0) return false;
  if (midiSize > workCap) return false;
  if (fs.readFile(midiName, (uint8_t*)work, midiSize) != midiSize) return false;
  const uint8_t* data = (const uint8_t*)work;
  uint32_t len = midiSize;

  // 2) Parse header
  uint16_t fmt = 0, ntrks = 0, division = 0;
  uint32_t off = 0;
  if (!parseHeaderSMF(data, len, fmt, ntrks, division, off)) return false;
  if ((division & 0x8000) != 0) {
    // SMPTE not supported
    return false;
  }
  uint16_t tpq = division;
  if (tpq == 0) return false;

  if (ntrks > COPROCLANG_MIDI_MAX_TRACKS) ntrks = COPROCLANG_MIDI_MAX_TRACKS;

  // 3) First pass: gather tempo across all tracks + count note-ons per track
  TempoEvent tempo[COPROCLANG_MIDI_MAX_TEMPO_EVENTS + 1];
  uint32_t tempoCount = 0;
  // Insert default 500000 at tick 0
  tempo[tempoCount].absTicks = 0;
  tempo[tempoCount].usPerBeat = 500000;
  tempoCount++;

  uint32_t trackNoteOnCounts[COPROCLANG_MIDI_MAX_TRACKS];
  for (uint16_t i = 0; i < ntrks; ++i) trackNoteOnCounts[i] = 0;

  uint32_t trkOff = off;
  for (uint16_t i = 0; i < ntrks; ++i) {
    uint32_t trkLen = 0;
    if (!findTrackAt(data, len, trkOff, trkLen)) return false;
    uint32_t tmpCount = 0;
    uint32_t lastAbs = 0;
    buildTempoMapPass(&data[trkOff], trkLen, tmpCount, tempo, tempoCount, COPROCLANG_MIDI_MAX_TEMPO_EVENTS, lastAbs);
    trackNoteOnCounts[i] = tmpCount;
    trkOff += trkLen;
  }

  // Track selection
  int selIndex = trackOpt;
  if (selIndex < 0) {
    int best = -1;
    uint32_t bestC = 0;
    for (uint16_t i = 0; i < ntrks; ++i) {
      if (trackNoteOnCounts[i] > bestC) {
        bestC = trackNoteOnCounts[i];
        best = (int)i;
      }
    }
    selIndex = best;
  }
  if (selIndex < 0 || selIndex >= (int)ntrks) return false;

  // 4) Second pass: parse selected track and build monophonic note durations -> store in .coplm
  // We'll parse the selected track twice: first to count output notes, then to fill the .coplm body.
  auto parseSelectedTrackMonoCount = [&](uint32_t& outNotesCount) -> bool {
    outNotesCount = 0;
    // Locate selected track again
    uint32_t scanOff = off;
    for (int i = 0; i < selIndex; ++i) {
      uint32_t tl = 0;
      if (!findTrackAt(data, len, scanOff, tl)) return false;
      scanOff += tl;
    }
    uint32_t trkLen = 0;
    uint32_t selTrackStart = scanOff;
    if (!findTrackAt(data, len, scanOff, trkLen)) return false;
    const uint8_t* p = &data[scanOff];
    const uint8_t* end = p + trkLen;
    uint8_t run = 0;
    uint32_t absTicks = 0;

    // Mono reduction state
    int currentNote = -1;
    uint32_t currentStart = 0;
    bool currentActive = false;

    while (p < end) {
      bool ok = false;
      uint32_t delta = vlq(p, end, ok);
      if (!ok) break;
      absTicks += delta;
      if (p >= end) break;
      uint8_t st = *p++;
      if (st < 0x80) {
        if (!run) break;
        p--;
        st = run;
      } else if (st < 0xF0) {
        run = st;
      }
      if ((st & 0xF0) == 0x90) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        uint8_t vel = *p++;
        if (vel > 0) {
          // preempt current
          if (currentActive) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs) outNotesCount++;
          }
          currentNote = note;
          currentStart = absTicks;
          currentActive = true;
        } else {
          // velocity=0 => off
          if (currentActive && currentNote == note) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs) outNotesCount++;
            currentActive = false;
          }
        }
      } else if ((st & 0xF0) == 0x80) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        (void)*p++;  // vel
        if (currentActive && currentNote == (int)note) {
          uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
          uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
          if (endUs > startUs) outNotesCount++;
          currentActive = false;
        }
      } else if (st == 0xFF) {
        if (p >= end) break;
        uint8_t meta = *p++;
        bool ok2 = false;
        uint32_t ml = vlq(p, end, ok2);
        if (!ok2 || p + ml > end) break;
        p += ml;
      } else {
        if (st == 0xF0 || st == 0xF7) {
          bool ok3 = false;
          uint32_t sl = vlq(p, end, ok3);
          if (!ok3 || p + sl > end) break;
          p += sl;
        } else {
          uint8_t hi = (st & 0xF0);
          int need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
          if (p + need > end) break;
          p += need;
        }
      }
    }
    // close remaining
    if (currentActive) {
      // Track end is absTicks of last event; no new end; drop it
      // We'll ignore unfinished trailing note (no explicit off and no preempt) => set end to last absTicks
      uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
      uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
      if (endUs > startUs) outNotesCount++;
    }
    return true;
  };

  uint32_t outNotes = 0;
  if (!parseSelectedTrackMonoCount(outNotes)) return false;

  // Build .coplm in-place into a small local structure and write to FS
  // Layout: header then note entries
  struct CoplmHeaderLocal {
    char magic[4];
    uint8_t version;
    uint8_t reserved;
    uint16_t flags;
    uint32_t buzzPin;
    uint32_t ledPin;
    uint32_t gapMs;
    uint32_t notesCount;
  } hdr;

  hdr.magic[0] = 'C';
  hdr.magic[1] = 'P';
  hdr.magic[2] = 'L';
  hdr.magic[3] = 'M';
  hdr.version = 1;
  hdr.reserved = 0;
  hdr.flags = 0;
  hdr.buzzPin = (uint32_t)buzzPin;
  hdr.ledPin = (uint32_t)ledPin;
  hdr.gapMs = (uint32_t)((gapMs < 0) ? 0 : gapMs);
  hdr.notesCount = outNotes;

  // Build filename
  char coplmName[40];
  makeCoplmName(coplmName, sizeof(coplmName), midiName);

  bool haveCoplm = false;
  if (fs.exists) {
    haveCoplm = fs.exists(coplmName);
  }

  if (!haveCoplm) {
    // Fill an output buffer (small chunk assemble)
    // .coplm is small: header + N*(8 bytes) (1+7 padded). outNotes is moderate for small tunes.
    // Compose into a temporary stack buffer if small; otherwise reuse work buffer to avoid heap.
    uint32_t coplmBytes = sizeof(CoplmHeaderLocal) + outNotes * sizeof(CoplmNoteEntry);
    if (coplmBytes > workCap) return false;
    uint8_t* outBuf = (uint8_t*)work;
    memcpy(outBuf, &hdr, sizeof(hdr));
    CoplmNoteEntry* entries = (CoplmNoteEntry*)(outBuf + sizeof(hdr));
    // Fill entries by parsing again
    // Locate selected track again
    uint32_t scanOff = off;
    for (int i = 0; i < selIndex; ++i) {
      uint32_t tl = 0;
      if (!findTrackAt(data, len, scanOff, tl)) return false;
      scanOff += tl;
    }
    uint32_t trkLen = 0;
    if (!findTrackAt(data, len, scanOff, trkLen)) return false;
    const uint8_t* p = &data[scanOff];
    const uint8_t* end = p + trkLen;
    uint8_t run = 0;
    uint32_t absTicks = 0;
    int currentNote = -1;
    uint32_t currentStart = 0;
    bool currentActive = false;
    uint32_t outIdx = 0;

    while (p < end) {
      bool ok = false;
      uint32_t delta = vlq(p, end, ok);
      if (!ok) break;
      absTicks += delta;
      if (p >= end) break;
      uint8_t st = *p++;
      if (st < 0x80) {
        if (!run) break;
        p--;
        st = run;
      } else if (st < 0xF0) {
        run = st;
      }
      if ((st & 0xF0) == 0x90) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        uint8_t vel = *p++;
        if (vel > 0) {
          if (currentActive) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs && outIdx < outNotes) {
              entries[outIdx].note = (uint8_t)currentNote;
              entries[outIdx].reserved0 = 0;
              entries[outIdx].reserved1 = 0;
              entries[outIdx].dur_us = endUs - startUs;
              outIdx++;
            }
          }
          currentNote = note;
          currentStart = absTicks;
          currentActive = true;
        } else {
          if (currentActive && currentNote == note) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs && outIdx < outNotes) {
              entries[outIdx].note = (uint8_t)currentNote;
              entries[outIdx].reserved0 = 0;
              entries[outIdx].reserved1 = 0;
              entries[outIdx].dur_us = endUs - startUs;
              outIdx++;
            }
            currentActive = false;
          }
        }
      } else if ((st & 0xF0) == 0x80) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        (void)*p++;
        if (currentActive && currentNote == (int)note) {
          uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
          uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
          if (endUs > startUs && outIdx < outNotes) {
            entries[outIdx].note = (uint8_t)currentNote;
            entries[outIdx].reserved0 = 0;
            entries[outIdx].reserved1 = 0;
            entries[outIdx].dur_us = endUs - startUs;
            outIdx++;
          }
          currentActive = false;
        }
      } else if (st == 0xFF) {
        if (p >= end) break;
        uint8_t meta = *p++;
        bool ok2 = false;
        uint32_t ml = vlq(p, end, ok2);
        if (!ok2 || p + ml > end) break;
        p += ml;
      } else {
        if (st == 0xF0 || st == 0xF7) {
          bool ok3 = false;
          uint32_t sl = vlq(p, end, ok3);
          if (!ok3 || p + sl > end) break;
          p += sl;
        } else {
          uint8_t hi = (st & 0xF0);
          int need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
          if (p + need > end) break;
          p += need;
        }
      }
    }
    // close if needed
    if (currentActive && outIdx < outNotes) {
      // end at last absTicks we saw
      uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
      uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
      if (endUs > startUs) {
        entries[outIdx].note = (uint8_t)currentNote;
        entries[outIdx].reserved0 = 0;
        entries[outIdx].reserved1 = 0;
        entries[outIdx].dur_us = endUs - startUs;
        outIdx++;
      }
    }
    // fix count if we had some zero/neg filters
    ((CoplmHeaderLocal*)outBuf)->notesCount = outIdx;
    // Write .coplm
    if (!fs.writeFile(coplmName, outBuf, sizeof(hdr) + outIdx * sizeof(CoplmNoteEntry), writeMode)) {
      return false;
    }
  }

  // 5) Playback: load .coplm from FS, then build scripts in chunks (<= maxLines) and stream them to coproc
  if (!play.playScriptText) return true;  // nothing to do if no playback
  // Read .coplm back (small)
  uint32_t coplmSize = 0;
  if (!fs.getFileSize(coplmName, coplmSize) || coplmSize < sizeof(CoplmHeaderLocal)) return false;
  if (coplmSize > workCap) return false;
  if (fs.readFile(coplmName, (uint8_t*)work, coplmSize) != coplmSize) return false;
  CoplmHeaderLocal* H = (CoplmHeaderLocal*)work;
  if (!(H->magic[0] == 'C' && H->magic[1] == 'P' && H->magic[2] == 'L' && H->magic[3] == 'M')) return false;
  if (H->version != 1) return false;
  uint32_t N = H->notesCount;
  if (sizeof(CoplmHeaderLocal) + N * sizeof(CoplmNoteEntry) > coplmSize) return false;
  CoplmNoteEntry* E = (CoplmNoteEntry*)((uint8_t*)work + sizeof(CoplmHeaderLocal));

  // Chunking by line count. Per-note lines:
  const uint32_t perNoteLines = 12 + ((gapMs > 0) ? 1u : 0u);
  const uint32_t headerLines = 5;
  const uint32_t footerLines = 2;

  // quick lambda to estimate max notes per chunk
  auto notesPerChunk = [&](uint32_t maxLines) -> uint32_t {
    if (maxLines <= headerLines + footerLines) return 0;
    uint32_t room = maxLines - (headerLines + footerLines);
    uint32_t k = room / (perNoteLines ? perNoteLines : 1);
    return (k > 0) ? k : 1;
  };
  uint32_t npc = notesPerChunk(maxLines);
  if (npc == 0) npc = 1;
  uint32_t parts = (N + npc - 1) / npc;
  if (parts == 0) parts = 1;
  if (parts > maxSplits) {
    // too many splits requested; adjust to allowed max by increasing notes per chunk
    npc = (N + maxSplits - 1) / maxSplits;
    parts = (N + npc - 1) / npc;
  }

  // Build and play each part as CoProcLang text
  // We'll render into an internal stack buffer in small-ish chunks to keep RAM low.
  // However the whole script string must be contiguous for the callback. Choose a modest cap; your ExecHost will stream chunks anyway.
  const size_t SCRIPT_BUF = 4096;
  char* sbuf = (char*)work;  // re-use the same buffer if large enough
  size_t sbufCap = (size_t)workCap;
  if (sbufCap < SCRIPT_BUF) return false;

  auto emitLine = [&](char* dst, size_t cap, size_t& off, const char* s) -> bool {
    size_t L = strlen(s);
    if (off + L + 1 >= cap) return false;
    memcpy(dst + off, s, L);
    off += L;
    dst[off++] = '\n';
    dst[off] = 0;
    return true;
  };

  auto buildAndPlayPart = [&](uint32_t startIndex, uint32_t count, uint32_t partIndex, uint32_t totalParts) -> bool {
    size_t offS = 0;
    // Header
    char line[96];
    snprintf(line, sizeof(line), "# Generated by midi2coproc (embedded) PART %lu/%lu", (unsigned long)(partIndex + 1), (unsigned long)totalParts);
    if (!emitLine(sbuf, sbufCap, offS, line)) return false;
    snprintf(line, sizeof(line), "PINMODE %d OUT", buzzPin);
    if (!emitLine(sbuf, sbufCap, offS, line)) return false;
    snprintf(line, sizeof(line), "PINMODE %d OUT", ledPin);
    if (!emitLine(sbuf, sbufCap, offS, line)) return false;
    if (!emitLine(sbuf, sbufCap, offS, "MBCLR")) return false;
    snprintf(line, sizeof(line), "DWRITE %d 0", ledPin);
    if (!emitLine(sbuf, sbufCap, offS, line)) return false;

    uint32_t localNotes = 0;
    for (uint32_t i = 0; i < count; ++i) {
      const CoplmNoteEntry& en = E[startIndex + i];
      if (en.dur_us == 0) continue;
      // compute half-period and cycles
      double f = midiNoteToFreq((int)en.note);
      if (f < 1.0) f = 1.0;
      uint32_t halfPeriod = (uint32_t)((1000000.0 / (2.0 * f)) + 0.5);
      if (halfPeriod == 0) halfPeriod = 1;
      // Ensure at least 1 cycle
      uint32_t cycles = (uint32_t)((double)en.dur_us / (double)(2ULL * (uint64_t)halfPeriod));
      if (cycles == 0) cycles = 1;

      snprintf(line, sizeof(line), "# Note %u ~%.1fHz, ~%lums",
               (unsigned)en.note, (float)f, (unsigned long)(en.dur_us / 1000u));
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      snprintf(line, sizeof(line), "LET R1 %lu", (unsigned long)halfPeriod);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      snprintf(line, sizeof(line), "LET R0 %lu", (unsigned long)cycles);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      snprintf(line, sizeof(line), "DWRITE %d 1", ledPin);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      char loopLabel[16];
      snprintf(loopLabel, sizeof(loopLabel), "N%lu", (unsigned long)i);
      snprintf(line, sizeof(line), "%s:", loopLabel);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      snprintf(line, sizeof(line), "DWRITE %d 1", buzzPin);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      if (!emitLine(sbuf, sbufCap, offS, "DELAY_US R1")) return false;
      snprintf(line, sizeof(line), "DWRITE %d 0", buzzPin);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      if (!emitLine(sbuf, sbufCap, offS, "DELAY_US R1")) return false;
      if (!emitLine(sbuf, sbufCap, offS, "SUB R0 1")) return false;
      snprintf(line, sizeof(line), "IF R0 > 0 GOTO %s", loopLabel);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      snprintf(line, sizeof(line), "DWRITE %d 0", ledPin);
      if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      if (gapMs > 0) {
        snprintf(line, sizeof(line), "DELAY %d", gapMs);
        if (!emitLine(sbuf, sbufCap, offS, line)) return false;
      }
      localNotes++;
    }
    snprintf(line, sizeof(line), "MBAPP \"notes=%lu\"", (unsigned long)localNotes);
    if (!emitLine(sbuf, sbufCap, offS, line)) return false;
    if (!emitLine(sbuf, sbufCap, offS, "RET 0")) return false;

    int32_t ret = 0;
    if (!play.playScriptText(sbuf, (uint32_t)offS, timeoutMs, ret)) return false;
    return true;
  };

  if (N == 0) {
    // trivial no-note script to keep consistent
    size_t offS = 0;
    char* sbuf2 = (char*)work;
    size_t cap = (size_t)workCap;
    char line[96];
    snprintf(line, sizeof(line), "# Generated by midi2coproc (embedded) - no notes");
    auto emitLine2 = [&](const char* s) -> bool {
      size_t L = strlen(s);
      if (offS + L + 1 >= cap) return false;
      memcpy(sbuf2 + offS, s, L);
      offS += L;
      sbuf2[offS++] = '\n';
      sbuf2[offS] = 0;
      return true;
    };
    if (!emitLine2(line)) return false;
    snprintf(line, sizeof(line), "PINMODE %d OUT", buzzPin);
    if (!emitLine2(line)) return false;
    snprintf(line, sizeof(line), "PINMODE %d OUT", ledPin);
    if (!emitLine2(line)) return false;
    if (!emitLine2("MBCLR")) return false;
    snprintf(line, sizeof(line), "DWRITE %d 0", ledPin);
    if (!emitLine2(line)) return false;
    if (!emitLine2("MBAPP \"notes=0\"")) return false;
    if (!emitLine2("RET 0")) return false;
    int32_t ret = 0;
    return play.playScriptText(sbuf2, (uint32_t)offS, timeoutMs, ret);
  }

  uint32_t done = 0;
  for (uint32_t part = 0; part < parts; ++part) {
    uint32_t remain = (N - done);
    uint32_t take = (remain > npc) ? npc : remain;
    if (!buildAndPlayPart(done, take, part, parts)) return false;
    done += take;
  }
  return true;
}

}  // namespace CoProcLang