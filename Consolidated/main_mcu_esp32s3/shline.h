#pragma once
#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// ========== Config (override with -D or before include if needed) ==========
#ifndef SHLINE_MAX_LINE
#define SHLINE_MAX_LINE 4096
#endif
#ifndef SHLINE_HISTORY_MAX
#define SHLINE_HISTORY_MAX 16
#endif
#ifndef SHLINE_HISTORY_ENTRY_MAX
#define SHLINE_HISTORY_ENTRY_MAX 192
#endif
#ifndef SHLINE_PROMPT_MAX
#define SHLINE_PROMPT_MAX 16
#endif
#ifndef SHLINE_TAB_SPACES
#define SHLINE_TAB_SPACES 2
#endif

namespace shline {

enum class Mode { Basic,
                  Advanced };

enum EscState {
  ES_Normal,
  ES_GotEsc,
  ES_CSI,
  ES_SS3,
  ES_CSI_Param
};

struct State {
  // IO
  Stream* io = nullptr;

  // Mode and prompt
  Mode mode = Mode::Basic;
  char prompt[SHLINE_PROMPT_MAX] = "> ";
  size_t term_cols = 80;

  // Line buffer and view state
  char line[SHLINE_MAX_LINE];
  size_t line_len = 0;
  size_t cursor = 0;
  size_t view_start = 0;
  size_t last_render_len = 0;

  // History
  char history[SHLINE_HISTORY_MAX][SHLINE_HISTORY_ENTRY_MAX];
  int history_count = 0;
  int history_browse = -1;  // -1 = not browsing
  char saved_before_browse[SHLINE_HISTORY_ENTRY_MAX];
  size_t saved_before_browse_len = 0;

  // Escape parser (advanced)
  EscState esc = ES_Normal;
  int csi_param = 0;

  // Basic mode newline accumulator
  // (we reuse 'line' buffer for both modes)
};

// ---------------- API ----------------
inline void begin(State& st, Stream& io, const char* prompt = "> ") {
  st.io = &io;
  if (prompt && *prompt) {
    strncpy(st.prompt, prompt, sizeof(st.prompt) - 1);
    st.prompt[sizeof(st.prompt) - 1] = 0;
  }
  st.term_cols = 80;
  st.line_len = 0;
  st.cursor = 0;
  st.view_start = 0;
  st.last_render_len = 0;
  st.history_count = 0;
  st.history_browse = -1;
  st.saved_before_browse_len = 0;
  st.esc = ES_Normal;
  st.csi_param = 0;
  st.line[0] = 0;
}

inline void setMode(State& st, Mode m) {
  st.mode = m;
}
inline Mode mode(const State& st) {
  return st.mode;
}

inline void setTerminalCols(State& st, size_t cols) {
  if (cols < 20) cols = 20;
  if (cols > 240) cols = 240;
  st.term_cols = cols;
}

inline void setPrompt(State& st, const char* prompt) {
  if (!prompt) return;
  strncpy(st.prompt, prompt, sizeof(st.prompt) - 1);
  st.prompt[sizeof(st.prompt) - 1] = 0;
}

inline void clear(State& st) {
  st.line_len = 0;
  st.cursor = 0;
  st.view_start = 0;
  st.last_render_len = 0;
  st.history_browse = -1;
  st.saved_before_browse_len = 0;
  st.line[0] = 0;
}

inline size_t promptLen(const State& st) {
  return strlen(st.prompt);
}
inline size_t maxLineLen() {
  return SHLINE_MAX_LINE - 1;
}

inline size_t visibleCols(const State& st) {
  size_t p = promptLen(st);
  if (st.term_cols <= p + 8) return 16;
  size_t v = st.term_cols - p;
  if (v < 8) v = 8;
  return v;
}

inline void historyPush(State& st, const char* s, size_t len) {
  if (!s || len == 0) return;
  if (st.history_count > 0) {
    const char* last = st.history[st.history_count - 1];
    if (strncmp(last, s, SHLINE_HISTORY_ENTRY_MAX - 1) == 0 && strlen(last) == len) return;
  }
  if (st.history_count < SHLINE_HISTORY_MAX) {
    size_t L = (len < (SHLINE_HISTORY_ENTRY_MAX - 1)) ? len : (SHLINE_HISTORY_ENTRY_MAX - 1);
    memcpy(st.history[st.history_count], s, L);
    st.history[st.history_count][L] = 0;
    st.history_count++;
  } else {
    for (int i = 1; i < SHLINE_HISTORY_MAX; ++i) {
      strncpy(st.history[i - 1], st.history[i], SHLINE_HISTORY_ENTRY_MAX);
      st.history[i - 1][SHLINE_HISTORY_ENTRY_MAX - 1] = 0;
    }
    size_t L = (len < (SHLINE_HISTORY_ENTRY_MAX - 1)) ? len : (SHLINE_HISTORY_ENTRY_MAX - 1);
    memcpy(st.history[SHLINE_HISTORY_MAX - 1], s, L);
    st.history[SHLINE_HISTORY_MAX - 1][L] = 0;
  }
}

inline void adjustViewToCursor(State& st) {
  size_t vcols = visibleCols(st);
  if (st.cursor < st.view_start) {
    st.view_start = st.cursor;
  } else if (st.cursor >= st.view_start + vcols) {
    st.view_start = st.cursor - (vcols - 1);
  }
  size_t maxStart = (st.line_len > vcols) ? (st.line_len - vcols) : 0;
  if (st.view_start > maxStart) st.view_start = maxStart;
}

inline void renderLine(State& st) {
  if (!st.io) return;
  adjustViewToCursor(st);
  st.io->print("\r\x1b[2K");
  st.io->print(st.prompt);
  size_t vcols = visibleCols(st);
  if (st.line_len > 0) {
    size_t visLen = (st.line_len > st.view_start) ? (st.line_len - st.view_start) : 0;
    if (visLen > vcols) visLen = vcols;
    if (visLen) st.io->write((const uint8_t*)(st.line + st.view_start), visLen);
  }
  size_t col = promptLen(st) + (st.cursor - st.view_start);
  st.io->print("\r\x1b[");
  st.io->print((unsigned)col);
  st.io->print('C');
  st.last_render_len = st.line_len;
}

inline void setLineFrom(State& st, const char* s) {
  if (!s) s = "";
  size_t L = strlen(s);
  if (L > maxLineLen()) L = maxLineLen();
  memcpy(st.line, s, L);
  st.line_len = L;
  st.line[st.line_len] = 0;
  st.cursor = st.line_len;
  st.view_start = (st.line_len > visibleCols(st)) ? (st.line_len - visibleCols(st)) : 0;
  if (st.mode == Mode::Advanced) renderLine(st);
}

inline void beginBrowseIfNeeded(State& st) {
  if (st.history_browse == -1) {
    size_t L = (st.line_len < (SHLINE_HISTORY_ENTRY_MAX - 1)) ? st.line_len : (SHLINE_HISTORY_ENTRY_MAX - 1);
    memcpy(st.saved_before_browse, st.line, L);
    st.saved_before_browse[L] = 0;
    st.saved_before_browse_len = L;
  }
}

inline void browseUp(State& st) {
  if (st.history_count == 0) return;
  beginBrowseIfNeeded(st);
  if (st.history_browse < (st.history_count - 1)) {
    st.history_browse++;
    int idx = st.history_count - 1 - st.history_browse;
    setLineFrom(st, st.history[idx]);
  }
}

inline void browseDown(State& st) {
  if (st.history_browse < 0) return;
  if (st.history_browse > 0) {
    st.history_browse--;
    int idx = st.history_count - 1 - st.history_browse;
    setLineFrom(st, st.history[idx]);
  } else {
    st.history_browse = -1;
    size_t L = (st.saved_before_browse_len < SHLINE_MAX_LINE - 1) ? st.saved_before_browse_len : (SHLINE_MAX_LINE - 1);
    memcpy(st.line, st.saved_before_browse, L);
    st.line_len = L;
    st.line[st.line_len] = 0;
    st.cursor = st.line_len;
    st.view_start = (st.line_len > visibleCols(st)) ? (st.line_len - visibleCols(st)) : 0;
    if (st.mode == Mode::Advanced) renderLine(st);
  }
}

inline void printPrompt(State& st) {
  if (!st.io) return;
  if (st.mode == Mode::Advanced) {
    renderLine(st);  // shows prompt and current buffer
  } else {
    st.io->print(st.prompt);
  }
}

// Try to detect ANSI-capable terminal by asking for cursor position (ESC[6n).
// Returns true if a response like ESC[row;colR] is seen quickly (non-blocking-ish).
inline bool probeAnsiSupport(State& st, uint32_t timeoutMs = 25) {
  if (!st.io) return false;
  // Flush any pending input
  while (st.io->available()) (void)st.io->read();
  // Ask for cursor position
  st.io->print("\x1b[6n");
  uint32_t start = millis();
  // Look for ESC [
  bool gotEsc = false, gotBracket = false;
  while ((millis() - start) < timeoutMs) {
    while (st.io->available()) {
      int c = st.io->read();
      if (c < 0) break;
      if (!gotEsc) {
        if (c == 0x1B) gotEsc = true;
        continue;
      }
      if (!gotBracket) {
        if (c == '[') gotBracket = true;
        continue;
      }
      // Now consume digits/; until 'R'
      if (c == 'R') return true;
    }
    delay(1);
    yield();
  }
  return false;
}

// Poll: returns true when a full line is ready in outBuf (NUL-terminated).
// Non-blocking: consumes available input and updates internal state.
inline bool poll(State& st, char* outBuf, size_t outCap) {
  if (!st.io || !outBuf || outCap == 0) return false;

  if (st.mode == Mode::Basic) {
    bool lineReady = false;
    while (st.io->available()) {
      int ic = st.io->read();
      if (ic < 0) break;
      char c = (char)ic;
      if (c == '\r' || c == '\n') {
        // Consume LF if CRLF
        if (c == '\r' && st.io->available() && st.io->peek() == '\n') st.io->read();
        st.io->write('\r');
        st.io->write('\n');

        st.line[st.line_len] = 0;
        if (st.line_len > 0) historyPush(st, st.line, st.line_len);

        size_t L = st.line_len < (outCap - 1) ? st.line_len : (outCap - 1);
        memcpy(outBuf, st.line, L);
        outBuf[L] = 0;

        // Reset buffer for next line
        st.line_len = 0;
        st.cursor = 0;
        st.view_start = 0;
        st.last_render_len = 0;
        st.history_browse = -1;
        st.saved_before_browse_len = 0;
        lineReady = true;
        break;
      } else if ((c == 0x08) || (c == 0x7F)) {
        if (st.line_len > 0) {
          st.line_len--;
          st.line[st.line_len] = 0;
          // Basic echo: move back, overwrite, move back (keep it simple)
          st.io->print("\b \b");
        } else {
          st.io->write('\a');
        }
      } else if (c >= 32 && c <= 126) {
        if (st.line_len + 1 <= maxLineLen()) {
          st.line[st.line_len++] = c;
          st.line[st.line_len] = 0;
          st.io->write(c);  // echo
        } else {
          st.io->write('\a');
        }
      } else {
        // ignore control sequences in basic mode
      }
    }
    return lineReady;
  }

  // Advanced mode (ANSI + single-line editor)
  bool lineReady = false;
  while (st.io->available()) {
    int ic = st.io->read();
    if (ic < 0) break;
    char c = (char)ic;

    if (st.esc == ES_Normal) {
      if (c == '\r' || c == '\n') {
        if (c == '\r') {
          if (st.io->available() && st.io->peek() == '\n') st.io->read();
        }
        st.io->write('\r');
        st.io->write('\n');
        st.line[st.line_len] = 0;
        if (st.line_len > 0) historyPush(st, st.line, st.line_len);

        size_t L = st.line_len < (outCap - 1) ? st.line_len : (outCap - 1);
        memcpy(outBuf, st.line, L);
        outBuf[L] = 0;

        st.history_browse = -1;
        st.saved_before_browse_len = 0;
        st.last_render_len = 0;
        st.cursor = 0;
        st.line_len = 0;
        st.view_start = 0;
        lineReady = true;
        break;
      } else if ((c == 0x08) || (c == 0x7F)) {
        if (st.cursor > 0) {
          for (size_t i = st.cursor - 1; i + 1 < st.line_len; ++i) st.line[i] = st.line[i + 1];
          st.line_len--;
          st.cursor--;
          st.line[st.line_len] = 0;
          renderLine(st);
        } else {
          st.io->write('\a');
        }
        continue;
      } else if (c == 0x1B) {
        st.esc = ES_GotEsc;
        st.csi_param = 0;
        continue;
      } else if (c == '\t') {
        if (st.line_len + SHLINE_TAB_SPACES <= maxLineLen()) {
          for (size_t i = st.line_len + (SHLINE_TAB_SPACES - 1); i >= st.cursor + SHLINE_TAB_SPACES; --i) st.line[i] = st.line[i - SHLINE_TAB_SPACES];
          for (int k = 0; k < SHLINE_TAB_SPACES; ++k) st.line[st.cursor + k] = ' ';
          st.cursor += SHLINE_TAB_SPACES;
          st.line_len += SHLINE_TAB_SPACES;
          st.line[st.line_len] = 0;
          renderLine(st);
        } else {
          st.io->write('\a');
        }
        continue;
      } else if (c == 0x01) {  // Ctrl+A
        st.cursor = 0;
        renderLine(st);
        continue;
      } else if (c == 0x05) {  // Ctrl+E
        st.cursor = st.line_len;
        renderLine(st);
        continue;
      } else if (c == 0x0B) {  // Ctrl+K
        st.line_len = st.cursor;
        st.line[st.line_len] = 0;
        renderLine(st);
        continue;
      } else if (c == 0x15) {  // Ctrl+U
        st.cursor = 0;
        st.line_len = 0;
        st.line[0] = 0;
        st.view_start = 0;
        renderLine(st);
        continue;
      } else if (c == 0x0C) {  // Ctrl+L
        st.io->println();
        renderLine(st);
        continue;
      } else if (c >= 32 && c <= 126) {
        if (st.line_len + 1 <= maxLineLen()) {
          for (size_t i = st.line_len; i > st.cursor; --i) st.line[i] = st.line[i - 1];
          st.line[st.cursor] = c;
          st.line_len++;
          st.cursor++;
          st.line[st.line_len] = 0;
          renderLine(st);
        } else {
          st.io->write('\a');
        }
        continue;
      } else {
        continue;
      }
    } else if (st.esc == ES_GotEsc) {
      if (c == '[') st.esc = ES_CSI;
      else if (c == 'O') st.esc = ES_SS3;
      else st.esc = ES_Normal;
    } else if (st.esc == ES_SS3) {
      if (c == 'H') {
        st.cursor = 0;
        renderLine(st);
      } else if (c == 'F') {
        st.cursor = st.line_len;
        renderLine(st);
      }
      st.esc = ES_Normal;
    } else if (st.esc == ES_CSI) {
      if (c >= '0' && c <= '9') {
        st.csi_param = (st.csi_param * 10) + (c - '0');
        st.esc = ES_CSI_Param;
      } else {
        if (c == 'A') {
          browseUp(st);
        } else if (c == 'B') {
          browseDown(st);
        } else if (c == 'C') {
          if (st.cursor < st.line_len) {
            st.cursor++;
            renderLine(st);
          }
        } else if (c == 'D') {
          if (st.cursor > 0) {
            st.cursor--;
            renderLine(st);
          }
        } else if (c == 'H') {
          st.cursor = 0;
          renderLine(st);
        } else if (c == 'F') {
          st.cursor = st.line_len;
          renderLine(st);
        }
        st.esc = ES_Normal;
        st.csi_param = 0;
      }
    } else if (st.esc == ES_CSI_Param) {
      if (c >= '0' && c <= '9') {
        st.csi_param = (st.csi_param * 10) + (c - '0');
      } else if (c == '~') {
        if (st.csi_param == 3) {  // Delete
          if (st.cursor < st.line_len) {
            for (size_t i = st.cursor; i + 1 < st.line_len; ++i) st.line[i] = st.line[i + 1];
            st.line_len--;
            st.line[st.line_len] = 0;
            renderLine(st);
          } else {
            st.io->write('\a');
          }
        } else if (st.csi_param == 1 || st.csi_param == 7) {
          st.cursor = 0;
          renderLine(st);
        } else if (st.csi_param == 4 || st.csi_param == 8) {
          st.cursor = st.line_len;
          renderLine(st);
        }
        st.esc = ES_Normal;
        st.csi_param = 0;
      } else {
        st.esc = ES_Normal;
        st.csi_param = 0;
      }
    }
  }

  return lineReady;
}

}  // namespace shline