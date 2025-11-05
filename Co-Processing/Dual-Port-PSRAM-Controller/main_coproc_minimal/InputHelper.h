// ================== Improved Console Line Editing with History (single-row, horizontal scroll) ==================
static const char* PROMPT = "> ";

// Input buffer (keep the existing large buffer, but we cap usable length to history capacity)
static char lineBuf[FS_SECTOR_SIZE];

// History (SRAM only)
#define HISTORY_MAX 16
#define HISTORY_ENTRY_MAX 192
static char g_history[HISTORY_MAX][HISTORY_ENTRY_MAX];
static int g_history_count = 0;                        // number of valid entries
static int g_history_browse = -1;                      // -1 = not browsing; 0 = newest; increases as we go up
static char g_saved_before_browse[HISTORY_ENTRY_MAX];  // buffer to restore when exiting browse
static size_t g_saved_before_browse_len = 0;

// Terminal width handling and single-line viewport
static size_t g_term_cols = 80;  // default, configurable via 'termwidth' command
static size_t g_view_start = 0;  // first byte index in lineBuf that is visible

// Renderer/editor state
static size_t g_line_len = 0;
static size_t g_cursor = 0;
static size_t g_last_render_len = 0;

// Helpers
static inline size_t promptLen() {
  return strlen(PROMPT);
}
static inline size_t maxLineLen() {
  return sizeof(lineBuf) - 1;  // 4095 with your FS_SECTOR_SIZE=4096
}
static inline size_t visibleCols() {
  // visible columns for content (prompt consumes some columns)
  size_t p = promptLen();
  if (g_term_cols <= p + 8) return 16;  // fallback if terminal is tiny or unknown
  size_t v = g_term_cols - p;
  if (v < 8) v = 8;
  return v;
}
static void historyPush(const char* s, size_t len) {
  if (!s || len == 0) return;
  // Avoid duplicate of last entry
  if (g_history_count > 0) {
    const char* last = g_history[g_history_count - 1];
    if (strncmp(last, s, HISTORY_ENTRY_MAX - 1) == 0 && strlen(last) == len) return;
  }
  if (g_history_count < HISTORY_MAX) {
    // append
    size_t L = (len < (HISTORY_ENTRY_MAX - 1)) ? len : (HISTORY_ENTRY_MAX - 1);
    memcpy(g_history[g_history_count], s, L);
    g_history[g_history_count][L] = 0;
    g_history_count++;
  } else {
    // shift left
    for (int i = 1; i < HISTORY_MAX; ++i) {
      strncpy(g_history[i - 1], g_history[i], HISTORY_ENTRY_MAX);
      g_history[i - 1][HISTORY_ENTRY_MAX - 1] = 0;
    }
    size_t L = (len < (HISTORY_ENTRY_MAX - 1)) ? len : (HISTORY_ENTRY_MAX - 1);
    memcpy(g_history[HISTORY_MAX - 1], s, L);
    g_history[HISTORY_MAX - 1][L] = 0;
  }
}
static void adjustViewToCursor() {
  size_t vcols = visibleCols();
  // Ensure cursor is within [g_view_start, g_view_start + vcols - 1]
  if (g_cursor < g_view_start) {
    g_view_start = g_cursor;
  } else if (g_cursor >= g_view_start + vcols) {
    g_view_start = g_cursor - (vcols - 1);
  }
  // If the tail got shorter, try to bring view back left so we don’t leave lots of empty cols
  size_t maxStart = (g_line_len > vcols) ? (g_line_len - vcols) : 0;
  if (g_view_start > maxStart) g_view_start = maxStart;
}
static void renderLine() {
  adjustViewToCursor();

  // Clear current row and repaint compact window
  Serial.print("\r\x1b[2K");  // CR + clear entire line
  Serial.print(PROMPT);

  size_t vcols = visibleCols();
  if (g_line_len > 0) {
    size_t visLen = (g_line_len > g_view_start) ? (g_line_len - g_view_start) : 0;
    if (visLen > vcols) visLen = vcols;
    if (visLen) Serial.write(lineBuf + g_view_start, visLen);
  }

  // Place cursor at logical position within the single row
  size_t col = promptLen() + (g_cursor - g_view_start);
  Serial.print("\r\x1b[");
  Serial.print((unsigned)col);
  Serial.print('C');

  g_last_render_len = g_line_len;
}
static void setLineFrom(const char* s) {
  if (!s) s = "";
  size_t L = strlen(s);
  if (L > maxLineLen()) L = maxLineLen();
  memcpy(lineBuf, s, L);
  g_line_len = L;
  lineBuf[g_line_len] = 0;
  g_cursor = g_line_len;
  g_view_start = (g_line_len > visibleCols()) ? (g_line_len - visibleCols()) : 0;
  renderLine();
}
static void beginBrowseIfNeeded() {
  if (g_history_browse == -1) {
    // save current edit buffer (for restore on down past newest)
    size_t L = (g_line_len < (HISTORY_ENTRY_MAX - 1)) ? g_line_len : (HISTORY_ENTRY_MAX - 1);
    memcpy(g_saved_before_browse, lineBuf, L);
    g_saved_before_browse[L] = 0;
    g_saved_before_browse_len = L;
  }
}
static void browseUp() {
  if (g_history_count == 0) return;
  beginBrowseIfNeeded();
  if (g_history_browse < (g_history_count - 1)) {
    g_history_browse++;
    int idx = g_history_count - 1 - g_history_browse;
    setLineFrom(g_history[idx]);
  }
}
static void browseDown() {
  if (g_history_browse < 0) return;
  if (g_history_browse > 0) {
    g_history_browse--;
    int idx = g_history_count - 1 - g_history_browse;
    setLineFrom(g_history[idx]);
  } else {
    // exit browse, restore saved
    g_history_browse = -1;
    size_t L = (g_saved_before_browse_len < sizeof(lineBuf) - 1) ? g_saved_before_browse_len : (sizeof(lineBuf) - 1);
    memcpy(lineBuf, g_saved_before_browse, L);
    g_line_len = L;
    lineBuf[g_line_len] = 0;
    g_cursor = g_line_len;
    g_view_start = (g_line_len > visibleCols()) ? (g_line_len - visibleCols()) : 0;
    renderLine();
  }
}
enum EscState {
  ES_Normal,
  ES_GotEsc,
  ES_CSI,
  ES_SS3,
  ES_CSI_Param
};
static bool readLine() {
  static EscState esc = ES_Normal;
  static int csi_param = 0;
  bool lineReady = false;
  while (Serial.available()) {
    int ic = Serial.read();
    if (ic < 0) break;
    char c = (char)ic;
    if (esc == ES_Normal) {
      if (c == '\r' || c == '\n') {
        // Eat CRLF combo
        if (c == '\r') {
          if (Serial.available()) {
            int p = Serial.peek();
            if (p == '\n') Serial.read();
          }
        }
        Serial.write('\r');
        Serial.write('\n');
        // Commit line
        lineBuf[g_line_len] = 0;
        // Push to history (non-empty)
        if (g_line_len > 0) historyPush(lineBuf, g_line_len);
        // Reset browsing state
        g_history_browse = -1;
        g_saved_before_browse[0] = 0;
        g_saved_before_browse_len = 0;
        // Reset render state for new prompt
        g_last_render_len = 0;
        g_cursor = 0;
        g_line_len = 0;
        g_view_start = 0;
        lineReady = true;
        break;
      } else if ((c == 0x08) || (c == 0x7F)) {
        // Backspace
        if (g_cursor > 0) {
          for (size_t i = g_cursor - 1; i + 1 < g_line_len; ++i) lineBuf[i] = lineBuf[i + 1];
          g_line_len--;
          g_cursor--;
          lineBuf[g_line_len] = 0;
          renderLine();
        } else {
          Serial.write('\a');
        }
        continue;
      } else if (c == 0x1B) {
        esc = ES_GotEsc;
        csi_param = 0;
        continue;
      } else if (c == '\t') {
        // Simple tab: insert 2 spaces
        if (g_line_len + 2 <= maxLineLen()) {
          for (size_t i = g_line_len + 1; i >= g_cursor + 2; --i) lineBuf[i] = lineBuf[i - 2];
          lineBuf[g_cursor] = ' ';
          lineBuf[g_cursor + 1] = ' ';
          g_cursor += 2;
          g_line_len += 2;
          lineBuf[g_line_len] = 0;
          renderLine();
        } else {
          Serial.write('\a');
        }
        continue;
      } else if (c == 0x01) {
        // Ctrl+A -> Home
        g_cursor = 0;
        renderLine();
        continue;
      } else if (c == 0x05) {
        // Ctrl+E -> End
        g_cursor = g_line_len;
        renderLine();
        continue;
      } else if (c == 0x0B) {
        // Ctrl+K -> kill to end
        g_line_len = g_cursor;
        lineBuf[g_line_len] = 0;
        renderLine();
        continue;
      } else if (c == 0x15) {
        // Ctrl+U -> clear line
        g_cursor = 0;
        g_line_len = 0;
        lineBuf[0] = 0;
        g_view_start = 0;
        renderLine();
        continue;
      } else if (c == 0x0C) {
        // Ctrl+L -> redraw line (simple)
        Serial.println();
        renderLine();
        continue;
      } else if (c >= 32 && c <= 126) {
        // Printable: insert at cursor
        if (g_line_len + 1 <= maxLineLen()) {
          for (size_t i = g_line_len; i > g_cursor; --i) lineBuf[i] = lineBuf[i - 1];
          lineBuf[g_cursor] = c;
          g_line_len++;
          g_cursor++;
          lineBuf[g_line_len] = 0;
          renderLine();
        } else {
          Serial.write('\a');
        }
        continue;
      } else {
        // ignore others
        continue;
      }
    } else if (esc == ES_GotEsc) {
      if (c == '[') {
        esc = ES_CSI;
      } else if (c == 'O') {
        esc = ES_SS3;  // function key like Home/End variants
      } else {
        esc = ES_Normal;
      }
    } else if (esc == ES_SS3) {
      // Expect 'H' (Home) or 'F' (End)
      if (c == 'H') {
        g_cursor = 0;
        renderLine();
      } else if (c == 'F') {
        g_cursor = g_line_len;
        renderLine();
      }
      esc = ES_Normal;
    } else if (esc == ES_CSI) {
      if (c >= '0' && c <= '9') {
        csi_param = (csi_param * 10) + (c - '0');
        esc = ES_CSI_Param;
      } else {
        // single-char CSI
        if (c == 'A') {
          // Up
          browseUp();
        } else if (c == 'B') {
          // Down
          browseDown();
        } else if (c == 'C') {
          // Right
          if (g_cursor < g_line_len) {
            g_cursor++;
            renderLine();
          }
        } else if (c == 'D') {
          // Left
          if (g_cursor > 0) {
            g_cursor--;
            renderLine();
          }
        } else if (c == 'H') {
          g_cursor = 0;
          renderLine();
        } else if (c == 'F') {
          g_cursor = g_line_len;
          renderLine();
        }
        esc = ES_Normal;
        csi_param = 0;
      }
    } else if (esc == ES_CSI_Param) {
      if (c >= '0' && c <= '9') {
        csi_param = (csi_param * 10) + (c - '0');
      } else if (c == '~') {
        // well-known: 3~ = Delete
        if (csi_param == 3) {
          if (g_cursor < g_line_len) {
            for (size_t i = g_cursor; i + 1 < g_line_len; ++i) lineBuf[i] = lineBuf[i + 1];
            g_line_len--;
            lineBuf[g_line_len] = 0;
            renderLine();
          } else {
            Serial.write('\a');
          }
        } else if (csi_param == 1 || csi_param == 7) {
          g_cursor = 0;
          renderLine();
        } else if (csi_param == 4 || csi_param == 8) {
          g_cursor = g_line_len;
          renderLine();
        }
        esc = ES_Normal;
        csi_param = 0;
      } else {
        esc = ES_Normal;
        csi_param = 0;
      }
    }
  }
  return lineReady;
}