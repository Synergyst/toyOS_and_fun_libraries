// main_mcu.ino - Running on WaveShare RP2040-Zero
#define COPROCLANG_COPY_INPUT 1
#include "ConsolePrint.h"
static ConsolePrint Console;  // Console wrapper
// ------- Shared HW SPI (FLASH + PSRAM + NAND) -------
// Force low identification speed compatible with the bridge
//#define UNIFIED_SPI_INSTANCE SPI1
#define UNIFIED_SPI_INSTANCE SPI
#define UNIFIED_SPI_CLOCK_HZ 104000000UL  // 104 MHz SPI
#define W25Q_SPI_CLOCK_HZ 104000000UL     // 104 MHz NOR
#define MX35_SPI_CLOCK_HZ 104000000UL     // 104 MHz NAND
#include "UnifiedSPIMemSimpleFS.h"
// ------- Custom scripting language interpreter -------
#include "LangInter.h"
// Store ASCII scripts as raw multi-line strings, expose pointer + length.
#ifndef DECLARE_ASCII_SCRIPT
#define DECLARE_ASCII_SCRIPT(name, literal) \
  static const char name##_ascii[] = literal; \
  const uint8_t* name = reinterpret_cast<const uint8_t*>(name##_ascii); \
  const unsigned int name##_len = (unsigned int)(sizeof(name##_ascii) - 1)
#endif
struct BlobReg;  // Forward declaration
#include "MCCompiler.h"
// ------- User preference configuration -------
#include "blob_dont.h"
#define FILE_DONT "dont"
#include "scripts.h"
#define FILE_BLINKSCRIPT "blinkscript"
#define FILE_ONSCRIPT "onscript"
#define FILE_SONG "song"
#define FILE_PT1 "pt1"
#define FILE_PT2 "pt2"
#define FILE_PT3 "pt3"
#include "shrxbin.h"
#include "rp_selfupdate.h"
#include "shsha256.h"
#include "shb64.h"
static shb64s::State g_b64;
#include "shfs.h"
#include "shline.h"
static shline::State g_le;
#include "shdiag.h"
#include "AudioWavOut.h"
#include "MidiPlayer.h"
static AudioWavOut Audio;
static MCP4921 MCP;
static MidiPlayer MIDI;

// ---- MIDI debug defaults (affect 'midi play' commands) ----
// Requires MidiPlayer.h updated with Debug enum and progress fields.
static MidiPlayer::Debug g_midi_debug = MidiPlayer::Debug::Info;  // Off|Errors|Info|Verbose
static bool g_midi_progress = true;                               // periodic progress printouts
static uint16_t g_midi_prog_ms = 250;                             // progress interval (ms)
static int g_midi_track_index = -1;                               // -1 = auto-select by most NoteOn
static bool g_midi_allow_q = true;                                // allow 'q' to stop playback

static shrxbin::State g_rxbin;

// ========== Pins for Memory Chips ==========
const uint8_t PIN_FLASH_MISO = 4;               // GP4
const uint8_t PIN_PSRAM_MISO = PIN_FLASH_MISO;  //
const uint8_t PIN_FLASH_MOSI = 3;               // GP3
const uint8_t PIN_PSRAM_MOSI = PIN_FLASH_MOSI;  //
const uint8_t PIN_FLASH_SCK = 6;                // GP6
const uint8_t PIN_PSRAM_SCK = PIN_FLASH_SCK;    //
const uint8_t PIN_FLASH_CS0 = 5;                // GP5
const uint8_t PIN_PSRAM_CS0 = 14;               // GP14
const uint8_t PIN_NAND_CS = 8;                  // GP8

// Audio/MCP pins
static const uint8_t PIN_DAC_CS = 7;   // MCP4921 CS on GP7
static const uint8_t PIN_BUZZER = 29;  // Buzzer/PWM on GP29

// ========== Flash/PSRAM instances (needed before bindActiveFs) ==========
// Persisted config you already have
const size_t PERSIST_LEN = 32;

UnifiedSpiMem::Manager uniMem(PIN_PSRAM_SCK, PIN_PSRAM_MOSI, PIN_PSRAM_MISO);  // Unified manager: one bus, many CS

// SimpleFS facades
W25QUnifiedSimpleFS fsFlash;
PSRAMUnifiedSimpleFS fsPSRAM;
MX35UnifiedSimpleFS fsNAND;
static bool g_dev_mode = true;

// ---- MIDI CLI helpers ----
static bool eqNoCase(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca + 32);
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb + 32);
    if (ca != cb) return false;
    ++a;
    ++b;
  }
  return *a == 0 && *b == 0;
}

static bool parseMidiDebug(const char* s, MidiPlayer::Debug& lvl) {
  if (!s) return false;
  if (eqNoCase(s, "off")) {
    lvl = MidiPlayer::Debug::Off;
    return true;
  }
  if (eqNoCase(s, "errors") || eqNoCase(s, "error")) {
    lvl = MidiPlayer::Debug::Errors;
    return true;
  }
  if (eqNoCase(s, "info")) {
    lvl = MidiPlayer::Debug::Info;
    return true;
  }
  if (eqNoCase(s, "verbose") || eqNoCase(s, "v")) {
    lvl = MidiPlayer::Debug::Verbose;
    return true;
  }
  return false;
}

static bool parseOnOff(const char* s, bool& v) {
  if (!s) return false;
  if (eqNoCase(s, "on") || eqNoCase(s, "1") || eqNoCase(s, "true")) {
    v = true;
    return true;
  }
  if (eqNoCase(s, "off") || eqNoCase(s, "0") || eqNoCase(s, "false")) {
    v = false;
    return true;
  }
  return false;
}

static const char* midiDebugName(MidiPlayer::Debug d) {
  switch (d) {
    case MidiPlayer::Debug::Off: return "off";
    case MidiPlayer::Debug::Errors: return "errors";
    case MidiPlayer::Debug::Info: return "info";
    case MidiPlayer::Debug::Verbose: return "verbose";
  }
  return "?";
}

// ========== Blob registry ==========
struct BlobReg {
  const char* id;
  const uint8_t* data;
  unsigned int len;
};

static const BlobReg g_blobs[] = {
  { FILE_DONT, blob_dont, blob_dont_len },
  { FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len },
  { FILE_ONSCRIPT, blob_onscript, blob_onscript_len },
  { FILE_SONG, blob_song, blob_song_len },
  { FILE_PT1, blob_pt1, blob_pt1_len },
  { FILE_PT2, blob_pt2, blob_pt2_len },
  { FILE_PT3, blob_pt3, blob_pt3_len },
};
static const size_t g_blobs_count = sizeof(g_blobs) / sizeof(g_blobs[0]);

// ========== FS helpers and console ==========
static bool checkNameLen(const char* name) {
  size_t n = strlen(name);
  if (n == 0 || n > ActiveFS::MAX_NAME) {
    Console.print("Error: filename length ");
    Console.print(n);
    Console.print(" exceeds max ");
    Console.print(ActiveFS::MAX_NAME);
    Console.println(". Use a shorter name.");
    return false;
  }
  return true;
}

static void listBlobs() {
  Console.println("Available blobs:");
  for (size_t i = 0; i < g_blobs_count; ++i) {
    Console.printf(" - %s  \t(%d bytes)\n", g_blobs[i].id, g_blobs[i].len);
  }
}

static const BlobReg* findBlob(const char* id) {
  for (size_t i = 0; i < g_blobs_count; ++i)
    if (strcmp(g_blobs[i].id, id) == 0) return &g_blobs[i];
  return nullptr;
}

static void dumpFileHead(const char* fname, uint32_t count) {
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz) || sz == 0) {
    Console.println("dump: missing/empty");
    return;
  }
  if (count > sz) count = sz;
  const size_t CHUNK = 32;
  uint8_t buf[CHUNK];
  uint32_t off = 0;
  Console.print(fname);
  Console.print(" size=");
  Console.println(sz);
  while (off < count) {
    size_t n = (count - off > CHUNK) ? CHUNK : (count - off);
    uint32_t got = activeFs.readFileRange(fname, off, buf, n);
    if (got != n) {
      Console.println("  read error");
      break;
    }
    Console.print("  ");
    for (size_t i = 0; i < n; ++i) {
      if (i) Console.print(' ');
      if (buf[i] < 0x10) Console.print('0');
      Console.print(buf[i], HEX);
    }
    Console.println();
    off += n;
  }
}

static bool ensureBlobFile(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = ActiveFS::SECTOR_SIZE) {
  if (!checkNameLen(fname)) return false;
  uint32_t eraseAlign = getEraseAlign();
  if (reserve < eraseAlign) reserve = eraseAlign;
  reserve = (len ? ((len + (eraseAlign - 1)) & ~(eraseAlign - 1)) : reserve);
  if (!activeFs.exists(fname)) {
    Console.print("Creating slot ");
    Console.print(fname);
    Console.print(" (");
    Console.print(reserve);
    Console.println(" bytes)...");
    if (activeFs.createFileSlot(fname, reserve, data, len)) {
      Console.println("Created and wrote blob");
      return true;
    }
    Console.println("Failed to create slot");
    return false;
  }
  uint32_t addr, size, cap;
  if (!activeFs.getFileInfo(fname, addr, size, cap)) {
    Console.println("getFileInfo failed");
    return false;
  }
  bool same = (size == len);
  if (same) {
    const size_t CHUNK = 64;
    uint8_t buf[CHUNK];
    uint32_t off = 0;
    while (off < size) {
      size_t n = (size - off > CHUNK) ? CHUNK : (size - off);
      activeFs.readFileRange(fname, off, buf, n);
      for (size_t i = 0; i < n; ++i) {
        if (buf[i] != data[off + i]) {
          same = false;
          break;
        }
      }
      if (!same) break;
      off += n;
      yield();
    }
  }
  if (same) {
    Console.println("Blob already up to date");
    return true;
  }
  if (cap >= len && activeFs.writeFileInPlace(fname, data, len, false)) {
    Console.println("Updated in place");
    return true;
  }
  if (activeFs.writeFile(fname, data, len, fsReplaceMode())) {
    Console.println("Updated by allocating new space");
    return true;
  }
  Console.println("Failed to update file");
  return false;
}

static bool ensureBlobIfMissing(const char* fname, const uint8_t* data, uint32_t len, uint32_t reserve = ActiveFS::SECTOR_SIZE) {
  if (!checkNameLen(fname)) return false;
  if (activeFs.exists(fname)) {
    Console.printf("Skipping: %s\n", fname);
    return true;
  }
  uint32_t eraseAlign = getEraseAlign();
  if (reserve < eraseAlign) reserve = eraseAlign;
  reserve = (len ? ((len + (eraseAlign - 1)) & ~(eraseAlign - 1)) : reserve);
  Console.printf("Auto-creating %s (%lu bytes)...\n", fname, (unsigned long)reserve);
  if (activeFs.createFileSlot(fname, reserve, data, len)) {
    Console.println("Created and wrote blob");
    return true;
  }
  Console.println("Auto-create failed");
  return false;
}

static void autogenBlobWrites() {
  bool allOk = true;
  allOk &= ensureBlobIfMissing(FILE_DONT, blob_dont, blob_dont_len);
  allOk &= ensureBlobIfMissing(FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len);
  allOk &= ensureBlobIfMissing(FILE_ONSCRIPT, blob_onscript, blob_onscript_len);
  allOk &= ensureBlobIfMissing(FILE_SONG, blob_song, blob_song_len);
  allOk &= ensureBlobIfMissing(FILE_PT1, blob_pt1, blob_pt1_len);
  allOk &= ensureBlobIfMissing(FILE_PT2, blob_pt2, blob_pt2_len);
  allOk &= ensureBlobIfMissing(FILE_PT3, blob_pt3, blob_pt3_len);
  Console.print("Autogen:  ");
  Console.println(allOk ? "OK" : "some failures");
}

static inline int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void printHexLower(const uint8_t* d, size_t n) {
  static const char* hexd = "0123456789abcdef";
  for (size_t i = 0; i < n; ++i) {
    Serial.write(hexd[d[i] >> 4]);
    Serial.write(hexd[d[i] & 0xF]);
  }
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
  uint8_t* buf = (uint8_t*)malloc(bytes ? bytes : 1);
  if (!buf) {
    Console.println("puthex: malloc failed");
    return false;
  }
  for (uint32_t i = 0; i < bytes; ++i) {
    int hi = hexVal(hex[2 * i + 0]);
    int lo = hexVal(hex[2 * i + 1]);
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

static bool decodeBase64String(const char* s, uint8_t*& out, uint32_t& outLen) {
  out = nullptr;
  outLen = 0;
  if (!s) return false;
  shb64s::initMap();  // was initB64MapOnce()

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
      int8_t v = shb64s::b64Map[c];  // was b64Map[c]
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

static bool sha256File(const char* fname, uint8_t out[32]) {
  uint32_t sz = 0;
  if (!activeFs.getFileSize(fname, sz)) {
    Console.println("hash: not found");
    return false;
  }
  shsha256::Ctx ctx;
  shsha256::init(&ctx);
  const size_t CHUNK = 1024;
  uint8_t buf[CHUNK];
  uint32_t off = 0;
  while (off < sz) {
    size_t n = (sz - off > CHUNK) ? CHUNK : (sz - off);
    uint32_t got = activeFs.readFileRange(fname, off, buf, n);
    if (got != n) {
      Console.println("hash: read error");
      return false;
    }
    shsha256::update(&ctx, buf, n);
    off += n;
    yield();
  }
  shsha256::final(&ctx, out);
  return true;
}

// ========== Compile Tiny-C source file -> raw Thumb binary on FS ==========
static void printCompileErrorContext(const char* src, size_t srcLen, size_t pos) {
  const size_t CONTEXT = 40;
  if (!src || srcLen == 0) return;
  size_t start = (pos > CONTEXT) ? (pos - CONTEXT) : 0;
  size_t end = (pos + CONTEXT < srcLen) ? (pos + CONTEXT) : srcLen;
  Console.println("----- context -----");
  for (size_t i = start; i < end; ++i) {
    char c = src[i];
    if (c == '\r') c = ' ';
    Console.print(c);
  }
  Console.println();
  size_t caret = pos - start;
  for (size_t i = 0; i < caret; ++i) Console.print(' ');
  Console.println("^");
  Console.println("-------------------");
}

static bool compileTinyCFileToFile(const char* srcName, const char* dstName) {
  if (!checkNameLen(srcName) || !checkNameLen(dstName)) return false;
  if (!activeFs.exists(srcName)) {
    Console.print("compile: source not found: ");
    Console.println(srcName);
    return false;
  }
  uint32_t srcSize = 0;
  if (!activeFs.getFileSize(srcName, srcSize) || srcSize == 0) {
    Console.println("compile: getFileSize failed or empty source");
    return false;
  }
  // Read source
  char* srcBuf = (char*)malloc(srcSize + 1);
  if (!srcBuf) {
    Console.println("compile: malloc src failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcName, (uint8_t*)srcBuf, srcSize);
  if (got != srcSize) {
    Console.println("compile: readFile failed");
    free(srcBuf);
    return false;
  }
  srcBuf[srcSize] = 0;

  // Prepare output buffer
  uint32_t outCap = (srcSize * 12u) + 256u;
  if (outCap < 512u) outCap = 512u;
  uint8_t* outBuf = (uint8_t*)malloc(outCap);
  if (!outBuf) {
    Console.println("compile: malloc out failed");
    free(srcBuf);
    return false;
  }

  MCCompiler comp;
  size_t outSize = 0;
  MCCompiler::Result r = comp.compile(srcBuf, srcSize, outBuf, outCap, &outSize);
  if (!r.ok) {
    Console.print("compile: error at pos ");
    Console.print((uint32_t)r.errorPos);
    Console.print(": ");
    Console.println(r.errorMsg ? r.errorMsg : "unknown");
    printCompileErrorContext(srcBuf, srcSize, r.errorPos);
    free(outBuf);
    free(srcBuf);
    return false;
  }

  // Write to destination file
  bool ok = writeBinaryToFS(dstName, outBuf, (uint32_t)outSize);
  if (ok) {
    Console.print("compile: OK -> ");
    Console.print(dstName);
    Console.print(" (");
    Console.print((uint32_t)outSize);
    Console.println(" bytes)");
    if (outSize & 1u) {
      Console.println("note: odd-sized output; for Thumb execution, even size is recommended.");
    }
  } else {
    Console.println("compile: write failed");
  }
  free(outBuf);
  free(srcBuf);
  return ok;
}

static bool devWriteAbs(void* ctx, uint32_t absAddr, const uint8_t* data, uint32_t len) {
  (void)ctx;
  UnifiedSpiMem::MemDevice* dev = nullptr;
  switch (g_storage) {
    case StorageBackend::Flash: dev = fsFlash.raw().device(); break;
    case StorageBackend::PSRAM_BACKEND: dev = fsPSRAM.raw().device(); break;
    case StorageBackend::NAND: dev = fsNAND.raw().device(); break;
    default: dev = nullptr; break;
  }
  if (!dev) return false;
  return dev->write(absAddr, data, len);
}
static bool devFinalizeSize(void* ctx, const char* name, uint32_t size, uint32_t baseAddr, uint32_t cap) {
  (void)ctx;
  (void)baseAddr;
  (void)cap;
  switch (g_storage) {
    case StorageBackend::Flash: return fsFlash.setFileSize(name, size);
    case StorageBackend::PSRAM_BACKEND: return fsPSRAM.setFileSize(name, size);
    case StorageBackend::NAND: return fsNAND.setFileSize(name, size);
    default: return false;
  }
}

// ========== Serial console / command handling ==========
static int nextToken(char*& p, char*& tok) {
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  if (!*p) {
    tok = nullptr;
    return 0;
  }
  tok = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  if (*p) {
    *p = 0;
    ++p;
  }
  return 1;
}

static void printHelp() {
  Console.println("Commands (filename max 32 chars):");
  Console.println("  help                         - this help");
  Console.println("  editor [on|off|auto|status]  - toggle/editing mode; Basic=monitor, Advanced=ANSI+history");
  Console.println("  storage                      - show active storage");
  Console.println("  storage flash|psram|nand     - switch storage backend");
  Console.println("  mode                         - show mode (dev/prod)");
  Console.println("  mode dev|prod                - set dev or prod mode");
  Console.println("  persist read                 - read persist file from active storage");
  Console.println("  persist write                - write persist file to active storage");
  Console.println("  blobs                        - list compiled-in blobs");
  Console.println("  autogen                      - auto-create enabled blobs if missing");
  Console.println("  files                        - list files in FS");
  Console.println("  info <file>                  - show file addr/size/cap");
  Console.println("  dump <file> <nbytes>         - hex dump head of file");
  Console.println("  mkSlot <file> <reserve>      - create sector-aligned slot");
  Console.println("  writeblob <file> <blobId>    - create/update file from blob");
  Console.println("  cat <file> [n]               - print file contents (text); default: entire file (truncates at 4096)");
  Console.println("  cp <src> <dst|folder/> [-f]  - copy file; -f overwrites destination");
  Console.println("  fscp <sFS:path> <dFS:path|folder/> [-f] - copy across filesystems (FS=flash|psram|nand)");
  Console.println("  del <file>                   - delete a file");
  Console.println("  rm <file>                    - alias for 'del'");
  Console.println("  format                       - format active FS");
  Console.println("  wipe                         - erase active chip (DANGEROUS to FS)");
  Console.println("  wipereboot                   - erase chip then reboot (DANGEROUS to FS)");
  Console.println("  wipebootloader               - erase chip then reboot to bootloader (DANGEROUS to FS)");
  Console.println("  meminfo                      - show heap/stack info");
  Console.println("  psramsmoketest               - safe, non-destructive PSRAM test");
  Console.println("  reboot                       - reboot the MCU");
  Console.println("  timeout [ms]                 - show or set core1 timeout override (0=defaults)");
  Console.println("  puthex <file> <hex>          - upload binary as hex string");
  Console.println("  putb64 <file> <base64>       - upload binary as base64");
  Console.println("  putb64s <file> [expected]    - paste base64; end with ESC[201~ (auto), Ctrl-D, or a line '.'");
  Console.println("  hash <file> [sha256]         - print SHA-256 of file");
  Console.println("  sha256 <file>                - alias for 'hash <file>'");
  Console.println("  cc <src> <dst>               - compile Tiny-C source file to binary");
  Console.println("  history                      - print recent command history");
  Console.println("  termwidth [cols]             - show or set terminal width used by line editor");
  Console.println();
  Console.println("Editor (Nano-style) commands:");
  Console.println("  nano <file>                  - open Nano-like editor");
  Console.println("  edit <file>                  - alias for 'nano'");
  Console.println();
  Console.println("Audio and MIDI (WAV/MCP4921/PWM):");
  Console.println("  wav play <file>               - play unsigned 8-bit mono WAV via MCP4921 DAC");
  Console.println("  wav playpwm <file> [pin]      - play unsigned 8-bit mono WAV via PWM (default GP29)");
  Console.println("  wav playpwmdac <file> [gain%] - play unsigned 8-bit mono WAV via DAC w/ fixed gain");
  Console.println("  wav dacperf                   - run MCP4921 DAC performance test");
  Console.println();
  Console.println("  midi status                   - show MIDI debug/progress settings");
  Console.println("  midi debug <off|errors|info|verbose>");
  Console.println("  midi progress <on|off> [intervalMs]");
  Console.println("  midi track <auto|-1|index>");
  Console.println("  midi q <on|off>               - enable/disable 'q' to stop");
  Console.println("  midi play buzz <file> [gapMs]                                      - buzzer (GPIO toggle square)");
  Console.println("  midi play pwm <file> [gapMs] [pin]                                 - PWM square (50% duty)");
  Console.println("  midi play dac <file> [gapMs] [wave=sine|square|saw] [amp%] [sr]    - DAC synth");
  Console.println("  midi play pwmdac <file> [gapMs] [wave=sine|square|saw] [amp%] [sr] - alias of 'dac'");
  Console.println();
  Console.println("  tone <pin> <half_us> <cycles> - simple digital tone (GPIO toggle)");
  Console.println();
  Console.println("Notes:");
  Console.println("  - Default buzzer pin: GP29 (PWM). DAC CS: GP7.");
  Console.println("  - MCP4921 shares SPI with storage (SCK=GP6, MOSI=GP3, MISO=GP4).");
  Console.println("  - Press 'q' during audio/MIDI playback to stop.");
  Console.println("  - WAV supported format: RIFF PCM, unsigned 8-bit, mono.");
  Console.println();
  Console.println("Folders (SimpleFS path emulation; full path <= 32 chars):");
  Console.println("  pwd                         - show current folder (\"/\" = root)");
  Console.println("  cd / | cd .. | cd .         - change folder (root, parent, stay)");
  Console.println("  cd <path>                   - change to relative or absolute path");
  Console.println("  mkdir <path>                - create folder marker");
  Console.println("  ls [path]                   - list current or specified folder");
  Console.println("  rmdir <path> [-r]           - remove folder; -r deletes all children");
  Console.println("  touch <path|name|folder/>   - create empty file or folder marker");
  Console.println("  df                          - show device and FS usage");
  Console.println("  mv <src> <dst|folder/>      - move/rename file");
  Console.println();
  Console.println("Linux hints: base64 -w0 your.bin  |  xxd -p -c 999999 your.bin | tr -d '\\n'");
}

#include "TextEditor.h"

static void handleCommand(char* line) {
  char* p = line;
  char* t0;
  if (!nextToken(p, t0)) return;

  if (!strcmp(t0, "help")) {
    printHelp();

  } else if (!strcmp(t0, "nano") || !strcmp(t0, "edit")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: nano <file>");
      return;
    }
    if (!checkNameLen(fn)) return;
    Console.printf("Opening editor for '%s'...\n", fn);
    runNanoEditor(fn);
    Console.println("Returned to console.");

  } else if (!strcmp(t0, "editor")) {
    char* arg = nullptr;
    if (!nextToken(p, arg) || !strcmp(arg, "status")) {
      Console.print("editor: ");
      Console.println(shline::mode(g_le) == shline::Mode::Advanced ? "on (ANSI)" : "off (basic)");
      return;
    }
    if (!strcmp(arg, "on")) {
      shline::setMode(g_le, shline::Mode::Advanced);
      Console.println("editor: on (ANSI)");
    } else if (!strcmp(arg, "off")) {
      shline::setMode(g_le, shline::Mode::Basic);
      Console.println("editor: off (basic)");
    } else if (!strcmp(arg, "auto")) {
      bool ok = shline::probeAnsiSupport(g_le);
      shline::setMode(g_le, ok ? shline::Mode::Advanced : shline::Mode::Basic);
      Console.print("editor: ");
      Console.println(ok ? "on (ANSI detected)" : "off (no ANSI)");
    } else {
      Console.println("usage: editor [on|off|auto|status]");
    }
    return;

  } else if (!strcmp(t0, "history")) {
    int count = g_le.history_count;
    for (int i = 0; i < count; ++i) {
      Console.printf("  %2d  %s\n", i + 1, g_le.history[i]);
    }

  } else if (!strcmp(t0, "termwidth")) {
    char* cstr;
    if (!nextToken(p, cstr)) {
      Console.print("termwidth: ");
      Console.print((unsigned)g_le.term_cols);
      Console.println(" cols");
    } else {
      size_t cols = (size_t)strtoul(cstr, nullptr, 0);
      shline::setTerminalCols(g_le, cols);
      Console.print("termwidth set to ");
      Console.print((unsigned)g_le.term_cols);
      Console.println(" cols");
    }

  } else if (!strcmp(t0, "storage")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.print("Active storage: ");
      switch (g_storage) {
        case StorageBackend::Flash: Console.println("flash"); break;
        case StorageBackend::PSRAM_BACKEND: Console.println("psram"); break;
        case StorageBackend::NAND: Console.println("nand"); break;
        default: Console.println("unknown"); break;
      }
      return;
    }
    if (!strcmp(tok, "flash")) {
      g_storage = StorageBackend::Flash;
      bindActiveFs(g_storage);
      bool ok = activeFs.mount(true);
      {
        // Re-attach FS to Audio (and MIDI) so callbacks point to the new active backend
        AudioWavOut::FS afs{};
        afs.exists = activeFs.exists;
        afs.getFileSize = activeFs.getFileSize;
        afs.readFileRange = activeFs.readFileRange;
        Audio.attachFS(afs);

        MidiPlayer::FS mfs{};
        mfs.exists = activeFs.exists;
        mfs.getFileSize = activeFs.getFileSize;
        mfs.readFile = activeFs.readFile;
        mfs.readFileRange = activeFs.readFileRange;
        MIDI.attachFS(mfs);
      }
      Console.println("Switched active storage to FLASH");
      Console.println(ok ? "Mounted FLASH (auto-format if empty)" : "Mount failed (FLASH)");
    } else if (!strcmp(tok, "psram")) {
      g_storage = StorageBackend::PSRAM_BACKEND;
      bindActiveFs(g_storage);
      bool ok = activeFs.mount(false);
      {
        // Re-attach FS to Audio (and MIDI) so callbacks point to the new active backend
        AudioWavOut::FS afs{};
        afs.exists = activeFs.exists;
        afs.getFileSize = activeFs.getFileSize;
        afs.readFileRange = activeFs.readFileRange;
        Audio.attachFS(afs);

        MidiPlayer::FS mfs{};
        mfs.exists = activeFs.exists;
        mfs.getFileSize = activeFs.getFileSize;
        mfs.readFile = activeFs.readFile;
        mfs.readFileRange = activeFs.readFileRange;
        MIDI.attachFS(mfs);
      }
      Console.println("Switched active storage to PSRAM");
      Console.println(ok ? "Mounted PSRAM (no auto-format)" : "Mount failed (PSRAM)");
    } else if (!strcmp(tok, "nand")) {
      g_storage = StorageBackend::NAND;
      bindActiveFs(g_storage);
      bool ok = activeFs.mount(true);
      {
        // Re-attach FS to Audio (and MIDI) so callbacks point to the new active backend
        AudioWavOut::FS afs{};
        afs.exists = activeFs.exists;
        afs.getFileSize = activeFs.getFileSize;
        afs.readFileRange = activeFs.readFileRange;
        Audio.attachFS(afs);

        MidiPlayer::FS mfs{};
        mfs.exists = activeFs.exists;
        mfs.getFileSize = activeFs.getFileSize;
        mfs.readFile = activeFs.readFile;
        mfs.readFileRange = activeFs.readFileRange;
        MIDI.attachFS(mfs);
      }
      Console.println("Switched active storage to NAND");
      Console.println(ok ? "Mounted NAND (auto-format if empty)" : "Mount failed (NAND)");
    } else {
      Console.println("usage: storage [flash|psram|nand]");
    }

  } else if (!strcmp(t0, "mode")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.print("Mode: ");
      Console.println(g_dev_mode ? "dev" : "prod");
      return;
    }
    if (!strcmp(tok, "dev")) {
      g_dev_mode = true;
      Console.println("mode=dev");
    } else if (!strcmp(tok, "prod")) {
      g_dev_mode = false;
      Console.println("mode=prod");
    } else Console.println("usage: mode [dev|prod]");

  } else if (!strcmp(t0, "persist")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.println("usage: persist [read|write]");
      return;
    }
    if (!strcmp(tok, "read")) {
      const char* pf = ".persist";
      if (!activeFs.exists(pf)) {
        Console.println("persist read: not found");
        return;
      }
      uint32_t sz = 0;
      activeFs.getFileSize(pf, sz);
      uint8_t buf[PERSIST_LEN];
      memset(buf, 0, PERSIST_LEN);
      uint32_t got = activeFs.readFile(pf, buf, PERSIST_LEN);
      Console.printf("persist read: %lu bytes: ", (unsigned long)got);
      for (size_t i = 0; i < got; ++i) {
        if (i) Console.print(' ');
        if (buf[i] < 0x10) Console.print('0');
        Console.print(buf[i], HEX);
      }
      Console.println();
    } else if (!strcmp(tok, "write")) {
      const char* pf = ".persist";
      uint8_t buf[PERSIST_LEN];
      for (size_t i = 0; i < PERSIST_LEN; ++i) buf[i] = (uint8_t)(i ^ (g_dev_mode ? 0xA5 : 0x5A));
      if (!activeFs.exists(pf)) {
        if (!activeFs.createFileSlot(pf, ActiveFS::SECTOR_SIZE, buf, PERSIST_LEN)) {
          Console.println("persist write: create failed");
          return;
        }
      } else {
        if (!activeFs.writeFileInPlace(pf, buf, PERSIST_LEN, true)) {
          if (!activeFs.writeFile(pf, buf, PERSIST_LEN, fsReplaceMode())) {
            Console.println("persist write: write failed");
            return;
          }
        }
      }
      Console.println("persist write: OK");
    } else {
      Console.println("usage: persist [read|write]");
    }

  } else if (!strcmp(t0, "autogen")) {
    autogenBlobWrites();

  } else if (!strcmp(t0, "files")) {
    activeFs.listFilesToSerial();

  } else if (!strcmp(t0, "info")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: info <file>");
      return;
    }
    uint32_t a, s, c;
    if (activeFs.getFileInfo(fn, a, s, c)) {
      Console.print(fn);
      Console.print(": addr=0x");
      Console.print(a, HEX);
      Console.print(" size=");
      Console.print(s);
      Console.print(" cap=");
      Console.println(c);
    } else Console.println("not found");

  } else if (!strcmp(t0, "dump")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: dump <file> <nbytes>");
      return;
    }
    dumpFileHead(fn, (uint32_t)strtoul(nstr, nullptr, 0));

  } else if (!strcmp(t0, "mkSlot")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: mkSlot <file> <reserve>");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint32_t res = (uint32_t)strtoul(nstr, nullptr, 0);
    if (activeFs.createFileSlot(fn, res, nullptr, 0)) Console.println("slot created");
    else Console.println("mkSlot failed");

  } else if (!strcmp(t0, "writeblob")) {
    char* fn;
    char* bid;
    if (!nextToken(p, fn) || !nextToken(p, bid)) {
      Console.println("usage: writeblob <file> <blobId>");
      return;
    }
    if (!checkNameLen(fn)) return;
    const BlobReg* br = findBlob(bid);
    if (!br) {
      Console.println("unknown blobId; use 'blobs'");
      return;
    }
    if (ensureBlobFile(fn, br->data, br->len)) Console.println("writeblob OK");
    else Console.println("writeblob failed");

  } else if (!strcmp(t0, "cat")) {
    char* fn;
    char* nstr;
    uint32_t limit = 0xFFFFFFFFu;
    if (!nextToken(p, fn)) {
      Console.println("usage: cat <file> [n]");
      return;
    }
    if (nextToken(p, nstr)) limit = (uint32_t)strtoul(nstr, nullptr, 0);
    if (!checkNameLen(fn)) return;

    uint32_t sz = 0;
    if (!activeFs.getFileSize(fn, sz)) {
      Console.println("cat: not found");
      return;
    }
    if (limit == 0xFFFFFFFFu) {
      if (sz > 4096) {
        limit = 4096;
        Console.println("(cat truncated to 4096 bytes; use 'dump' to hex-dump larger files)");
      } else {
        limit = sz;
      }
    } else if (limit > sz) {
      limit = sz;
    }
    const size_t CHUNK = 128;
    uint8_t buf[CHUNK];
    uint32_t off = 0;
    while (off < limit) {
      size_t n = (limit - off > CHUNK) ? CHUNK : (limit - off);
      uint32_t got = activeFs.readFileRange(fn, off, buf, n);
      if (got != n) {
        Console.println("\ncat: read error");
        break;
      }
      for (size_t i = 0; i < n; ++i) Serial.write(buf[i]);
      off += n;
      yield();
    }
    Serial.write('\n');

  } else if (!strcmp(t0, "cp")) {
    // cp <src> <dst|folder/> [-f]
    char* srcArg;
    char* dstArg;
    char* opt;
    bool force = false;
    if (!nextToken(p, srcArg) || !nextToken(p, dstArg)) {
      Console.println("usage: cp <src> <dst|folder/> [-f]");
      return;
    }
    if (nextToken(p, opt)) {
      if (!strcmp(opt, "-f")) force = true;
      else {
        Console.println("usage: cp <src> <dst|folder/> [-f]");
        return;
      }
    }
    if (!cmdCpImpl(g_cwd, srcArg, dstArg, force)) {
      Console.println("cp failed");
    }

  } else if (!strcmp(t0, "fscp")) {
    // fscp <srcFS:path> <dstFS:path|folder/> [-f]
    char* srcSpec;
    char* dstSpec;
    char* opt;
    bool force = false;
    if (!nextToken(p, srcSpec) || !nextToken(p, dstSpec)) {
      Console.println("usage: fscp <sFS:path> <dFS:path|folder/> [-f]");
      Console.println("       FS = flash | psram | nand");
      return;
    }
    if (nextToken(p, opt)) {
      if (!strcmp(opt, "-f")) force = true;
      else {
        Console.println("usage: fscp <sFS:path> <dFS:path|folder/> [-f]");
        return;
      }
    }
    if (!cmdFsCpImpl(srcSpec, dstSpec, force)) {
      Console.println("fscp failed");
    }

  } else if (!strcmp(t0, "blobs")) {
    listBlobs();

  } else if (!strcmp(t0, "rescanfs")) {
    size_t found = uniMem.rescan(50000UL);
    Console.printf("Unified scan: found %u device(s)\n", (unsigned)found);
    for (size_t i = 0; i < uniMem.detectedCount(); ++i) {
      const auto* di = uniMem.detectedInfo(i);
      if (!di) continue;
      Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes\n", di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName, (unsigned long long)di->capacityBytes);
    }

  } else if (!strcmp(t0, "meminfo")) {
    Console.println();
    Console.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Console.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Console.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Console.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Console.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Console.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
    shdiag::Diag::printStackInfo(Console);  // or Serial
    shdiag::Diag::psramPrintCapacityReport(uniMem, Console);

  } else if (!strcmp(t0, "psramsmoketest")) {
    shdiag::Diag::psramPrintCapacityReport(uniMem, Console);
    shdiag::Diag::psramSafeSmokeTest(fsPSRAM, Console);

  } else if (!strcmp(t0, "reboot")) {
    Console.printf("Rebooting..\n");
    delay(20);
    yield();
    rp2040.reboot();

  } else if (!strcmp(t0, "del") || !strcmp(t0, "rm")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: del|rm <file>");
      return;
    }
    char abs[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(abs, sizeof(abs), g_cwd, arg, /*wantTrailingSlash*/ false)) {
      Console.println("del: path too long (<= 32 chars buffer)");
      return;
    }
    normalizePathInPlace(abs, /*wantTrailingSlash=*/false);
    bool ok = false;
    if (activeFs.exists && activeFs.exists(abs)) ok = activeFs.deleteFile(abs);
    if (!ok && strstr(abs, "//")) {
      char alt[sizeof abs];
      strncpy(alt, abs, sizeof alt);
      alt[sizeof alt - 1] = 0;
      normalizePathInPlace(alt, false);
      if (activeFs.exists && activeFs.exists(alt)) ok = activeFs.deleteFile(alt);
    }
    Console.println(ok ? "deleted" : "delete failed");

  } else if (!strcmp(t0, "format")) {
    if (activeFs.format && activeFs.format()) Console.println("FS formatted");
    else Console.println("format failed");

  } else if (!strcmp(t0, "wipebootloader")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Console.println("Chip wiped, rebooting to bootloader now..");
      delay(20);
      yield();
      rp2040.rebootToBootloader();
    } else Console.println("wipe failed");

  } else if (!strcmp(t0, "wipereboot")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Console.println("Chip wiped, rebooting now..");
      delay(20);
      yield();
      rp2040.reboot();
    } else Console.println("wipe failed");

  } else if (!strcmp(t0, "wipe")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) Console.println("Chip wiped");
    else Console.println("wipe failed");

  } else if (!strcmp(t0, "puthex")) {
    char* fn;
    char* hex;
    if (!nextToken(p, fn) || !nextToken(p, hex)) {
      Console.println("usage: puthex <file> <hex>");
      Console.println("tip: xxd -p -c 999999 your.bin | tr -d '\\n'");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeHexString(hex, bin, binLen)) {
      Console.println("puthex: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Console.print("puthex: wrote ");
      Console.print(binLen);
      Console.print(" bytes to ");
      Console.println(fn);
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
    } else Console.println("puthex: write failed");

  } else if (!strcmp(t0, "putb64")) {
    char* fn;
    char* b64;
    if (!nextToken(p, fn) || !nextToken(p, b64)) {
      Console.println("usage: putb64 <file> <base64>");
      Console.println("tip: base64 -w0 your.bin");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeBase64String(b64, bin, binLen)) {
      Console.println("putb64: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Console.print("putb64: wrote ");
      Console.print(binLen);
      Console.print(" bytes to ");
      Console.println(fn);
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
    } else Console.println("putb64: write failed");

  } else if (!strcmp(t0, "putb64s")) {
    char* fn;
    char* opt;
    if (!nextToken(p, fn)) {
      Console.println("usage: putb64s <file> [expected_decoded_size]");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint32_t expected = 0;
    if (nextToken(p, opt)) expected = (uint32_t)strtoul(opt, nullptr, 0);
    if (shb64s::active(g_b64)) {
      Console.println("putb64s: another upload is active");
      return;
    }
    // Begin streaming upload; shb64 interacts with Serial directly, like your old code
    shb64s::begin(g_b64, Serial, fn, expected, writeBinaryToFS);
    return;  // loop() will pump until completion

  } else if (!strcmp(t0, "hash") || !strcmp(t0, "sha256")) {
    char* fn;
    char* algo = nullptr;
    if (!nextToken(p, fn)) {
      Console.println("usage: hash <file> [sha256]");
      return;
    }
    // optional algo token for 'hash'; ignored for 'sha256'
    if (!strcmp(t0, "hash")) (void)nextToken(p, algo);
    if (algo && strcmp(algo, "sha256") != 0) {
      Console.println("hash: only sha256 is supported");
      return;
    }
    uint8_t dig[32];
    if (!sha256File(fn, dig)) return;
    printHexLower(dig, sizeof(dig));
    Serial.write(' ');
    Serial.println(fn);

  } else if (!strcmp(t0, "cc")) {
    char* src;
    char* dst;
    if (!nextToken(p, src) || !nextToken(p, dst)) {
      Console.println("usage: cc <src> <dst>");
      Console.println("example: cc myprog.c myprog.bin");
      return;
    }
    if (!compileTinyCFileToFile(src, dst)) {
      Console.println("cc: failed");
    }

  } else if (!strcmp(t0, "pwd")) {
    Console.print("cwd: /");
    Console.println(g_cwd);

  } else if (!strcmp(t0, "cd")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.print("cwd: /");
      Console.println(g_cwd);
      return;
    }
    if (!strcmp(arg, "/")) {
      g_cwd[0] = 0;
      Console.println("ok");
      return;
    }
    if (!strcmp(arg, ".")) {
      Console.println("ok");
      return;
    }
    if (!strcmp(arg, "..")) {
      pathParent(g_cwd);
      Console.println("ok");
      return;
    }
    char target[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(target, sizeof(target), g_cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("cd: path too long for SimpleFS (<=32 chars)");
      return;
    }
    if (!folderExists(target)) {
      Console.println("cd: no such folder (create with mkdir)");
      return;
    }
    target[strlen(target) - 1] = 0;
    strncpy(g_cwd, target, sizeof(g_cwd));
    g_cwd[sizeof(g_cwd) - 1] = 0;
    Console.println("ok");

  } else if (!strcmp(t0, "mkdir")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: mkdir <name|/abs|rel/path>");
      return;
    }
    if (strcmp(arg, "/") == 0 || arg[0] == 0) {
      Console.println("mkdir: refusing to create root");
      return;
    }
    char marker[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(marker, sizeof(marker), g_cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("mkdir: path too long for SimpleFS (<=32 chars)");
      return;
    }
    if (folderExists(marker)) {
      Console.println("mkdir: already exists");
      return;
    }
    if (mkdirFolder(marker)) Console.println("mkdir: ok");
    else Console.println("mkdir: failed");

  } else if (!strcmp(t0, "ls")) {
    char* arg;
    char folder[ActiveFS::MAX_NAME + 1] = { 0 };
    bool ok;
    if (!nextToken(p, arg)) {
      ok = pathJoin(folder, sizeof(folder), g_cwd, "", /*wantTrailingSlash*/ true);
    } else if (!strcmp(arg, "/")) {
      folder[0] = 0;
      ok = true;
    } else if (!strcmp(arg, ".")) {
      ok = pathJoin(folder, sizeof(folder), g_cwd, "", true);
    } else if (!strcmp(arg, "..")) {
      char parent[ActiveFS::MAX_NAME + 1];
      strncpy(parent, g_cwd, sizeof(parent));
      parent[sizeof(parent) - 1] = 0;
      pathParent(parent);
      ok = pathJoin(folder, sizeof(folder), parent, "", true);
    } else {
      ok = pathJoin(folder, sizeof(folder), g_cwd, arg, /*wantTrailingSlash*/ true);
    }
    if (!ok) {
      Console.println("ls: path too long");
      return;
    }
    FsIndexEntry idx[64];
    size_t n = buildFsIndex(idx, 64);
    Console.print("Listing /");
    Console.print(folder);
    Console.println(":");
    if (folder[0] == 0) {
      for (size_t i = 0; i < n; ++i) {
        if (idx[i].deleted) continue;
        if (strchr(idx[i].name, '/') == nullptr) {
          Console.print("  ");
          Console.print(idx[i].name);
          Console.print("  (");
          Console.print(idx[i].size);
          Console.println(" bytes)");
        }
      }
      const size_t MAX_SEEN = 32;
      char seen[MAX_SEEN][ActiveFS::MAX_NAME + 1];
      size_t seenCount = 0;
      auto seenPush = [&](const char* seg) {
        for (size_t k = 0; k < seenCount; ++k)
          if (strcmp(seen[k], seg) == 0) return false;
        if (seenCount < MAX_SEEN) {
          strncpy(seen[seenCount], seg, ActiveFS::MAX_NAME);
          seen[seenCount][ActiveFS::MAX_NAME] = 0;
          ++seenCount;
          return true;
        }
        return false;
      };
      for (size_t i = 0; i < n; ++i) {
        if (idx[i].deleted) continue;
        const char* slash = strchr(idx[i].name, '/');
        if (!slash) continue;
        char seg[ActiveFS::MAX_NAME + 1];
        size_t segLen = (size_t)(slash - idx[i].name);
        if (segLen > ActiveFS::MAX_NAME) segLen = ActiveFS::MAX_NAME;
        memcpy(seg, idx[i].name, segLen);
        seg[segLen] = 0;
        if (seenPush(seg)) {
          Console.print("  ");
          Console.print(seg);
          Console.println("/");
        }
      }
      return;
    }
    size_t folderLen = strlen(folder);
    if (folderExists(folder)) Console.println("  .");
    const size_t MAX_SEEN = 32;
    char seenChild[MAX_SEEN][ActiveFS::MAX_NAME + 1];
    size_t seenCount = 0;
    auto seenPush = [&](const char* seg) {
      for (size_t k = 0; k < seenCount; ++k)
        if (strcmp(seenChild[k], seg) == 0) return false;
      if (seenCount < MAX_SEEN) {
        strncpy(seenChild[seenCount], seg, ActiveFS::MAX_NAME);
        seenChild[seenCount][ActiveFS::MAX_NAME] = 0;
        ++seenCount;
        return true;
      }
      return false;
    };
    for (size_t i = 0; i < n; ++i) {
      if (idx[i].deleted) continue;
      if (strncmp(idx[i].name, folder, folderLen) != 0) continue;
      const char* rest = idx[i].name + folderLen;
      if (*rest == 0) continue;
      if (*rest == '/') ++rest;
      if (*rest == 0) continue;
      const char* slash = strchr(rest, '/');
      if (!slash) {
        Console.print("  ");
        Console.print(rest);
        Console.print("  (");
        Console.print(idx[i].size);
        Console.println(" bytes)");
      } else {
        if (slash == rest) continue;
        size_t segLen = (size_t)(slash - rest);
        char seg[ActiveFS::MAX_NAME + 1];
        if (segLen > ActiveFS::MAX_NAME) segLen = ActiveFS::MAX_NAME;
        memcpy(seg, rest, segLen);
        seg[segLen] = 0;
        if (seenPush(seg)) {
          Console.print("  ");
          Console.print(seg);
          Console.println("/");
        }
      }
    }

  } else if (!strcmp(t0, "lsdebug")) {
    char* nstr;
    uint32_t n = 256;
    if (nextToken(p, nstr)) n = (uint32_t)strtoul(nstr, nullptr, 0);
    dumpDirHeadRaw(n);

  } else if (!strcmp(t0, "rmdir")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: rmdir <name|path> [-r]");
      return;
    }
    bool recursive = false;
    char* opt;
    if (nextToken(p, opt) && !strcmp(opt, "-r")) recursive = true;

    char folder[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folder, sizeof(folder), g_cwd, arg, /*wantTrailingSlash*/ true)) {
      Console.println("rmdir: path too long");
      return;
    }
    FsIndexEntry idx[64];
    size_t n = buildFsIndex(idx, 64);
    if (n == 0) {
      Console.println("(ls fallback) Directory scan empty; printing files view:");
      activeFs.listFilesToSerial();
      return;
    }
    bool empty = true;
    for (size_t i = 0; i < n; ++i) {
      if (idx[i].deleted) continue;
      if (hasPrefix(idx[i].name, folder)) {
        if (strlen(idx[i].name) == strlen(folder)) continue;
        empty = false;
        break;
      }
    }
    if (!empty && !recursive) {
      Console.println("rmdir: not empty (use -r to remove all files under folder)");
      return;
    }
    if (recursive) {
      size_t delCount = 0;
      for (size_t i = 0; i < n; ++i) {
        if (idx[i].deleted) continue;
        if (hasPrefix(idx[i].name, folder)) {
          if (activeFs.deleteFile(idx[i].name)) delCount++;
          yield();
        }
      }
      Console.print("rmdir -r: deleted ");
      Console.print((unsigned)delCount);
      Console.println(" entries");
      return;
    }
    if (folderExists(folder)) {
      if (activeFs.deleteFile(folder)) Console.println("rmdir: ok");
      else Console.println("rmdir: failed to remove marker");
    } else {
      Console.println("rmdir: marker not found (already removed?)");
    }

  } else if (!strcmp(t0, "touch")) {
    char* pathArg;
    if (!nextToken(p, pathArg)) {
      Console.println("usage: touch <path|name|folder/>");
      return;
    }
    if (touchPath(g_cwd, pathArg)) {
      Console.println("ok");
    } else {
      Console.println("touch failed");
    }

  } else if (!strcmp(t0, "df")) {
    cmdDf();

  } else if (!strcmp(t0, "mv")) {
    char* srcArg;
    char* dstArg;
    if (!nextToken(p, srcArg) || !nextToken(p, dstArg)) {
      Console.println("usage: mv <src> <dst|folder/>");
      return;
    }
    if (!cmdMvImpl(g_cwd, srcArg, dstArg)) {
      Console.println("mv failed");
    }

  } else if (!strcmp(t0, "lsraw")) {
    FsIndexEntry idx[64];
    size_t n = buildFsIndex(idx, 64);
    for (size_t i = 0; i < n; ++i) {
      if (idx[i].deleted) continue;
      Console.printf("- %s  (%lu bytes)\n", idx[i].name, (unsigned long)idx[i].size);
    }

  } else if (!strcmp(t0, "wav")) {
    char* sub = nullptr;
    if (!nextToken(p, sub)) {
      Console.println("wav cmds:");
      Console.println("  wav play <file>              - unsigned 8-bit mono via MCP4921 DAC");
      Console.println("  wav playpwm <file> [pin]     - unsigned 8-bit mono via PWM (piezo)");
      Console.println("  wav playpwmdac <file> [gain%]- unsigned 8-bit mono via DAC w/ gain");
      Console.println("  wav dacperf                  - run MCP4921 performance test");
      return;
    }
    if (!strcmp(sub, "play")) {
      char* fn = nullptr;
      if (!nextToken(p, fn)) {
        Console.println("usage: wav play <file>");
        return;
      }
      if (!Audio.playWavDAC(fn)) Console.println("wav play: failed");
    } else if (!strcmp(sub, "playpwm")) {
      char* fn = nullptr;
      char* pinStr = nullptr;
      if (!nextToken(p, fn)) {
        Console.println("usage: wav playpwm <file> [pin]");
        return;
      }
      int pin = -1;
      if (nextToken(p, pinStr)) pin = (int)strtol(pinStr, nullptr, 0);
      if (!Audio.playWavPWM(fn, pin)) Console.println("wav playpwm: failed");
    } else if (!strcmp(sub, "playpwmdac")) {
      char* fn = nullptr;
      char* gstr = nullptr;
      if (!nextToken(p, fn)) {
        Console.println("usage: wav playpwmdac <file> [gain%]");
        return;
      }
      int gain = 50;
      if (nextToken(p, gstr)) gain = (int)strtol(gstr, nullptr, 0);
      if (!Audio.playWavPWMDAC(fn, gain)) Console.println("wav playpwmdac: failed");
    } else if (!strcmp(sub, "dacperf")) {
      Console.println("Running DAC performance test..");
      Audio.dacPerfTest();
    } else {
      Console.println("usage: wav play|playpwm|playpwmdac|dacperf");
    }

  } else if (!strcmp(t0, "tone")) {
    // tone <pin> <half_us> <cycles>
    char* pPin = nullptr;
    char* pHalf = nullptr;
    char* pCyc = nullptr;
    if (!nextToken(p, pPin) || !nextToken(p, pHalf) || !nextToken(p, pCyc)) {
      Console.println("usage: tone <pin> <half_us> <cycles>");
      return;
    }
    int pin = (int)strtol(pPin, nullptr, 0);
    int halfUs = (int)strtol(pHalf, nullptr, 0);
    int cyc = (int)strtol(pCyc, nullptr, 0);
    if (!Audio.toneDigital(pin, halfUs, cyc)) Console.println("tone failed");

  } else if (!strcmp(t0, "midi")) {
    char* sub = nullptr;
    if (!nextToken(p, sub)) {
      Console.println("midi cmds:");
      Console.println("  midi status");
      Console.println("  midi debug <off|errors|info|verbose>");
      Console.println("  midi progress <on|off> [intervalMs]");
      Console.println("  midi track <auto|-1|index>");
      Console.println("  midi q <on|off>           (enable 'q' to stop)");
      Console.println("  midi play buzz   <file> [gapMs]");
      Console.println("  midi play pwm    <file> [gapMs] [pin]");
      Console.println("  midi play dac    <file> [gapMs] [wave=sine|square|saw] [amp%] [sr]");
      Console.println("  midi play pwmdac <file> [gapMs] [wave=sine|square|saw] [amp%] [sr]");
      return;
    }
    if (!strcmp(sub, "status")) {
      Console.println("midi status:");
      Console.printf("  debug:     %s\n", midiDebugName(g_midi_debug));
      Console.printf("  progress:  %s, interval=%u ms\n", g_midi_progress ? "on" : "off", (unsigned)g_midi_prog_ms);
      Console.printf("  track:     %d (%s)\n", g_midi_track_index, (g_midi_track_index < 0) ? "auto" : "fixed");
      Console.printf("  allow 'q': %s\n", g_midi_allow_q ? "on" : "off");
      return;
    }
    if (!strcmp(sub, "debug")) {
      char* lev = nullptr;
      if (!nextToken(p, lev)) {
        Console.printf("midi debug: %s\n", midiDebugName(g_midi_debug));
        return;
      }
      MidiPlayer::Debug lvl;
      if (!parseMidiDebug(lev, lvl)) {
        Console.println("usage: midi debug <off|errors|info|verbose>");
        return;
      }
      g_midi_debug = lvl;
      Console.printf("midi debug set to %s\n", midiDebugName(g_midi_debug));
      return;
    }
    if (!strcmp(sub, "progress")) {
      char* onoff = nullptr;
      if (!nextToken(p, onoff)) {
        Console.printf("midi progress: %s, interval=%u ms\n", g_midi_progress ? "on" : "off", (unsigned)g_midi_prog_ms);
        return;
      }
      bool v;
      if (!parseOnOff(onoff, v)) {
        Console.println("usage: midi progress <on|off> [intervalMs]");
        return;
      }
      g_midi_progress = v;
      char* ms = nullptr;
      if (nextToken(p, ms)) {
        uint32_t x = (uint32_t)strtoul(ms, nullptr, 0);
        if (x == 0) x = 1;
        if (x > 5000) x = 5000;
        g_midi_prog_ms = (uint16_t)x;
      }
      Console.printf("midi progress: %s, interval=%u ms\n", g_midi_progress ? "on" : "off", (unsigned)g_midi_prog_ms);
      return;
    }
    if (!strcmp(sub, "track")) {
      char* idx = nullptr;
      if (!nextToken(p, idx)) {
        Console.printf("midi track: %d (%s)\n", g_midi_track_index, (g_midi_track_index < 0) ? "auto" : "fixed");
        return;
      }
      if (eqNoCase(idx, "auto") || !strcmp(idx, "-1")) {
        g_midi_track_index = -1;
      } else {
        g_midi_track_index = (int)strtol(idx, nullptr, 0);
        if (g_midi_track_index < -1) g_midi_track_index = -1;
      }
      Console.printf("midi track set to %d (%s)\n", g_midi_track_index, (g_midi_track_index < 0) ? "auto" : "fixed");
      return;
    }
    if (!strcmp(sub, "q")) {
      char* onoff = nullptr;
      if (!nextToken(p, onoff)) {
        Console.printf("midi q: %s\n", g_midi_allow_q ? "on" : "off");
        return;
      }
      bool v;
      if (!parseOnOff(onoff, v)) {
        Console.println("usage: midi q <on|off>");
        return;
      }
      g_midi_allow_q = v;
      Console.printf("midi q: %s\n", g_midi_allow_q ? "on" : "off");
      return;
    }

    // Work buffer (adjust to your expected file sizes; must hold file+8*notes)
    static uint8_t midiWork[128 * 1024];
    if (!strcmp(sub, "play")) {
      char* mode = nullptr;
      if (!nextToken(p, mode)) {
        Console.println("usage: midi play <buzz|pwm|dac|pwmdac> <file> ...");
        return;
      }
      char* fn = nullptr;
      if (!nextToken(p, fn)) {
        Console.println("usage: midi play <mode> <file> ...");
        return;
      }
      MidiPlayer::Config cfg;
      cfg.gapMs = 0;
      cfg.ledPin = 2;  // keep LED on GP2 as requested
      cfg.debug = g_midi_debug;
      cfg.debugProgress = g_midi_progress;
      cfg.debugIntervalMs = g_midi_prog_ms;
      cfg.trackIndex = g_midi_track_index;
      cfg.allowSerialQ = g_midi_allow_q;

      if (!strcmp(mode, "buzz")) {
        cfg.backend = MidiPlayer::Backend::BuzzerDigital;
        char* gap = nullptr;
        if (nextToken(p, gap)) cfg.gapMs = (int)strtol(gap, nullptr, 0);
        if (!MIDI.playSMF(fn, cfg, midiWork, sizeof(midiWork))) Console.println("midi buzz failed");

      } else if (!strcmp(mode, "pwm")) {
        cfg.backend = MidiPlayer::Backend::PWMSquare;
        cfg.outPin = PIN_BUZZER;  // default to GP29
        char* gap = nullptr;
        if (nextToken(p, gap)) cfg.gapMs = (int)strtol(gap, nullptr, 0);
        char* pinStr = nullptr;
        if (nextToken(p, pinStr)) cfg.outPin = (int)strtol(pinStr, nullptr, 0);
        if (!MIDI.playSMF(fn, cfg, midiWork, sizeof(midiWork))) Console.println("midi pwm failed");

      } else if (!strcmp(mode, "dac") || !strcmp(mode, "pwmdac")) {
        cfg.backend = !strcmp(mode, "dac") ? MidiPlayer::Backend::DAC : MidiPlayer::Backend::PWMDAC;
        cfg.waveform = MidiPlayer::Waveform::Sine;
        cfg.dacAmpPercent = 60;
        cfg.sampleRate = 22050;

        char* gap = nullptr;
        if (nextToken(p, gap)) cfg.gapMs = (int)strtol(gap, nullptr, 0);
        char* wstr = nullptr;
        if (nextToken(p, wstr)) {
          if (!strcmp(wstr, "sine")) cfg.waveform = MidiPlayer::Waveform::Sine;
          else if (!strcmp(wstr, "square")) cfg.waveform = MidiPlayer::Waveform::Square50;
          else if (!strcmp(wstr, "saw")) cfg.waveform = MidiPlayer::Waveform::Saw;
        }
        char* amp = nullptr;
        if (nextToken(p, amp)) cfg.dacAmpPercent = (int)strtol(amp, nullptr, 0);
        char* sr = nullptr;
        if (nextToken(p, sr)) cfg.sampleRate = (uint32_t)strtoul(sr, nullptr, 0);
        if (!MIDI.playSMF(fn, cfg, midiWork, sizeof(midiWork))) Console.println("midi dac/pwmdac failed");

      } else {
        Console.println("usage: midi play <buzz|pwm|dac|pwmdac> <file> [...]");
      }

    } else {
      Console.println("midi cmds:");
      Console.println("  midi status");
      Console.println("  midi debug <off|errors|info|verbose>");
      Console.println("  midi progress <on|off> [intervalMs]");
      Console.println("  midi track <auto|-1|index>");
      Console.println("  midi q <on|off>");
      Console.println("  midi play buzz   <file> [gapMs]");
      Console.println("  midi play pwm    <file> [gapMs] [pin]");
      Console.println("  midi play dac    <file> [gapMs] [wave=sine|square|saw] [amp%] [sr]");
      Console.println("  midi play pwmdac <file> [gapMs] [wave=sine|square|saw] [amp%] [sr]");
    }

  } else if (!strcmp(t0, "putbin")) {
    char* fn = nullptr;
    char* sz = nullptr;
    if (!nextToken(p, fn) || !nextToken(p, sz)) {
      Console.println("usage: putbin <file> <size>");
      return;
    }
    if (!checkNameLen(fn)) return;
    if (shrxbin::active(g_rxbin)) {
      Console.println("putbin: another transfer is active");
      return;
    }

    uint32_t total = (uint32_t)strtoul(sz, nullptr, 0);
    if (total == 0) {
      Console.println("putbin: size must be > 0");
      return;
    }

    // Prepare sector-aligned slot
    if (activeFs.exists && activeFs.exists(fn)) {
      if (!activeFs.deleteFile(fn)) {
        Console.println("putbin: failed to delete existing file");
        return;
      }
    }
    uint32_t eraseAlign = getEraseAlign();
    uint32_t reserve = (total + (eraseAlign - 1)) & ~(eraseAlign - 1);
    if (!activeFs.createFileSlot(fn, reserve, nullptr, 0)) {
      Console.println("putbin: createFileSlot failed");
      return;
    }

    // Get base/cap
    uint32_t base, size, cap;
    if (!activeFs.getFileInfo(fn, base, size, cap)) {
      Console.println("putbin: getFileInfo failed");
      return;
    }
    if (cap < total) {
      Console.println("putbin: slot cap < size");
      return;
    }

    shrxbin::Writer wr;
    wr.writeAbs = devWriteAbs;
    wr.finalizeSize = devFinalizeSize;
    wr.ctx = nullptr;
    wr.baseAddr = base;
    wr.cap = cap;

    if (!shrxbin::begin(g_rxbin, Serial, fn, total, wr)) {
      Console.println("putbin: begin failed");
      return;
    }
    return;
  } else {
    Console.println("Unknown command. Type 'help'.");
  }
}

// ========== Setup and main loops ==========
void setup() {
  delay(500);
  Serial.begin(2000000);
  while (!Serial) { delay(20); }
  delay(20);

  Console.println("System booting..");
  Console.begin();

  uniMem.begin();
  uniMem.setPreservePsramContents(true);
  //uniMem.setCsList({ PIN_FLASH_CS0, PIN_PSRAM_CS0, PIN_PSRAM_CS1, PIN_PSRAM_CS2, PIN_PSRAM_CS3, PIN_NAND_CS, PIN_FLASH_CS1 });
  //uniMem.setCsList({ PIN_NAND_CS });
  uniMem.setCsList({ PIN_FLASH_CS0, PIN_PSRAM_CS0 });

  // Keep slow scan for robustness
  size_t found = uniMem.rescan(50000UL);
  Console.printf("Unified scan: found %u device(s)\n", (unsigned)found);
  for (size_t i = 0; i < uniMem.detectedCount(); ++i) {
    const auto* di = uniMem.detectedInfo(i);
    if (!di) continue;
    Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes\n", di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName, (unsigned long long)di->capacityBytes);
  }

  bool nandOk = fsNAND.begin(uniMem);
  bool flashOk = fsFlash.begin(uniMem);
  bool psramOk = fsPSRAM.begin(uniMem);
  if (!nandOk) Console.println("NAND FS: no suitable device found or open failed");
  if (!flashOk) Console.println("Flash FS: no suitable device found or open failed");
  if (!psramOk) Console.println("PSRAM FS: no suitable device found or open failed");

  shfs_bindDevices(&fsFlash, &fsPSRAM, &fsNAND, &uniMem);
  shfs_setPrint(&Console);  // optional
  bindActiveFs(g_storage);

  bool mounted = activeFs.mount(g_storage == StorageBackend::Flash /*autoFormatIfEmpty*/);
  if (!mounted) {
    Console.println("FS mount failed on active storage");
  }

  // ---- Audio (WAV) + MIDI attach to FS/console/hooks ----
  // Audio
  {
    AudioWavOut::FS afs{};
    afs.exists = activeFs.exists;
    afs.getFileSize = activeFs.getFileSize;
    afs.readFileRange = activeFs.readFileRange;
    Audio.attachFS(afs);
    Audio.setConsole(&Console);
    Audio.setServiceHook([]() {
    });
    Audio.setCancelHook([]() -> bool {
      return false;
    });
    // PWM path defaults: buzzer on GP29, ~62.5kHz carrier, 8-bit range
    Audio.beginPWM(/*defaultPin*/ PIN_BUZZER, /*carrierHz*/ 62500, /*dutyRange*/ 127);
  }

  // MIDI
  {
    MidiPlayer::FS mfs{};
    mfs.exists = activeFs.exists;
    mfs.getFileSize = activeFs.getFileSize;
    mfs.readFile = activeFs.readFile;
    mfs.readFileRange = activeFs.readFileRange;  // not required but OK
    MIDI.attachFS(mfs);
    MIDI.setConsole(&Console);
    MIDI.setServiceHook([]() {
    });
    MIDI.setCancelHook([]() -> bool {
      return false;
    });
    MIDI.setMonitorSerialForQ(true);
    MIDI.beginPWM(255);
  }

  // ---- SPI bus alignment for MCP4921 on the SAME bus as UnifiedSpiMem ----
  // Important: set SPI pin mapping BEFORE SPI.begin(), so MCP_DAC uses SCK=6 MOSI=3 MISO=4 like your memories
  SPI.setSCK(PIN_FLASH_SCK);    // GP6
  SPI.setMOSI(PIN_FLASH_MOSI);  // GP3
  SPI.setMISO(PIN_FLASH_MISO);  // GP4
  SPI.begin();

  // Keep DAC CS high when idle to avoid spurious selects
  pinMode(PIN_DAC_CS, OUTPUT);
  digitalWrite(PIN_DAC_CS, HIGH);

  // MCP4921 on shared bus (CS=GP7). One init only.
  MCP.begin(PIN_DAC_CS);

  // Attach the same MCP DAC to both Audio and MIDI libraries
  Audio.attachMCP4921(&MCP);
  MIDI.attachMCP4921(&MCP);

  Console.print("Testing for Arduino IDE terminal (ignore artifacts)..");
  // Line editor: Basic by default (Arduino Serial Monitor)
  shline::begin(g_le, Serial, "> ");
  // Option A: manual selection (recommended), no ANSI
  //shline::setMode(g_le, shline::Mode::Basic);
  // Option B: try auto-probe and fall back
  if (shline::probeAnsiSupport(g_le)) shline::setMode(g_le, shline::Mode::Advanced);
  else shline::setMode(g_le, shline::Mode::Basic);
  Console.println(" done");

  Console.printf("System ready. Type 'help'\n");
  shline::printPrompt(g_le);
}
void setup1() {
}
void loop1() {
  tight_loop_contents();
}
void loop() {
  char cmdBuf[SHLINE_MAX_LINE];

  // If a binary upload is active, pump it and pause the console
  if (shrxbin::active(g_rxbin)) {
    shrxbin::pump(g_rxbin);
    return;
  }

  if (shb64s::active(g_b64)) {
    shb64s::pump(g_b64);
    return;
  }

  if (shline::poll(g_le, cmdBuf, sizeof(cmdBuf))) {
    handleCommand(cmdBuf);
    shline::printPrompt(g_le);
  }
}