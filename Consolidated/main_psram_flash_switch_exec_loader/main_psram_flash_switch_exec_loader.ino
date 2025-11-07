// main_psram_flash_switch_exec_loader.ino
#define COPROCLANG_COPY_INPUT 1
// ------- Shared HW SPI (FLASH + PSRAM) -------
#include "ConsolePrint.h"
// Force low identification speed compatible with the bridge
//#define UNIFIED_SPI_INSTANCE SPI1
#define UNIFIED_SPI_INSTANCE SPI
#define UNIFIED_SPI_CLOCK_HZ 104000000UL  // 104 MHz SPI
#define W25Q_SPI_CLOCK_HZ 104000000UL     // 104 MHz NOR
#define MX35_SPI_CLOCK_HZ 104000000UL     // 104 MHz NAND
#include "UnifiedSPIMemSimpleFS.h"
// ------- Co-Processor over Software Serial (framed RPC) -------
#include <SoftwareSerial.h>
#include "CoProcProto.h"
#include "CoProcLang.h"
#ifndef COPROC_BAUD
//#define COPROC_BAUD 230400
//#define COPROC_BAUD 115200
#define COPROC_BAUD 57600
#endif
#ifndef PIN_COPROC_RX
#define PIN_COPROC_RX 0  // GP0 (main RX)
#endif
#ifndef PIN_COPROC_TX
#define PIN_COPROC_TX 1  // GP1 (main TX)
#endif
static SoftwareSerial coprocLink(PIN_COPROC_RX, PIN_COPROC_TX, false);  // RX, TX, non-inverted
// Store ASCII scripts as raw multi-line strings, expose pointer + length.
#ifndef DECLARE_ASCII_SCRIPT
#define DECLARE_ASCII_SCRIPT(name, literal) \
  static const char name##_ascii[] = literal; \
  const uint8_t* name = reinterpret_cast<const uint8_t*>(name##_ascii); \
  const unsigned int name##_len = (unsigned int)(sizeof(name##_ascii) - 1)
#endif
struct BlobReg;               // Forward declaration
static ConsolePrint Console;  // Console wrapper
// ------- MC Compiiler -------
#include "MCCompiler.h"
// ------- User preference configuration -------
#include "blob_mailbox_config.h"  // Do not remove this line. This is the memory address where we handle the shared interprocess-mailbox-buffer
#include "blob_ret42.h"
#define FILE_RET42 "ret42"
#include "blob_add2.h"
#define FILE_ADD2 "add2"
#include "blob_pwmc.h"
#define FILE_PWMC "pwmc"
#include "blob_pwmc2350.h"
#define FILE_PWMC2350 "pwmc2350"
#include "blob_retmin.h"
#define FILE_RETMIN "retmin"
#include "blob_dont.h"
#define FILE_DONT "dont"
#include "scripts.h"
#define FILE_BLINKSCRIPT "blinkscript"
#define FILE_ONSCRIPT "onscript"
#define FILE_SONG "song"
#define FILE_PT1 "pt1"
#define FILE_PT2 "pt2"
#define FILE_PT3 "pt3"
#include <SPI.h>
#include "MCP_DAC.h"  // Only if you will use MCP4921 DAC output
#include "AudioWavOut.h"
#include "MidiPlayer.h"
static AudioWavOut Audio;
static MCP4921 MCP;  // for MCP4921 DAC output
static MidiPlayer MIDI;

// ---- MIDI debug defaults (affect 'midi play' commands) ----
// Requires MidiPlayer.h updated with Debug enum and progress fields.
static MidiPlayer::Debug g_midi_debug = MidiPlayer::Debug::Info;  // Off|Errors|Info|Verbose
static bool g_midi_progress = true;                               // periodic progress printouts
static uint16_t g_midi_prog_ms = 250;                             // progress interval (ms)
static int g_midi_track_index = -1;                               // -1 = auto-select by most NoteOn
static bool g_midi_allow_q = true;                                // allow 'q' to stop playback

// ========== Static buffer-related compile-time constant ==========
#define FS_SECTOR_SIZE 4096
// ========== Pins for Memory Chips ==========
const uint8_t PIN_FLASH_MISO = 4;               // GP4
const uint8_t PIN_PSRAM_MISO = PIN_FLASH_MISO;  // GP4
const uint8_t PIN_FLASH_MOSI = 3;               // GP3
const uint8_t PIN_PSRAM_MOSI = PIN_FLASH_MOSI;  // GP3
const uint8_t PIN_FLASH_SCK = 6;                // GP6
const uint8_t PIN_PSRAM_SCK = PIN_FLASH_SCK;    // GP6
const uint8_t PIN_FLASH_CS0 = 5;                // GP
const uint8_t PIN_PSRAM_CS0 = 14;               // GP14
const uint8_t PIN_PSRAM_CS1 = 15;               // GP15
const uint8_t PIN_PSRAM_CS2 = 26;               // GP26
const uint8_t PIN_PSRAM_CS3 = 27;               // GP27
const uint8_t PIN_NAND_CS = 8;                  // GP5
const uint8_t PIN_FLASH_CS1 = 29;               // GP29
// Audio/MCP pins
static const uint8_t PIN_DAC_CS = 7;   // MCP4921 CS on GP7
static const uint8_t PIN_BUZZER = 29;  // Buzzer/PWM on GP29
// ========== Flash/PSRAM instances (needed before bindActiveFs) ==========
// Persisted config you already have
const size_t PERSIST_LEN = 32;
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND,
  NAND,
};
static StorageBackend g_storage = StorageBackend::Flash;
UnifiedSpiMem::Manager uniMem(PIN_PSRAM_SCK, PIN_PSRAM_MOSI, PIN_PSRAM_MISO);  // Unified manager: one bus, many CS
// SimpleFS facades
W25QUnifiedSimpleFS fsFlash;
PSRAMUnifiedSimpleFS fsPSRAM;
MX35UnifiedSimpleFS fsNAND;
static bool g_dev_mode = true;
// ========== ActiveFS structure ==========
struct ActiveFS {
  bool (*mount)(bool) = nullptr;
  bool (*format)() = nullptr;
  bool (*wipeChip)() = nullptr;
  bool (*exists)(const char*) = nullptr;
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t) = nullptr;
  bool (*writeFile)(const char*, const uint8_t*, uint32_t, int /*mode*/) = nullptr;
  bool (*writeFileInPlace)(const char*, const uint8_t*, uint32_t, bool) = nullptr;
  uint32_t (*readFile)(const char*, uint8_t*, uint32_t) = nullptr;
  uint32_t (*readFileRange)(const char*, uint32_t, uint8_t*, uint32_t) = nullptr;
  bool (*getFileSize)(const char*, uint32_t&) = nullptr;
  bool (*getFileInfo)(const char*, uint32_t&, uint32_t&, uint32_t&) = nullptr;
  bool (*deleteFile)(const char*) = nullptr;
  void (*listFilesToSerial)() = nullptr;
  uint32_t (*nextDataAddr)() = nullptr;
  uint32_t (*capacity)() = nullptr;
  uint32_t (*dataRegionStart)() = nullptr;
  static constexpr uint32_t SECTOR_SIZE = FS_SECTOR_SIZE;
  static constexpr uint32_t PAGE_SIZE = 256;
  static constexpr size_t MAX_NAME = 32;
} activeFs;
struct FsIndexEntry {
  char name[ActiveFS::MAX_NAME + 1];
  uint32_t size;
  bool deleted;
  uint32_t seq;
};
struct FSIface {
  bool (*mount)(bool);
  bool (*exists)(const char*);
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t);
  bool (*writeFile)(const char*, const uint8_t*, uint32_t, int);
  bool (*writeFileInPlace)(const char*, const uint8_t*, uint32_t, bool);
  uint32_t (*readFile)(const char*, uint8_t*, uint32_t);
  bool (*getFileInfo)(const char*, uint32_t&, uint32_t&, uint32_t&);
};
// Helper to get device for specific backend (used by fscp)
static inline UnifiedSpiMem::MemDevice* deviceForBackend(StorageBackend backend) {
  switch (backend) {
    case StorageBackend::Flash: return fsFlash.raw().device();
    case StorageBackend::PSRAM_BACKEND: return fsPSRAM.raw().device();
    case StorageBackend::NAND: return fsNAND.raw().device();
    default: return nullptr;
  }
}
static inline UnifiedSpiMem::MemDevice* activeFsDevice() {
  switch (g_storage) {
    case StorageBackend::Flash: return fsFlash.raw().device();
    case StorageBackend::PSRAM_BACKEND: return fsPSRAM.raw().device();
    case StorageBackend::NAND: return fsNAND.raw().device();
    default: return nullptr;
  }
}
// Directory stride helper: 32 for NOR/PSRAM, NAND page size when on MX35
static inline uint32_t dirEntryStride() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 32u;
  return (dev->type() == UnifiedSpiMem::DeviceType::SpiNandMX35) ? dev->pageSize() : 32u;
}
// Erase alignment helper for reserve rounding (PSRAM => fallback to 4K)
static inline uint32_t getEraseAlign() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return ActiveFS::SECTOR_SIZE;
  uint32_t e = dev->eraseSize();
  return (e > 0) ? e : ActiveFS::SECTOR_SIZE;
}
// Erase alignment helper for a specific backend (for fscp)
static inline uint32_t getEraseAlignFor(StorageBackend b) {
  UnifiedSpiMem::MemDevice* dev = deviceForBackend(b);
  if (!dev) return ActiveFS::SECTOR_SIZE;
  uint32_t e = dev->eraseSize();
  return (e > 0) ? e : ActiveFS::SECTOR_SIZE;
}
static inline int fsReplaceMode() {
  switch (g_storage) {
    case StorageBackend::Flash:
      return (int)W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::NAND:
      return (int)MX35UnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::PSRAM_BACKEND:
      return 0;
  }
  return 0;
}
// Replace mode per backend (for fscp)
static inline int fsReplaceModeFor(StorageBackend b) {
  switch (b) {
    case StorageBackend::Flash:
      return (int)W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::NAND:
      return (int)MX35UnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::PSRAM_BACKEND:
      return 0;
  }
  return 0;
}
static void bindActiveFs(StorageBackend backend) {
  if (backend == StorageBackend::Flash) {
    activeFs.mount = [](bool b) {
      return fsFlash.mount(b);
    };
    activeFs.format = []() {
      return fsFlash.format();
    };
    activeFs.wipeChip = []() {
      return fsFlash.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsFlash.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsFlash.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsFlash.writeFile(n, d, s, static_cast<W25QUnifiedSimpleFS::WriteMode>(m));
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsFlash.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsFlash.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsFlash.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsFlash.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsFlash.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsFlash.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsFlash.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsFlash.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsFlash.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsFlash.dataRegionStart();
    };
  } else if (backend == StorageBackend::NAND) {
    activeFs.mount = [](bool b) {
      return fsNAND.mount(b);
    };
    activeFs.format = []() {
      return fsNAND.format();
    };
    activeFs.wipeChip = []() {
      return fsNAND.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsNAND.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsNAND.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsNAND.writeFile(n, d, s, static_cast<MX35UnifiedSimpleFS::WriteMode>(m));
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsNAND.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsNAND.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsNAND.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsNAND.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsNAND.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsNAND.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsNAND.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsNAND.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsNAND.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsNAND.dataRegionStart();
    };
  } else {
    activeFs.mount = [](bool b) {
      return fsPSRAM.mount(b);
    };
    activeFs.format = []() {
      return fsPSRAM.format();
    };
    activeFs.wipeChip = []() {
      return fsPSRAM.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsPSRAM.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsPSRAM.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsPSRAM.writeFile(n, d, s, m);
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsPSRAM.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsPSRAM.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsPSRAM.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsPSRAM.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsPSRAM.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsPSRAM.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsPSRAM.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsPSRAM.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsPSRAM.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsPSRAM.dataRegionStart();
    };
  }
}
static bool makePath(char* out, size_t outCap, const char* folder, const char* name) {
  if (!out || !folder || !name) return false;
  int needed = snprintf(out, outCap, "%s/%s", folder, name);
  if (needed < 0) return false;
  if ((size_t)needed > ActiveFS::MAX_NAME) return false;
  if ((size_t)needed >= outCap) return false;
  return true;
}
static void normalizePathInPlace(char* s, bool wantTrailingSlash) {
  if (!s) return;
  size_t r = 0;
  while (s[r] == '/') ++r;
  size_t w = 0;
  for (; s[r]; ++r) {
    char c = s[r];
    if (c == '/' && w > 0 && s[w - 1] == '/') continue;
    s[w++] = c;
  }
  s[w] = 0;
  size_t n = strlen(s);
  if (wantTrailingSlash) {
    if (n > 0 && s[n - 1] != '/') {
      if (n + 1 < ActiveFS::MAX_NAME + 1) {
        s[n] = '/';
        s[n + 1] = 0;
      }
    }
  } else {
    while (n > 0 && s[n - 1] == '/') { s[--n] = 0; }
  }
}
static bool makePathSafe(char* out, size_t outCap, const char* folder, const char* name) {
  if (!out || !folder || !name) return false;
  char tmp[ActiveFS::MAX_NAME + 1];
  if (folder[0] == 0) {
    if (snprintf(tmp, sizeof(tmp), "%s", name) < 0) return false;
  } else {
    if (snprintf(tmp, sizeof(tmp), "%s/%s", folder, name) < 0) return false;
  }
  normalizePathInPlace(tmp, /*wantTrailingSlash=*/false);
  size_t L = strlen(tmp);
  if (L > ActiveFS::MAX_NAME || L >= outCap) return false;
  memcpy(out, tmp, L + 1);
  return true;
}
static bool writeFileToFolder(const char* folder, const char* fname, const uint8_t* data, uint32_t len) {
  char path[ActiveFS::MAX_NAME + 1];
  if (!makePathSafe(path, sizeof(path), folder, fname)) {
    Console.println("path too long for SimpleFS (max 32 chars)");
    return false;
  }
  uint32_t eraseAlign = getEraseAlign();
  if (!activeFs.exists(path)) {
    uint32_t reserve = (len ? ((len + (eraseAlign - 1)) & ~(eraseAlign - 1)) : eraseAlign);
    if (reserve < eraseAlign) reserve = eraseAlign;
    return activeFs.createFileSlot(path, reserve, data, len);
  }
  if (activeFs.writeFileInPlace(path, data, len, true)) return true;
  return activeFs.writeFile(path, data, len, fsReplaceMode());
}
static uint32_t readFileFromFolder(const char* folder, const char* fname, uint8_t* buf, uint32_t bufSize) {
  char path[ActiveFS::MAX_NAME + 1];
  if (!makePathSafe(path, sizeof(path), folder, fname)) {
    Console.println("path too long for SimpleFS (max 32 chars)");
    return 0;
  }
  return activeFs.readFile(path, buf, bufSize);
}
static bool folderExists(const char* absFolder) {
  if (!absFolder || !absFolder[0]) return false;
  if (absFolder[strlen(absFolder) - 1] != '/') return false;
  return activeFs.exists(absFolder);
}
static bool mkdirFolder(const char* path) {
  if (activeFs.exists(path)) return true;
  return activeFs.writeFile(path, nullptr, 0, fsReplaceMode());
}
static bool touchPath(const char* cwd, const char* arg) {
  if (!arg) return false;
  size_t L = strlen(arg);
  if (L > 0 && arg[L - 1] == '/') {
    char marker[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(marker, sizeof(marker), cwd, arg, /*wantTrailingSlash=*/true)) {
      Console.println("touch: folder path too long (<= 32 chars)");
      return false;
    }
    if (folderExists(marker)) return true;
    return mkdirFolder(marker);
  }
  char path[ActiveFS::MAX_NAME + 1];
  if (!pathJoin(path, sizeof(path), cwd, arg, /*wantTrailingSlash=*/false)) {
    Console.println("touch: path too long (<= 32 chars)");
    return false;
  }
  if (activeFs.exists(path)) {
    return true;
  }
  return activeFs.writeFile(path, /*data*/ nullptr, /*size*/ 0, fsReplaceMode());
}
// ========== mv helpers ==========
static const char* lastSlash(const char* s) {
  const char* p = strrchr(s, '/');
  return p ? (p + 1) : s;
}
static bool buildAbsFile(char* out, size_t outCap, const char* cwd, const char* path) {
  if (!path || !out) return false;
  size_t L = strlen(path);
  if (L > 0 && path[L - 1] == '/') return false;
  return pathJoin(out, outCap, cwd, path, /*wantTrailingSlash*/ false);
}
static bool cmdMvImpl(const char* cwd, const char* srcArg, const char* dstArg) {
  if (!srcArg || !dstArg) return false;
  char srcAbs[ActiveFS::MAX_NAME + 1];
  if (!buildAbsFile(srcAbs, sizeof(srcAbs), cwd, srcArg)) {
    Console.println("mv: invalid source path");
    return false;
  }
  bool srcExists = activeFs.exists(srcAbs);
  if (!srcExists) {
    if (strstr(srcAbs, "//")) {
      char alt[sizeof(srcAbs)];
      strncpy(alt, srcAbs, sizeof(alt));
      alt[sizeof(alt) - 1] = 0;
      normalizePathInPlace(alt, /*wantTrailingSlash=*/false);
      if (activeFs.exists(alt)) {
        strncpy(srcAbs, alt, sizeof(srcAbs));
        srcAbs[sizeof(srcAbs) - 1] = 0;
        srcExists = true;
      }
    }
  }
  if (!srcExists) {
    Console.println("mv: source not found");
    return false;
  }
  uint32_t srcAddr = 0, srcSize = 0, srcCap = 0;
  if (!activeFs.getFileInfo(srcAbs, srcAddr, srcSize, srcCap)) {
    Console.println("mv: getFileInfo failed");
    return false;
  }
  char dstAbs[ActiveFS::MAX_NAME + 1];
  size_t Ldst = strlen(dstArg);
  bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
  if (dstIsFolder) {
    char folderNoSlash[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), cwd, dstArg, /*wantTrailingSlash*/ false)) {
      Console.println("mv: destination folder path too long");
      return false;
    }
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
      Console.println("mv: resulting path too long");
      return false;
    }
  } else {
    if (!buildAbsFile(dstAbs, sizeof(dstAbs), cwd, dstArg)) {
      Console.println("mv: invalid destination path");
      return false;
    }
  }
  normalizePathInPlace(dstAbs, /*wantTrailingSlash=*/false);
  if (strcmp(srcAbs, dstAbs) == 0) {
    Console.println("mv: source and destination are the same");
    return true;
  }
  if (pathTooLongForOnDisk(dstAbs)) {
    Console.println("mv: destination name too long for FS (would be truncated)");
    return false;
  }
  uint8_t* buf = (uint8_t*)malloc(srcSize ? srcSize : 1);
  if (!buf) {
    Console.println("mv: malloc failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcAbs, buf, srcSize);
  if (got != srcSize) {
    Console.println("mv: read failed");
    free(buf);
    return false;
  }
  uint32_t eraseAlign = getEraseAlign();
  uint32_t reserve = srcCap;
  if (reserve < eraseAlign) {
    uint32_t a = (srcSize + (eraseAlign - 1)) & ~(eraseAlign - 1);
    if (a > reserve) reserve = a;
  }
  if (reserve < eraseAlign) reserve = eraseAlign;
  bool ok = false;
  if (!activeFs.exists(dstAbs)) {
    ok = activeFs.createFileSlot(dstAbs, reserve, buf, srcSize);
  } else {
    uint32_t dA, dS, dC;
    if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
    if (dC >= srcSize) {
      ok = activeFs.writeFileInPlace(dstAbs, buf, srcSize, false);
    }
    if (!ok) {
      ok = activeFs.writeFile(dstAbs, buf, srcSize, fsReplaceMode());
    }
  }
  if (!ok) {
    Console.println("mv: write to destination failed");
    free(buf);
    return false;
  }
  if (!activeFs.deleteFile(srcAbs)) {
    Console.println("mv: warning: source delete failed");
  } else {
    Console.println("mv: ok");
  }
  free(buf);
  return true;
}
// ======== cp (copy) helper ========
static bool cmdCpImpl(const char* cwd, const char* srcArg, const char* dstArg, bool force) {
  if (!srcArg || !dstArg) return false;
  char srcAbs[ActiveFS::MAX_NAME + 1];
  if (!buildAbsFile(srcAbs, sizeof(srcAbs), cwd, srcArg)) {
    Console.println("cp: invalid source path");
    return false;
  }
  if (!activeFs.exists(srcAbs)) {
    Console.println("cp: source not found");
    return false;
  }
  uint32_t srcAddr = 0, srcSize = 0, srcCap = 0;
  if (!activeFs.getFileInfo(srcAbs, srcAddr, srcSize, srcCap)) {
    Console.println("cp: getFileInfo failed");
    return false;
  }
  char dstAbs[ActiveFS::MAX_NAME + 1];
  size_t Ldst = strlen(dstArg);
  bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
  if (dstIsFolder) {
    char folderNoSlash[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), cwd, dstArg, /*wantTrailingSlash*/ false)) {
      Console.println("cp: destination folder path too long");
      return false;
    }
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
      Console.println("cp: resulting path too long");
      return false;
    }
  } else {
    if (!buildAbsFile(dstAbs, sizeof(dstAbs), cwd, dstArg)) {
      Console.println("cp: invalid destination path");
      return false;
    }
  }
  normalizePathInPlace(dstAbs, /*wantTrailingSlash=*/false);
  if (pathTooLongForOnDisk(dstAbs)) {
    Console.println("cp: destination name too long for FS (would be truncated)");
    return false;
  }
  // If destination exists and not forcing, refuse
  if (activeFs.exists(dstAbs) && !force) {
    Console.println("cp: destination exists (use -f to overwrite)");
    return false;
  }
  // Read full source (consistent with mv behavior)
  uint8_t* buf = (uint8_t*)malloc(srcSize ? srcSize : 1);
  if (!buf) {
    Console.println("cp: malloc failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcAbs, buf, srcSize);
  if (got != srcSize) {
    Console.println("cp: read failed");
    free(buf);
    return false;
  }
  uint32_t eraseAlign = getEraseAlign();
  uint32_t reserve = srcCap;
  if (reserve < eraseAlign) {
    uint32_t a = (srcSize + (eraseAlign - 1)) & ~(eraseAlign - 1);
    if (a > reserve) reserve = a;
  }
  if (reserve < eraseAlign) reserve = eraseAlign;
  bool ok = false;
  if (!activeFs.exists(dstAbs)) {
    ok = activeFs.createFileSlot(dstAbs, reserve, buf, srcSize);
  } else {
    // overwrite existing
    uint32_t dA, dS, dC;
    if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
    if (dC >= srcSize) ok = activeFs.writeFileInPlace(dstAbs, buf, srcSize, false);
    if (!ok) ok = activeFs.writeFile(dstAbs, buf, srcSize, fsReplaceMode());
  }
  free(buf);
  if (!ok) {
    Console.println("cp: write failed");
    return false;
  }
  Console.println("cp: ok");
  return true;
}
static void printPct2(uint32_t num, uint32_t den) {
  if (den == 0) {
    Console.print("n/a");
    return;
  }
  uint32_t scaled = (uint32_t)(((uint64_t)num * 10000ULL + (den / 2)) / den);
  Console.printf("%lu.%02lu%%", (unsigned long)(scaled / 100), (unsigned long)(scaled % 100));
}
static uint32_t dirBytesUsedEstimate() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 0;
  constexpr uint32_t DIR_START = 0x000000;
  constexpr uint32_t DIR_SIZE = 64 * 1024;
  const uint32_t stride = dirEntryStride();
  uint8_t rec[32];
  uint32_t records = 0;
  for (uint32_t off = 0; off + sizeof(rec) <= DIR_SIZE; off += stride) {
    size_t got = dev->read(DIR_START + off, rec, sizeof(rec));
    if (got != sizeof(rec)) break;
    if (rec[0] == 0x57 && rec[1] == 0x46) {
      records++;
    }
  }
  return records * stride;
}
// ========== Minimal directory enumeration (reads the DIR table directly) ==========
static inline uint32_t rd32_be(const uint8_t* p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
}
static bool isAllFF(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i)
    if (p[i] != 0xFF) return false;
  return true;
}
static size_t buildFsIndex(FsIndexEntry* out, size_t outMax) {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 0;
  constexpr uint32_t DIR_START = 0x000000;
  constexpr uint32_t DIR_SIZE = 64 * 1024;
  const uint32_t stride = dirEntryStride();
  FsIndexEntry map[64];
  size_t mapCount = 0;
  uint8_t rec[32];
  for (uint32_t off = 0; off + sizeof(rec) <= DIR_SIZE; off += stride) {
    if (dev->read(DIR_START + off, rec, sizeof(rec)) != sizeof(rec)) break;
    if (rec[0] != 0x57 || rec[1] != 0x46) continue;
    uint8_t flags = rec[2];
    uint8_t nlen = rec[3];
    if (nlen == 0 || nlen > ActiveFS::MAX_NAME) continue;
    char name[ActiveFS::MAX_NAME + 1];
    memset(name, 0, sizeof(name));
    for (uint8_t i = 0; i < nlen; ++i) name[i] = (char)rec[4 + i];
    uint32_t size = rd32_be(&rec[24]);
    uint32_t seq = rd32_be(&rec[28]);
    bool deleted = (flags & 0x01) != 0;
    int idx = -1;
    for (size_t i = 0; i < mapCount; ++i) {
      if (strncmp(map[i].name, name, ActiveFS::MAX_NAME) == 0) {
        idx = (int)i;
        break;
      }
    }
    if (idx < 0) {
      if (mapCount >= 64) continue;
      idx = (int)mapCount++;
      strncpy(map[idx].name, name, ActiveFS::MAX_NAME);
      map[idx].name[ActiveFS::MAX_NAME] = 0;
      map[idx].seq = 0;
    }
    if (seq >= map[idx].seq) {
      map[idx].seq = seq;
      map[idx].deleted = deleted;
      map[idx].size = size;
    }
  }
  if (out && outMax) {
    size_t n = (mapCount < outMax) ? mapCount : outMax;
    for (size_t i = 0; i < n; ++i) out[i] = map[i];
  }
  return mapCount;
}
static bool hasPrefix(const char* name, const char* prefix) {
  size_t lp = strlen(prefix);
  return strncmp(name, prefix, lp) == 0;
}
static bool childOfFolder(const char* name, const char* folderPrefix, const char*& childOut) {
  size_t lp = strlen(folderPrefix);
  if (strncmp(name, folderPrefix, lp) != 0) return false;
  const char* rest = name + lp;
  if (*rest == 0) return false;
  const char* slash = strchr(rest, '/');
  if (slash) return false;
  childOut = rest;
  return true;
}
static void dumpDirHeadRaw(uint32_t bytes = 256) {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) {
    Console.println("lsdebug: no device");
    return;
  }
  if (bytes == 0 || bytes > 1024) bytes = 256;
  uint8_t buf[1024];
  size_t got = dev->read(0, buf, bytes);
  Console.printf("lsdebug: read %u bytes @ 0x000000\n", (unsigned)got);
  for (size_t i = 0; i < got; i += 16) {
    Console.printf("  %04u: ", (unsigned)i);
    for (size_t j = 0; j < 16 && (i + j) < got; ++j) {
      uint8_t b = buf[i + j];
      if (b < 16) Console.print('0');
      Console.print(b, HEX);
      Console.print(' ');
    }
    Console.println();
  }
}
static void cmdDf() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (dev) {
    const auto t = dev->type();
    const char* style = UnifiedSpiMem::deviceTypeName(t);
    const uint8_t cs = dev->cs();
    const uint64_t devCap = dev->capacity();
    const uint32_t dataStart = activeFs.dataRegionStart();
    const uint32_t fsCap32 = activeFs.capacity();
    const uint32_t dataCap = (fsCap32 > dataStart) ? (fsCap32 - dataStart) : 0;
    const uint32_t dataUsed = (activeFs.nextDataAddr() > dataStart) ? (activeFs.nextDataAddr() - dataStart) : 0;
    const uint32_t dataFree = (dataCap > dataUsed) ? (dataCap - dataUsed) : 0;
    const uint32_t dirUsed = dirBytesUsedEstimate();
    const uint32_t dirFree = (64u * 1024u > dirUsed) ? (64u * 1024u - dirUsed) : 0;
    Console.println("Filesystem (active):");
    Console.printf("  Device:  %s  CS=%u\n", style, (unsigned)cs);
    Console.printf("  DevCap:  %llu bytes\n", (unsigned long long)devCap);
    Console.printf("  FS data: %lu used (", (unsigned long)dataUsed);
    printPct2(dataUsed, dataCap);
    Console.printf(")  %lu free (", (unsigned long)dataFree);
    printPct2(dataFree, dataCap);
    Console.println(")");
    Console.printf("  DIR:     %lu used (", (unsigned long)dirUsed);
    printPct2(dirUsed, 64u * 1024u);
    Console.printf(")  %lu free\n", (unsigned long)dirFree);
  } else {
    Console.println("Filesystem (active): none");
  }
  size_t n = uniMem.detectedCount();
  if (n == 0) return;
  Console.println("Detected devices:");
  for (size_t i = 0; i < n; ++i) {
    const auto* di = uniMem.detectedInfo(i);
    if (!di) continue;
    bool isActive = false;
    UnifiedSpiMem::MemDevice* dev = activeFsDevice();
    if (dev) {
      isActive = (di->cs == dev->cs() && di->type == dev->type());
    }
    Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes%s\n", di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName, (unsigned long long)di->capacityBytes, isActive ? "\t <-- [mounted]" : "");
  }
}
// ========== CWD + path helpers ==========
static char g_cwd[ActiveFS::MAX_NAME + 1] = "";  // "" = root, printed as "/"
static void pathStripTrailingSlashes(char* p) {
  if (!p) return;
  size_t n = strlen(p);
  while (n > 0 && p[n - 1] == '/') { p[--n] = 0; }
}
static bool pathJoin(char* out, size_t outCap, const char* base, const char* name, bool wantTrailingSlash) {
  if (!out || !name) return false;
  if (name[0] == '/') {
    const char* s = name + 1;
    size_t L = strlen(s);
    if (L > ActiveFS::MAX_NAME || L >= outCap) return false;
    memcpy(out, s, L + 1);
    if (wantTrailingSlash && L > 0 && out[L - 1] != '/') {
      if (L + 1 > ActiveFS::MAX_NAME || L + 2 > outCap) return false;
      out[L] = '/';
      out[L + 1] = 0;
    }
    return true;
  }
  if (!base) base = "";
  char tmp[ActiveFS::MAX_NAME + 1];
  int need = 0;
  if (base[0] == 0) need = snprintf(tmp, sizeof(tmp), "%s", name);
  else if (name[0] == 0) need = snprintf(tmp, sizeof(tmp), "%s", base);
  else need = snprintf(tmp, sizeof(tmp), "%s/%s", base, name);
  if (need < 0 || (size_t)need > ActiveFS::MAX_NAME || (size_t)need >= sizeof(tmp)) return false;
  size_t L = strlen(tmp);
  if (wantTrailingSlash && L > 0 && tmp[L - 1] != '/') {
    if (L + 1 > ActiveFS::MAX_NAME) return false;
    tmp[L] = '/';
    tmp[L + 1] = 0;
  }
  if (L >= outCap) return false;
  memcpy(out, tmp, L + 1);
  return true;
}
static void pathParent(char* p) {
  if (!p) return;
  pathStripTrailingSlashes(p);
  size_t n = strlen(p);
  if (n == 0) return;
  char* last = strrchr(p, '/');
  if (!last) {
    p[0] = 0;
  } else if (last == p) {
    p[0] = 0;
  } else {
    *last = 0;
  }
}
static bool isFolderMarkerName(const char* nm) {
  size_t n = strlen(nm);
  return (n > 0 && nm[n - 1] == '/');
}
static const char* firstSlash(const char* s) {
  return strchr(s, '/');
}
static constexpr size_t FS_NAME_ONDISK_MAX = 32;  // match SimpleFS MAX_NAME
static bool pathTooLongForOnDisk(const char* full) {
  return strlen(full) > FS_NAME_ONDISK_MAX;
}

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
  { FILE_RET42, blob_ret42, blob_ret42_len },
  { FILE_ADD2, blob_add2, blob_add2_len },
  { FILE_PWMC, blob_pwmc, blob_pwmc_len },
  { FILE_PWMC2350, blob_pwmc2350, blob_pwmc2350_len },
  { FILE_RETMIN, blob_retmin, blob_retmin_len },
  { FILE_DONT, blob_dont, blob_dont_len },
  { FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len },
  { FILE_ONSCRIPT, blob_onscript, blob_onscript_len },
  { FILE_SONG, blob_song, blob_song_len },
  { FILE_PT1, blob_pt1, blob_pt1_len },
  { FILE_PT2, blob_pt2, blob_pt2_len },
  { FILE_PT3, blob_pt3, blob_pt3_len },
};
static const size_t g_blobs_count = sizeof(g_blobs) / sizeof(g_blobs[0]);
// ========== Return mailbox reservation (Scratch) ==========
extern "C" __scratch_x("blob_mailbox") __attribute__((aligned(4)))
int8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };
// ================= ExecHost header =================
#include "ExecHost.h"
static ExecHost Exec;
// ================= FSHelpers header =================
static void updateExecFsTable() {
  ExecFSTable t{};
  t.exists = activeFs.exists;
  t.getFileSize = activeFs.getFileSize;
  t.readFile = activeFs.readFile;
  t.readFileRange = activeFs.readFileRange;
  t.createFileSlot = activeFs.createFileSlot;
  t.writeFile = activeFs.writeFile;
  t.writeFileInPlace = activeFs.writeFileInPlace;
  t.getFileInfo = activeFs.getFileInfo;
  //t.deleteFile = activeFs.deleteFile;
  Exec.attachFS(t);
}
#include "MemDiag.h"
#include "BlobGen.h"
#include "PSRAMDiag.h"
#include "CoProcRPCHelpers.h"
#include "NanoishTextEditor.h"
#include "SHA256hashcmd.h"
#include "Base64Utils.h"
#include "InputHelper.h"
#include "CrossFSUtils.h"
#include "CompilerHelpers.h"
#include "ZModem.h"
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
  Console.printf("  exec <file> [a0..aN] [&]     - execute blob with 0..%d int args on core1; '&' to background\n", (int)MAX_EXEC_ARGS);
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
  Console.println("Co-Processor (serial RPC) commands:");
  Console.println("  coproc ping|info|exec|sexec|func|status|mbox|cancel|reset|isp enter|exit");
  Console.println();
  Console.println("Linux hints: base64 -w0 your.bin  |  xxd -p -c 999999 your.bin | tr -d '\\n'");
}
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
  } else if (!strcmp(t0, "history")) {
    int start = (g_history_count > HISTORY_MAX) ? (g_history_count - HISTORY_MAX) : 0;
    for (int i = start; i < g_history_count; ++i) {
      Console.printf("  %2d  %s\n", i - start + 1, g_history[i]);
    }
  } else if (!strcmp(t0, "termwidth")) {
    char* cstr;
    if (!nextToken(p, cstr)) {
      Console.print("termwidth: ");
      Console.print((unsigned)g_term_cols);
      Console.println(" cols");
    } else {
      size_t cols = (size_t)strtoul(cstr, nullptr, 0);
      if (cols < 20) cols = 20;
      if (cols > 240) cols = 240;
      g_term_cols = cols;
      Console.print("termwidth set to ");
      Console.print((unsigned)g_term_cols);
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
      updateExecFsTable();
      bool ok = activeFs.mount(true);
      Console.println("Switched active storage to FLASH");
      Console.println(ok ? "Mounted FLASH (auto-format if empty)" : "Mount failed (FLASH)");
    } else if (!strcmp(tok, "psram")) {
      g_storage = StorageBackend::PSRAM_BACKEND;
      bindActiveFs(g_storage);
      updateExecFsTable();
      bool ok = activeFs.mount(false);
      Console.println("Switched active storage to PSRAM");
      Console.println(ok ? "Mounted PSRAM (no auto-format)" : "Mount failed (PSRAM)");
    } else if (!strcmp(tok, "nand")) {
      g_storage = StorageBackend::NAND;
      bindActiveFs(g_storage);
      updateExecFsTable();
      bool ok = activeFs.mount(true);
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
  } else if (!strcmp(t0, "coproc")) {
    char* sub;
    if (!nextToken(p, sub)) {
      Console.println("coproc cmds:");
      Console.println("  coproc ping|info|exec <file> [a0..]|sexec <file> [a0..]|func <name> [a0..]|status|mbox [n]|cancel|reset|isp enter|exit");
      return;
    }
    if (!strcmp(sub, "ping")) {
      if (!Exec.coprocHello()) Console.println("coproc ping failed");
    } else if (!strcmp(sub, "info")) {
      if (!Exec.coprocInfo()) Console.println("coproc info failed");
    } else if (!strcmp(sub, "sexec")) {
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc sexec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) tokens[argc++] = tok;
      if (!Exec.coprocScriptLoadFile(fname)) {
        Console.println("coproc sexec failed: load failed");
        return;
      }
      if (!Exec.coprocScriptExecTokens(tokens, argc)) Console.println("coproc sexec failed");
    } else if (!strcmp(sub, "exec")) {
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc exec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) tokens[argc++] = tok;
      if (!Exec.coprocLoadFile(fname)) {
        Console.println("coproc exec failed: load failed");
        return;
      }
      if (!Exec.coprocExecTokens(tokens, argc)) Console.println("coproc exec failed");
    } else if (!strcmp(sub, "func")) {
      char* name = nullptr;
      if (!nextToken(p, name)) {
        Console.println("usage: coproc func <name> [a0..aN]");
        return;
      }
      static int32_t argvN[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) {
        argvN[argc++] = (int32_t)strtol(tok, nullptr, 0);
      }
      int32_t result = 0;
      if (coprocCallFunc(name, argvN, argc, result)) {
        Console.print("coproc func OK, result=");
        Console.println(result);
      } else {
        Console.println("coproc func failed");
      }
    } else if (!strcmp(sub, "status")) {
      if (!Exec.coprocStatus()) Console.println("coproc status failed");
    } else if (!strcmp(sub, "mbox")) {
      char* nstr;
      uint32_t n = 128;
      if (nextToken(p, nstr)) n = (uint32_t)strtoul(nstr, nullptr, 0);
      if (!Exec.coprocMailboxRead(n)) Console.println("coproc mbox failed");
    } else if (!strcmp(sub, "cancel")) {
      if (!Exec.coprocCancel()) Console.println("coproc cancel failed");
    } else if (!strcmp(sub, "reset")) {
      if (!Exec.coprocReset()) Console.println("coproc reset failed (transport)");
    } else if (!strcmp(sub, "isp")) {
      char* arg = nullptr;
      if (!nextToken(p, arg)) {
        Console.println("usage: coproc isp enter|exit");
        return;
      }
      if (!strcmp(arg, "enter")) {
        if (!Exec.coprocIspEnter()) Console.println("coproc isp enter failed");
      } else if (!strcmp(arg, "exit")) {
        if (!Exec.coprocIspExit()) Console.println("coproc isp exit failed");
      } else {
        Console.println("usage: coproc isp enter|exit");
      }
    } else {
      Console.println("usage: coproc ping|info|exec|sexec|func|status|mbox|cancel|reset|isp");
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
  } else if (!strcmp(t0, "timeout")) {
    char* msStr;
    if (!nextToken(p, msStr)) {
      Console.print("timeout override = ");
      Console.print((uint32_t)Exec.getTimeoutOverride());
      Console.println(" ms (0 = use per-call defaults)");
    } else {
      uint32_t ms = (uint32_t)strtoul(msStr, nullptr, 0);
      Exec.setTimeoutOverride(ms);
      Console.print("timeout override set to ");
      Console.print(ms);
      Console.println(" ms");
    }
  } else if (!strcmp(t0, "meminfo")) {
    Console.println();
    Console.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Console.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Console.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Console.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Console.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Console.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
#ifdef ARDUINO_ARCH_RP2040
    printStackInfo();
#else
    Console.printf("Free Stack (core0 approx): %u bytes\n", freeStackCurrentCoreApprox());
#endif
    psramPrintCapacityReport(uniMem);
  } else if (!strcmp(t0, "psramsmoketest")) {
    psramPrintCapacityReport(uniMem);
    psramSafeSmokeTest(fsPSRAM);
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
    if (g_b64u.active) {
      Console.println("putb64s: another upload is active");
      return;
    }
    b64uStart(fn, expected);
    return;  // loop() will pump until upload completes
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
  } else {
    Console.println("Unknown command. Type 'help'.");
  }
}
// ========== Setup and main loops ==========
void setup() {
  delay(500);
  Serial.begin(115200);
  while (!Serial) { delay(20); }
  delay(20);
  Console.println("System booting..");
  Console.begin();
  // Arbiter + CBTLV3257 wiring help (this MCU = Owner A)
  /*Console.println("External SPI arbiter wiring (this MCU = Owner A):");
  Console.println("  REQ_A:   RP2040 GP4  -> ATtiny861A PA0 (active-low request)");
  Console.println("  GRANT_A: RP2040 GP5  <- ATtiny861A PA4 (OWNER_A, high = granted)");
  Console.println("CBTLV3257 control (driven by ATtiny861A):");
  Console.println("  S (select, pin 1):  <- PB3  (selects between B1 and B2; B side selected when HIGH)");
  Console.println("  OE# (enable, pin 15): <- PB4  (active LOW; bus connected when LOW)");
  Console.println("  Vcc: pin 16, GND: pin 8");
  Console.println("CBTLV3257 channel map:");
  Console.println("  CH0 (SCK):   Master A -> pin 2  (1B1), \tMaster B -> pin 3  (1B2), \tY (shared) -> pin 4  (1A)");
  Console.println("  CH1 (MOSI):  Master A -> pin 5  (2B1), \tMaster B -> pin 6  (2B2), \tY (shared) -> pin 7  (2A)");
  Console.println("  CH2 (MISO):  Master A -> pin 11 (3B1), \tMaster B -> pin 10 (3B2), \tY (shared) -> pin 9  (3A)");
  Console.println("  CH3 (opt):   Master A -> pin 14 (4B1), \tMaster B -> pin 13 (4B2), \tY (shared) -> pin 12 (4A)");
  Console.println("Notes:");
  Console.println("  - S = pin 1, OE# = pin 15. Device CS lines may remain direct from each MCU.");
  Console.println("  - Master A = this MCU");
  Console.println("  - Master B = co-processor MCU");
  Console.println("  - Y = shared to memories");
  Console.println();*/
  // Enable arbiter guard (Host A)
  //UnifiedSpiMem::ExternalArbiter::begin(/*REQ*/ 4, /*GRANT*/ 5, /*reqActiveLow*/ true, /*grantActiveHigh*/ true, /*defaultAcquireMs*/ 1000, /*shortAcquireMs*/ 300);
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
  bindActiveFs(g_storage);
  bool mounted = activeFs.mount(g_storage == StorageBackend::Flash /*autoFormatIfEmpty*/);
  if (!mounted) {
    Console.println("FS mount failed on active storage");
  }
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;
  Exec.attachConsole(&Console);
  Exec.attachCoProc(&coprocLink, COPROC_BAUD);
  updateExecFsTable();
  Console.printf("Controller serial link ready @ %u bps (RX=GP%u, TX=GP%u)\n", (unsigned)COPROC_BAUD, (unsigned)PIN_COPROC_RX, (unsigned)PIN_COPROC_TX);
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
      // Exec.pollBackground();
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
      // Exec.pollBackground();
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
  Console.printf("System ready. Type 'help'\n> ");
}
void setup1() {
  Exec.core1Setup();
}
void loop1() {
  Exec.core1Poll();
  tight_loop_contents();
}
void loop() {
  Exec.pollBackground();
  if (g_b64u.active) {
    b64uPump();
    return;
  }
  if (readLine()) {
    handleCommand(lineBuf);
    Console.print("> ");
  }
}