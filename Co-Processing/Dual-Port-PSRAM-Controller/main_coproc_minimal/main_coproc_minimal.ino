/*
  main_coproc_softserial.ino
  RP2350 / RP2040 Co-Processor (software-serial "bitbanged UART") - transport + blob executor + script interpreter
*/
#include <Arduino.h>
// ------- Shared HW SPI (FLASH + PSRAM) -------
#include "ConsolePrint.h"
// Force low identification speed compatible with the bridge
#define UNIFIED_SPI_INSTANCE SPI1
#define UNIFIED_SPI_CLOCK_HZ 104000000UL  // 104 MHz SPI
#define W25Q_SPI_CLOCK_HZ 104000000UL     // 104 MHz NOR
#define MX35_SPI_CLOCK_HZ 104000000UL     // 104 MHz NAND
#include "UnifiedSPIMemSimpleFS.h"
#include "BusArbiterWithISP.h"
#include "rp_selfupdate.h"
#include <SoftwareSerial.h>
#include "CoProcProto.h"
#include "CoProcLang.h"
// ------- Debug toggle -------
#ifndef COPROC_DEBUG
#define COPROC_DEBUG 1
#endif
#if COPROC_DEBUG
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...) \
  do { \
  } while (0)
#endif
// ------- Pins (as requested) -------
static const uint8_t PIN_RX = 0;  // GP0  (co-processor RX)
static const uint8_t PIN_TX = 1;  // GP1  (co-processor TX)
// ------- Software serial config -------
#ifndef SOFT_BAUD
//#define SOFT_BAUD 230400
#define SOFT_BAUD 115200
#endif
static SoftwareSerial coproclink(PIN_RX, PIN_TX, false);  // RX, TX, non-inverted
// Store ASCII scripts as raw multi-line strings, expose pointer + length.
#ifndef DECLARE_ASCII_SCRIPT
#define DECLARE_ASCII_SCRIPT(name, literal) \
  static const char name##_ascii[] = literal; \
  const uint8_t* name = reinterpret_cast<const uint8_t*>(name##_ascii); \
  const unsigned int name##_len = (unsigned int)(sizeof(name##_ascii) - 1)
#endif
struct BlobReg;  // Forward declaration
// ------- Console wrapper -------
static ConsolePrint Console;
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

// ------- Mailbox / cancel (shared with blob + script) -------
struct BlobReg {
  // ========== Blob registry ==========
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
#ifndef BLOB_MAILBOX_MAX
#define BLOB_MAILBOX_MAX 256
#endif
extern "C" __attribute__((aligned(4))) uint8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };
volatile uint8_t g_cancel_flag = 0;
// ------- Flash/PSRAM instances -------
const uint8_t PIN_PSRAM_MISO = 12;            // GP12
const uint8_t PIN_PSRAM_MOSI = 15;            // GP15
const uint8_t PIN_PSRAM_SCK = 14;             // GP14
const uint8_t PIN_NOR_MISO = PIN_PSRAM_MISO;  // PIN_PSRAM_MISO
const uint8_t PIN_NOR_MOSI = PIN_PSRAM_MOSI;  // PIN_PSRAM_MOSI
const uint8_t PIN_NOR_SCK = PIN_PSRAM_SCK;    // PIN_PSRAM_SCK
const uint8_t PIN_PSRAM_CS0 = 13;             // GP13
const uint8_t PIN_NOR_CS0 = 13;               // GP
//const uint8_t PIN_NAND_CS0 = 13;              // GP
// Persisted config you already have
const size_t PERSIST_LEN = 32;
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND,
  NAND,
};
static StorageBackend g_storage = StorageBackend::PSRAM_BACKEND;
UnifiedSpiMem::Manager uniMem(PIN_PSRAM_SCK, PIN_PSRAM_MOSI, PIN_PSRAM_MISO);
// SimpleFS facades
W25QUnifiedSimpleFS fsFlash;
PSRAMUnifiedSimpleFS fsPSRAM;
MX35UnifiedSimpleFS fsNAND;

// ======== SimpleFS active facade (PSRAM only on co-processor) ========
#define FS_SECTOR_SIZE 4096
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

static void bindActiveFsPSRAM() {
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

#include "MemDiag.h"
#include "BlobGen.h"
#include "PSRAMDiag.h"
//#include "CoProcRPCHelpers.h"
//#include "NanoishTextEditor.h"
#include "SHA256hashcmd.h"
#include "Base64Utils.h"
#include "InputHelper.h"
#include "CrossFSUtils.h"
#include "CompilerHelpers.h"

// ========= Minimal interactive serial console (USB Serial) =========
static char g_lineBuf[256];
static size_t g_lineLen = 0;
static bool g_prompted = false;
static inline void consolePromptOnce() {
  if (!g_prompted) {
    Console.print("> ");
    g_prompted = true;
  }
}
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
static inline void compiler_barrier() {
  __asm__ __volatile__("" ::
                         : "memory");
}
// ========= End console scaffolding =========

// ===== Directory index helpers (before buildFsIndex and console usage) =====
static inline uint32_t rd32_be(const uint8_t* p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
}
static inline uint32_t dirEntryStride() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 32u;
  return (dev->type() == UnifiedSpiMem::DeviceType::SpiNandMX35) ? dev->pageSize() : 32u;
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
    if (rec[0] == 0x57 && rec[1] == 0x46) records++;
  }
  return records * stride;
}
static bool pathTooLongForOnDisk(const char* full) {
  return strlen(full) > ActiveFS::MAX_NAME;
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
static void pathParent(char* p) {
  if (!p) return;
  size_t n = strlen(p);
  while (n > 0 && p[n - 1] == '/') { p[--n] = 0; }
  n = strlen(p);
  if (n == 0) return;
  char* last = strrchr(p, '/');
  if (!last) p[0] = 0;
  else if (last == p) p[0] = 0;
  else *last = 0;
}
static const char* lastSlash(const char* s) {
  const char* p = strrchr(s, '/');
  return p ? (p + 1) : s;
}

// ------- ISP mode state -------
static volatile bool g_isp_active = false;
static inline void serviceISPIfActive() {
  if (g_isp_active) {
    ArbiterISP::serviceISPOnce();
  }
}
// ------- Executor and script handling (moved to header-only class) -------
#include "CoProcExec.h"
static CoProcExec g_exec;
// ------- Transport / protocol buffers -------
static CoProc::Frame g_reqHdr, g_respHdr;
static uint8_t* g_reqBuf = nullptr;
static uint8_t* g_respBuf = nullptr;
static const uint32_t RESP_MAX = 8192;
static const uint32_t REQ_MAX = 8192;

// ========= Console command handler =========
static char g_cwd[ActiveFS::MAX_NAME + 1] = "";  // "" = root
static void cmdDf() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (dev) {
    const auto t = dev->type();
    const char* style = UnifiedSpiMem::deviceTypeName(t);
    const uint8_t cs = dev->cs();
    const uint64_t devCap = dev->capacity();
    const uint32_t dataStart = activeFs.dataRegionStart ? activeFs.dataRegionStart() : 0;
    const uint32_t fsCap32 = activeFs.capacity ? activeFs.capacity() : 0;
    const uint32_t dataCap = (fsCap32 > dataStart) ? (fsCap32 - dataStart) : 0;
    const uint32_t dataUsed = (activeFs.nextDataAddr && activeFs.nextDataAddr() > dataStart) ? (activeFs.nextDataAddr() - dataStart) : 0;
    const uint32_t dataFree = (dataCap > dataUsed) ? (dataCap - dataUsed) : 0;
    const uint32_t dirUsed = dirBytesUsedEstimate();
    const uint32_t dirFree = (64u * 1024u > dirUsed) ? (64u * 1024u - dirUsed) : 0;
    Console.println("Filesystem (PSRAM):");
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
    Console.println("Filesystem: none");
  }
  size_t n = uniMem.detectedCount();
  if (n == 0) return;
  Console.println("Detected devices:");
  for (size_t i = 0; i < n; ++i) {
    const auto* di = uniMem.detectedInfo(i);
    if (!di) continue;
    bool isActive = false;
    UnifiedSpiMem::MemDevice* devA = activeFsDevice();
    if (devA) isActive = (di->cs == devA->cs() && di->type == devA->type());
    Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes%s\n",
                   di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName,
                   (unsigned long long)di->capacityBytes, isActive ? "\t <-- [mounted]" : "");
  }
}
static bool buildAbsFile(char* out, size_t outCap, const char* cwd, const char* path) {
  if (!path || !out) return false;
  size_t L = strlen(path);
  if (L > 0 && path[L - 1] == '/') return false;
  return pathJoin(out, outCap, cwd, path, /*wantTrailingSlash*/ false);
}
static bool writeFileToFolder(const char* folder, const char* fname, const uint8_t* data, uint32_t len) {
  char path[ActiveFS::MAX_NAME + 1];
  if (!makePathSafe(path, sizeof(path), folder, fname)) {
    Console.println("path too long for SimpleFS (max 32 chars)");
    return false;
  }
  return writeBinaryToFS(path, data, len);
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
  normalizePathInPlace(path, /*wantTrailingSlash=*/false);
  if (activeFs.exists(path)) return true;
  return activeFs.writeFile(path, /*data*/ nullptr, /*size*/ 0, /*mode*/ 0);
}
static void printHelp() {
  Console.println("Co-Processor Console Commands (filename max 32 chars):");
  Console.println("  help                         - this help");
  Console.println("  files                        - list files in PSRAM FS");
  Console.println("  info <file>                  - show file addr/size/cap");
  Console.println("  dump <file> <nbytes>         - hex dump head of file");
  Console.println("  mkSlot <file> <reserve>      - create sector-aligned slot");
  Console.println("  cat <file> [n]               - print file contents (truncates if very large)");
  Console.println("  del <file>                   - delete a file");
  Console.println("  rm <file>                    - alias for 'del'");
  Console.println("  cp <src> <dst|folder/>       - copy file (within PSRAM FS)");
  Console.println("  mv <src> <dst|folder/>       - move/rename file (within PSRAM FS)");
  Console.println("  mkdir <path>                 - create folder marker");
  Console.println("  touch <path|name|folder/>    - create empty file or folder marker");
  Console.println("  pwd                          - show current folder (\"/\" = root)");
  Console.println("  cd <path|/|..|.>             - change folder");
  Console.println("  ls [path]                    - list current or specified folder");
  Console.println("  rmdir <path> [-r]            - remove folder; -r deletes all children");
  Console.println("  df                           - show device and FS usage");
  Console.println("  rescanfs                     - rescan SPI devices");
  Console.println("  format                       - format PSRAM FS");
  Console.println("  wipe                         - erase PSRAM chip (DANGEROUS)");
  Console.println("  puthex <file> <hex>          - upload binary as hex string");
  Console.println("  meminfo                      - show heap info");
  Console.println("  isp enter|exit               - enter/exit ISP mode on arbiter");
  Console.println("  dacperf                      - run DAC performance test");
  Console.println("  reset                        - reboot the MCU");
}
static void handleCommand(char* line) {
  char* p = line;
  char* t0;
  if (!nextToken(p, t0)) return;

  if (!strcmp(t0, "help")) {
    printHelp();
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
    uint32_t n = (uint32_t)strtoul(nstr, nullptr, 0);
    dumpFileHead(fn, n);
  } else if (!strcmp(t0, "mkSlot")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: mkSlot <file> <reserve>");
      return;
    }
    uint32_t res = (uint32_t)strtoul(nstr, nullptr, 0);
    if (activeFs.createFileSlot(fn, res, nullptr, 0)) Console.println("slot created");
    else Console.println("mkSlot failed");
  } else if (!strcmp(t0, "cat")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn)) {
      Console.println("usage: cat <file> [n]");
      return;
    }
    uint32_t limit = 0xFFFFFFFFu;
    if (nextToken(p, nstr)) limit = (uint32_t)strtoul(nstr, nullptr, 0);
    uint32_t sz = 0;
    if (!activeFs.getFileSize(fn, sz)) {
      Console.println("cat: not found");
      return;
    }
    if (limit == 0xFFFFFFFFu) {
      if (sz > 4096) {
        limit = 4096;
        Console.println("(cat truncated to 4096 bytes)");
      } else limit = sz;
    } else if (limit > sz) limit = sz;
    const size_t CHUNK = 128;
    uint8_t buf[CHUNK];
    uint32_t off = 0;
    while (off < limit) {
      size_t n = (limit - off > CHUNK) ? CHUNK : (limit - off);
      uint32_t got = activeFs.readFileRange(fn, off, buf, (uint32_t)n);
      if (got != n) {
        Console.println("\ncat: read error");
        break;
      }
      for (size_t i = 0; i < n; ++i) Serial.write(buf[i]);
      off += n;
      yield();
    }
    Serial.write('\n');
  } else if (!strcmp(t0, "del") || !strcmp(t0, "rm")) {
    char* arg;
    if (!nextToken(p, arg)) {
      Console.println("usage: del|rm <file>");
      return;
    }
    char abs[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(abs, sizeof(abs), g_cwd, arg, false)) {
      Console.println("del: path too long");
      return;
    }
    normalizePathInPlace(abs, false);
    bool ok = false;
    if (activeFs.exists && activeFs.exists(abs)) ok = activeFs.deleteFile(abs);
    Console.println(ok ? "deleted" : "delete failed");
  } else if (!strcmp(t0, "cp")) {
    char* srcArg;
    char* dstArg;
    if (!nextToken(p, srcArg) || !nextToken(p, dstArg)) {
      Console.println("usage: cp <src> <dst|folder/>");
      return;
    }
    char srcAbs[ActiveFS::MAX_NAME + 1];
    if (!buildAbsFile(srcAbs, sizeof(srcAbs), g_cwd, srcArg)) {
      Console.println("cp: bad source");
      return;
    }
    if (!activeFs.exists(srcAbs)) {
      Console.println("cp: source not found");
      return;
    }
    uint32_t a, s, cap;
    if (!activeFs.getFileInfo(srcAbs, a, s, cap)) {
      Console.println("cp: getFileInfo failed");
      return;
    }
    char dstAbs[ActiveFS::MAX_NAME + 1];
    size_t Ldst = strlen(dstArg);
    bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
    if (dstIsFolder) {
      char folderNoSlash[ActiveFS::MAX_NAME + 1];
      if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), g_cwd, dstArg, false)) {
        Console.println("cp: dest folder path too long");
        return;
      }
      const char* base = lastSlash(srcAbs);
      if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
        Console.println("cp: resulting path too long");
        return;
      }
    } else {
      if (!buildAbsFile(dstAbs, sizeof(dstAbs), g_cwd, dstArg)) {
        Console.println("cp: invalid dest");
        return;
      }
    }
    normalizePathInPlace(dstAbs, false);
    if (pathTooLongForOnDisk(dstAbs)) {
      Console.println("cp: destination name too long (FS=32 chars)");
      return;
    }
    uint8_t* buf = (uint8_t*)malloc(s ? s : 1);
    if (!buf) {
      Console.println("cp: malloc failed");
      return;
    }
    uint32_t got = activeFs.readFile(srcAbs, buf, s);
    if (got != s) {
      Console.println("cp: read failed");
      free(buf);
      return;
    }
    uint32_t eraseAlign = getEraseAlign();
    uint32_t reserve = cap;
    if (reserve < eraseAlign) {
      uint32_t a2 = (s + (eraseAlign - 1)) & ~(eraseAlign - 1);
      if (a2 > reserve) reserve = a2;
    }
    if (reserve < eraseAlign) reserve = eraseAlign;
    bool ok = false;
    if (!activeFs.exists(dstAbs)) ok = activeFs.createFileSlot(dstAbs, reserve, buf, s);
    else {
      uint32_t dA, dS, dC;
      if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
      if (dC >= s) ok = activeFs.writeFileInPlace(dstAbs, buf, s, false);
      if (!ok) ok = activeFs.writeFile(dstAbs, buf, s, 0);
    }
    free(buf);
    Console.println(ok ? "cp: ok" : "cp: write failed");
  } else if (!strcmp(t0, "mv")) {
    char* srcArg;
    char* dstArg;
    if (!nextToken(p, srcArg) || !nextToken(p, dstArg)) {
      Console.println("usage: mv <src> <dst|folder/>");
      return;
    }
    char srcAbs[ActiveFS::MAX_NAME + 1];
    if (!buildAbsFile(srcAbs, sizeof(srcAbs), g_cwd, srcArg)) {
      Console.println("mv: invalid source path");
      return;
    }
    if (!activeFs.exists(srcAbs)) {
      Console.println("mv: source not found");
      return;
    }
    uint32_t srcAddr = 0, srcSize = 0, srcCap = 0;
    if (!activeFs.getFileInfo(srcAbs, srcAddr, srcSize, srcCap)) {
      Console.println("mv: getFileInfo failed");
      return;
    }
    char dstAbs[ActiveFS::MAX_NAME + 1];
    size_t Ldst = strlen(dstArg);
    bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
    if (dstIsFolder) {
      char folderNoSlash[ActiveFS::MAX_NAME + 1];
      if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), g_cwd, dstArg, false)) {
        Console.println("mv: destination folder path too long");
        return;
      }
      const char* base = lastSlash(srcAbs);
      if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
        Console.println("mv: resulting path too long");
        return;
      }
    } else {
      if (!buildAbsFile(dstAbs, sizeof(dstAbs), g_cwd, dstArg)) {
        Console.println("mv: invalid destination path");
        return;
      }
    }
    normalizePathInPlace(dstAbs, false);
    if (strcmp(srcAbs, dstAbs) == 0) {
      Console.println("mv: source and destination are the same");
      return;
    }
    if (pathTooLongForOnDisk(dstAbs)) {
      Console.println("mv: destination name too long (FS=32 chars)");
      return;
    }
    uint8_t* buf = (uint8_t*)malloc(srcSize ? srcSize : 1);
    if (!buf) {
      Console.println("mv: malloc failed");
      return;
    }
    uint32_t got = activeFs.readFile(srcAbs, buf, srcSize);
    if (got != srcSize) {
      Console.println("mv: read failed");
      free(buf);
      return;
    }
    uint32_t eraseAlign = getEraseAlign();
    uint32_t reserve = srcCap;
    if (reserve < eraseAlign) {
      uint32_t a2 = (srcSize + (eraseAlign - 1)) & ~(eraseAlign - 1);
      if (a2 > reserve) reserve = a2;
    }
    if (reserve < eraseAlign) reserve = eraseAlign;
    bool ok = false;
    if (!activeFs.exists(dstAbs)) ok = activeFs.createFileSlot(dstAbs, reserve, buf, srcSize);
    else {
      uint32_t dA, dS, dC;
      if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
      if (dC >= srcSize) ok = activeFs.writeFileInPlace(dstAbs, buf, srcSize, false);
      if (!ok) ok = activeFs.writeFile(dstAbs, buf, srcSize, 0);
    }
    if (!ok) {
      Console.println("mv: write to destination failed");
      free(buf);
      return;
    }
    if (!activeFs.deleteFile(srcAbs)) Console.println("mv: warning: source delete failed");
    else Console.println("mv: ok");
    free(buf);
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
    Console.println(mkdirFolder(marker) ? "mkdir: ok" : "mkdir: failed");
  } else if (!strcmp(t0, "touch")) {
    char* pathArg;
    if (!nextToken(p, pathArg)) {
      Console.println("usage: touch <path|name|folder/>");
      return;
    }
    Console.println(touchPath(g_cwd, pathArg) ? "ok" : "touch failed");
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
    if (n == 0) {
      Console.println("(empty)");
      return;
    }
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
      Console.println(activeFs.deleteFile(folder) ? "rmdir: ok" : "rmdir: failed to remove marker");
    } else {
      Console.println("rmdir: marker not found (already removed?)");
    }
  } else if (!strcmp(t0, "df")) {
    cmdDf();
  } else if (!strcmp(t0, "rescanfs")) {
    size_t found = uniMem.rescan(50000UL);
    Console.printf("Unified scan: found %u device(s)\n", (unsigned)found);
    for (size_t i = 0; i < uniMem.detectedCount(); ++i) {
      const auto* di = uniMem.detectedInfo(i);
      if (!di) continue;
      Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes\n",
                     di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName,
                     (unsigned long long)di->capacityBytes);
    }
  } else if (!strcmp(t0, "format")) {
    Console.println(activeFs.format && activeFs.format() ? "FS formatted" : "format failed");
  } else if (!strcmp(t0, "wipe")) {
    Console.println("Erasing entire chip... this can take a while");
    Console.println(activeFs.wipeChip && activeFs.wipeChip() ? "Chip wiped" : "wipe failed");
  } else if (!strcmp(t0, "puthex")) {
    char* fn;
    char* hex;
    if (!nextToken(p, fn) || !nextToken(p, hex)) {
      Console.println("usage: puthex <file> <hex>");
      Console.println("tip: xxd -p -c 999999 your.bin | tr -d '\\n'");
      return;
    }
    if (strlen(fn) > ActiveFS::MAX_NAME) {
      Console.println("puthex: name too long");
      return;
    }
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    auto decodeHexString = [](const char* hex, uint8_t*& out, uint32_t& outLen) -> bool {
      if (!hex) return false;
      size_t L = strlen(hex);
      if (L == 0) {
        out = nullptr;
        outLen = 0;
        return true;
      }
      if (L & 1u) return false;
      outLen = (uint32_t)(L / 2);
      out = (uint8_t*)malloc(outLen ? outLen : 1);
      if (!out) return false;
      auto hexVal = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        return -1;
      };
      for (size_t i = 0; i < L; i += 2) {
        int hi = hexVal(hex[i]);
        int lo = hexVal(hex[i + 1]);
        if (hi < 0 || lo < 0) {
          free(out);
          out = nullptr;
          outLen = 0;
          return false;
        }
        out[i / 2] = (uint8_t)((hi << 4) | lo);
      }
      return true;
    };
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
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec may reject (needs even bytes).");
    } else Console.println("puthex: write failed");
  } else if (!strcmp(t0, "meminfo")) {
    Console.println();
#ifdef ARDUINO_ARCH_RP2040
    Console.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Console.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Console.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Console.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Console.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Console.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
#else
    Console.println("RP2040 meminfo not available on this core");
#endif
  } else if (!strcmp(t0, "isp")) {
    char* arg = nullptr;
    if (!nextToken(p, arg)) {
      Console.println("usage: isp enter|exit");
      return;
    }
    if (!strcmp(arg, "enter")) {
      if (!g_isp_active) {
        ArbiterISP::initTestPins();
        ArbiterISP::enterISPMode();
        g_isp_active = true;
      }
      Console.println("ISP: entered");
    } else if (!strcmp(arg, "exit")) {
      if (g_isp_active) {
        ArbiterISP::exitISPMode();
        ArbiterISP::cleanupToResetState();
        g_isp_active = false;
      }
      Console.println("ISP: exited");
    } else {
      Console.println("usage: isp enter|exit");
    }
  } else if (!strcmp(t0, "dacperf")) {
    Console.println("Running DAC performance test..");
    extern void performance_test();
    performance_test();
  } else if (!strcmp(t0, "reset")) {
    Console.println("Rebooting..");
    delay(20);
    yield();
    watchdog_reboot(0, 0, 1500);
  } else {
    Console.println("Unknown command. Type 'help'.");
  }
}
static void handleConsoleLine(char* line) {
  handleCommand(line);
}
static void pumpConsole() {
  if (!Serial) return;
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      Serial.write('\n');
      g_lineBuf[g_lineLen] = 0;
      if (g_lineLen > 0) {
        handleConsoleLine(g_lineBuf);
      }
      g_lineLen = 0;
      g_prompted = false;
      consolePromptOnce();
      break;                                // handle at most one line per pump
    } else if (ch == 0x08 || ch == 0x7F) {  // backspace/delete
      if (g_lineLen > 0) {
        g_lineLen--;
        Serial.write('\b');
        Serial.write(' ');
        Serial.write('\b');
      }
    } else if (ch == 0x15) {  // Ctrl-U: kill line
      while (g_lineLen > 0) {
        g_lineLen--;
        Serial.write('\b');
        Serial.write(' ');
        Serial.write('\b');
      }
    } else if (isprint(ch) && g_lineLen + 1 < sizeof(g_lineBuf)) {
      g_lineBuf[g_lineLen++] = (char)ch;
      Serial.write((char)ch);
    }
  }
  consolePromptOnce();
}

// ------- Serial byte helpers -------
static bool readByte(uint8_t& b, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) <= timeoutMs) {
    pumpConsole();
    int a = coproclink.available();
    if (a > 0) {
      int v = coproclink.read();
      if (v >= 0) {
        b = (uint8_t)v;
        return true;
      }
    }
    serviceISPIfActive();
    tight_loop_contents();
    yield();
  }
  return false;
}
static bool readExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
  for (size_t i = 0; i < n; ++i) {
    if (!readByte(dst[i], timeoutPerByteMs)) return false;
  }
  return true;
}
static bool writeAll(const uint8_t* src, size_t n, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t off = 0;
  while (off < n) {
    size_t wrote = coproclink.write(src + off, n - off);
    if (wrote > 0) {
      off += wrote;
      continue;
    }
    if ((millis() - start) > timeoutMs) return false;
    yield();
  }
  coproclink.flush();
  return true;
}
// ------- ISP handlers -------
static int32_t handleISP_ENTER(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  (void)in;
  (void)len;
  if (g_isp_active) {
    int32_t st = CoProc::ST_STATE;
    CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
    DBG("[DBG] ISP_ENTER: already active\n");
#endif
    return st;
  }
  ArbiterISP::initTestPins();
  ArbiterISP::enterISPMode();
  g_isp_active = true;
  int32_t st = CoProc::ST_OK;
  CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
  DBG("[DBG] ISP_ENTER: entered ISP mode\n");
#endif
  return st;
}
static int32_t handleISP_EXIT(uint8_t* out, size_t cap, size_t& off) {
  if (!g_isp_active) {
    int32_t st = CoProc::ST_STATE;
    CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
    DBG("[DBG] ISP_EXIT: not active\n");
#endif
    return st;
  }
  ArbiterISP::exitISPMode();
  ArbiterISP::cleanupToResetState();
  g_isp_active = false;
  int32_t st = CoProc::ST_OK;
  CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
  DBG("[DBG] ISP_EXIT: exited ISP mode\n");
#endif
  return st;
}
// Command names (debug)
static const char* cmdName(uint16_t c) {
  switch (c) {
    case CoProc::CMD_HELLO: return "HELLO";
    case CoProc::CMD_INFO: return "INFO";
    case CoProc::CMD_LOAD_BEGIN: return "LOAD_BEGIN";
    case CoProc::CMD_LOAD_DATA: return "LOAD_DATA";
    case CoProc::CMD_LOAD_END: return "LOAD_END";
    case CoProc::CMD_EXEC: return "EXEC";
    case CoProc::CMD_STATUS: return "STATUS";
    case CoProc::CMD_MAILBOX_RD: return "MAILBOX_RD";
    case CoProc::CMD_CANCEL: return "CANCEL";
    case CoProc::CMD_RESET: return "RESET";
    case CoProc::CMD_SCRIPT_BEGIN: return "SCRIPT_BEGIN";
    case CoProc::CMD_SCRIPT_DATA: return "SCRIPT_DATA";
    case CoProc::CMD_SCRIPT_END: return "SCRIPT_END";
    case CoProc::CMD_SCRIPT_EXEC: return "SCRIPT_EXEC";
    case CoProc::CMD_FUNC: return "FUNC";
    case CoProc::CMD_ISP_ENTER: return "ISP_ENTER";
    case CoProc::CMD_ISP_EXIT: return "ISP_EXIT";
  }
  return "UNKNOWN";
}
// Build response from request (delegates exec/script commands to CoProcExec)
static void processRequest(const CoProc::Frame& hdr, const uint8_t* payload,
                           CoProc::Frame& respH, uint8_t* respBuf, uint32_t& respLen) {
  size_t off = 0;
  int32_t st = CoProc::ST_BAD_CMD;
  respLen = 0;
#if COPROC_DEBUG
  DBG("[DBG] CMD=%s (0x%02X) seq=%u len=%u\n",
      cmdName(hdr.cmd), (unsigned)hdr.cmd, (unsigned)hdr.seq, (unsigned)hdr.len);
#endif
  switch (hdr.cmd) {
    case CoProc::CMD_HELLO: st = g_exec.cmdHELLO(g_isp_active, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_INFO: st = g_exec.cmdINFO(g_isp_active, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_LOAD_BEGIN: st = g_exec.cmdLOAD_BEGIN(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_LOAD_DATA: st = g_exec.cmdLOAD_DATA(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_LOAD_END: st = g_exec.cmdLOAD_END(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_EXEC: st = g_exec.cmdEXEC(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_STATUS: st = g_exec.cmdSTATUS(respBuf, RESP_MAX, off); break;
    case CoProc::CMD_MAILBOX_RD: st = g_exec.cmdMAILBOX_RD(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_CANCEL: st = g_exec.cmdCANCEL(respBuf, RESP_MAX, off); break;
    case CoProc::CMD_RESET:
#if COPROC_DEBUG
      DBG("[DBG] RESET\n");
#endif
      delay(10);
      watchdog_reboot(0, 0, 1500);
      st = CoProc::ST_OK;
      break;
    case CoProc::CMD_SCRIPT_BEGIN: st = g_exec.cmdSCRIPT_BEGIN(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_SCRIPT_DATA: st = g_exec.cmdSCRIPT_DATA(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_SCRIPT_END: st = g_exec.cmdSCRIPT_END(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_SCRIPT_EXEC: st = g_exec.cmdSCRIPT_EXEC(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_FUNC: st = g_exec.cmdFUNC(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_ISP_ENTER: st = handleISP_ENTER(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_ISP_EXIT: st = handleISP_EXIT(respBuf, RESP_MAX, off); break;
    default:
      st = CoProc::ST_BAD_CMD;
      CoProc::writePOD(respBuf, RESP_MAX, off, st);
      break;
  }
  respLen = (uint32_t)off;
  uint32_t crc = (respLen ? CoProc::crc32_ieee(respBuf, respLen) : 0);
  CoProc::makeResponseHeader(respH, hdr.cmd, hdr.seq, respLen, crc);
#if COPROC_DEBUG
  DBG("[DBG] RESP len=%u crc=0x%08X\n", (unsigned)respLen, (unsigned)crc);
#endif
}
// ------- Serial protocol helpers -------
static bool readFramedRequest(CoProc::Frame& hdr, uint8_t* payloadBuf) {
  const uint8_t magicBytes[4] = { (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0') };
  uint8_t w[4] = { 0, 0, 0, 0 };
  uint32_t lastActivity = millis();
  for (;;) {
    uint8_t b;
    if (readByte(b, 50)) {
      w[0] = w[1];
      w[1] = w[2];
      w[2] = w[3];
      w[3] = b;
      lastActivity = millis();
      if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
        union {
          CoProc::Frame f;
          uint8_t bytes[sizeof(CoProc::Frame)];
        } u;
        memcpy(u.bytes, w, 4);
        if (!readExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) {
          DBG("[COPROC] header tail timeout\n");
          return false;
        }
        hdr = u.f;
        if ((hdr.magic != CoProc::MAGIC) || (hdr.version != CoProc::VERSION)) {
          DBG("[COPROC] bad header magic=0x%08X ver=0x%04X\n", (unsigned)hdr.magic, (unsigned)hdr.version);
          return false;
        }
        if (hdr.len > REQ_MAX) {
          DBG("[COPROC] req too large (%u > %u), draining\n", (unsigned)hdr.len, (unsigned)REQ_MAX);
          uint8_t sink[64];
          uint32_t left = hdr.len;
          while (left) {
            uint32_t chunk = (left > sizeof(sink)) ? sizeof(sink) : left;
            if (!readExact(sink, chunk, 200)) {
              DBG("[COPROC] drain timeout\n");
              return false;
            }
            left -= chunk;
          }
          size_t off = 0;
          int32_t st = CoProc::ST_SIZE;
          CoProc::writePOD(g_respBuf, RESP_MAX, off, st);
          uint32_t crc = CoProc::crc32_ieee(g_respBuf, off);
          CoProc::makeResponseHeader(g_respHdr, hdr.cmd, hdr.seq, (uint32_t)off, crc);
          writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000);
          if (off) writeAll(g_respBuf, off, 5000);
          return false;
        }
        if (hdr.len) {
          if (!readExact(payloadBuf, hdr.len, 200)) {
            DBG("[COPROC] payload read timeout\n");
            return false;
          }
          uint32_t crc = CoProc::crc32_ieee(payloadBuf, hdr.len);
          if (crc != hdr.crc32) {
            DBG("[COPROC] CRC mismatch exp=0x%08X got=0x%08X\n", (unsigned)hdr.crc32, (unsigned)crc);
            size_t off = 0;
            int32_t st = CoProc::ST_CRC;
            CoProc::writePOD(g_respBuf, RESP_MAX, off, st);
            uint32_t rcrc = CoProc::crc32_ieee(g_respBuf, off);
            CoProc::makeResponseHeader(g_respHdr, hdr.cmd, hdr.seq, (uint32_t)off, rcrc);
            writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000);
            if (off) writeAll(g_respBuf, off, 5000);
            return false;
          }
        }
        return true;
      }
    } else {
      if ((millis() - lastActivity) > 1000) {
        lastActivity = millis();
      }
      pumpConsole();
      serviceISPIfActive();
      tight_loop_contents();
      yield();
    }
  }
}
// ------- Core0: serial protocol loop -------
static void protocolLoop() {
  for (;;) {
    serviceISPIfActive();
    pumpConsole();
    if (!readFramedRequest(g_reqHdr, g_reqBuf)) {
      continue;
    }
    uint32_t respLen = 0;
    processRequest(g_reqHdr, g_reqBuf, g_respHdr, g_respBuf, respLen);
    if (!writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000)) {
      DBG("[COPROC] write header failed\n");
      continue;
    }
    if (respLen) {
      if (!writeAll(g_respBuf, respLen, 10000)) {
        DBG("[COPROC] write payload failed\n");
        continue;
      }
    }
    pumpConsole();
    serviceISPIfActive();
    if (BOOTSEL) {
      Serial.println("Rebooting now..");
      delay(1500);
      yield();
      watchdog_reboot(0, 0, 1500);
      while (true) tight_loop_contents();
    }
  }
}
// ------- rp_selfupdate -------
const uint8_t PIN_UPDATE = 22;
volatile bool doUpdate = false;
void onTrig() {
  doUpdate = true;
}
// Example user function implementations
static int32_t fn_ping(const int32_t*, uint32_t) {
  return 1234;
}
static int32_t fn_add(const int32_t* a, uint32_t n) {
  int64_t sum = 0;
  for (uint32_t i = 0; i < n; ++i) sum += a[i];
  return (int32_t)sum;
}
static int32_t fn_tone(const int32_t* a, uint32_t n) {
  if (n < 3) return CoProc::ST_PARAM;
  int pin = (int)a[0], half = (int)a[1], cyc = (int)a[2];
  if (pin < 0 || half <= 0 || cyc <= 0) return CoProc::ST_PARAM;
  pinMode((uint8_t)pin, OUTPUT);
  for (int i = 0; i < cyc; ++i) {
    if (g_cancel_flag) break;
    digitalWrite((uint8_t)pin, HIGH);
    delayMicroseconds((uint32_t)half);
    digitalWrite((uint8_t)pin, LOW);
    delayMicroseconds((uint32_t)half);
    tight_loop_contents();
    yield();
  }
  return CoProc::ST_OK;
}
#include "MCP_DAC.h"
//MCP4921 MCP(19, 18);  //  SW SPI
MCP4921 MCP;  //  HW SPI
volatile int x;
uint32_t start, stop;
void performance_test() {
  Console.println();
  Console.println(__FUNCTION__);
  start = micros();
  for (uint16_t value = 0; value < MCP.maxValue(); value++) {
    x = MCP.write(value, 0);
  }
  stop = micros();
  Console.print(MCP.maxValue());
  Console.print(" x MCP.write():\t");
  Console.print(stop - start);
  Console.print("\t");
  Console.println((stop - start) / (MCP.maxValue() + 1.0));
  delay(10);
  start = micros();
  for (uint16_t value = 0; value < MCP.maxValue(); value++) {
    MCP.fastWriteA(value);
  }
  stop = micros();
  Console.print(MCP.maxValue());
  Console.print(" x MCP.fastWriteA():\t");
  Console.print(stop - start);
  Console.print("\t");
  Console.println((stop - start) / (MCP.maxValue() + 1.0));
  delay(10);
}
// ========== Setup and main ==========
void setup() {
  delay(500);
  Serial.begin();
  if (!Serial) delay(1000);
  delay(5000);
  Serial.println("CoProc (soft-serial) booting...");
  // Arbiter + CBTLV3257 wiring help (this MCU = Owner B)
  /*Serial.println("External SPI arbiter wiring (this MCU = Owner B):");
  ... wiring notes ... */
  ArbiterISP::initTestPins();
  ArbiterISP::cleanupToResetState();
  // Protocol buffers
  g_reqBuf = (uint8_t*)malloc(REQ_MAX);
  g_respBuf = (uint8_t*)malloc(RESP_MAX);
  // Software serial coproclink
  coproclink.begin(SOFT_BAUD);
  coproclink.listen();
  // Executor
  g_exec.begin();
  g_exec.registerFunc("ping", 0, fn_ping);
  g_exec.registerFunc("add", -1, fn_add);
  g_exec.registerFunc("tone", 3, fn_tone);
  // Enable arbiter guard (Co-Proc B)
  //UnifiedSpiMem::ExternalArbiter::begin(/*REQ*/ 4, /*GRANT*/ 5, /*reqActiveLow*/ true, /*grantActiveHigh*/ true, /*defaultAcquireMs*/ 1000, /*shortAcquireMs*/ 300);
  uniMem.begin();
  uniMem.setPreservePsramContents(true);
  uniMem.setCsList({ PIN_PSRAM_CS0 });
  // Keep slow scan for robustness
  size_t found = uniMem.rescan(50000UL);
  Console.printf("Unified scan: found %u device(s)\n", (unsigned)found);
  for (size_t i = 0; i < uniMem.detectedCount(); ++i) {
    const auto* di = uniMem.detectedInfo(i);
    if (!di) continue;
    Console.printf("  CS=%u  \tType=%s \tVendor=%s \tCap=%llu bytes\n", di->cs, UnifiedSpiMem::deviceTypeName(di->type), di->vendorName, (unsigned long long)di->capacityBytes);
  }
  bool psramOk = fsPSRAM.begin(uniMem);
  if (!psramOk) Console.println("PSRAM FS: no suitable device found or open failed");
  bindActiveFsPSRAM();
  if (psramOk) {
    bool m = activeFs.mount(false);
    Console.println(m ? "PSRAM FS mounted" : "PSRAM FS mount failed");
  }
  SPI.begin();
  MCP.begin(17);
  Console.print("DAC CHANNELS:\t");
  Console.println(MCP.channels());
  Console.print("DAC MAXVALUE:\t");
  Console.println(MCP.maxValue());
  performance_test();
  Serial.printf("CoProc ready (soft-serial %u bps). RX=GP%u TX=GP%u\n", (unsigned)SOFT_BAUD, (unsigned)PIN_RX, (unsigned)PIN_TX);
  Console.println("USB console ready. Type 'help' for commands.");
  consolePromptOnce();
}
void loop() {
  protocolLoop();
}  // never returns
// setup1/loop1 for core1 (Philhower)
void setup1() { /* nothing */
}
void loop1() {
  g_exec.workerPoll();
  tight_loop_contents();
}