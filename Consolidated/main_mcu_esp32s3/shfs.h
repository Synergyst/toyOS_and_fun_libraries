#pragma once
// shfs.h - Single-header SimpleFS helpers (ActiveFS + folders + cp/mv + fscp + df/lsdebug)
// Depends on: Arduino core + UnifiedSPIMemSimpleFS.h
// Usage:
//   1) #include "UnifiedSPIMemSimpleFS.h"
//   2) #include "shfs_all.h"
//   3) After creating your fsFlash/fsPSRAM/fsNAND and uniMem in setup(), call:
//        shfs_bindDevices(&fsFlash, &fsPSRAM, &fsNAND, &uniMem);
//        shfs_setPrint(&Console); // optional; defaults to Serial
//   4) Use bindActiveFs(g_storage) and the rest of the functions as before.

#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include "UnifiedSPIMemSimpleFS.h"

// ---------------- Configuration defaults (override before include if needed) ----------------
#ifndef SHFS_SECTOR_SIZE
#define SHFS_SECTOR_SIZE 4096u
#endif
#ifndef SHFS_MAX_NAME
#define SHFS_MAX_NAME 32u
#endif
#ifndef SHFS_DIR_HEAD_BYTES
#define SHFS_DIR_HEAD_BYTES (64u * 1024u)
#endif

// ---------------- External device pointers (bound at runtime by shfs_bindDevices) -----------
static W25QUnifiedSimpleFS* shfs_pFlash = nullptr;
static PSRAMUnifiedSimpleFS* shfs_pPSRAM = nullptr;
static MX35UnifiedSimpleFS* shfs_pNAND = nullptr;
static UnifiedSpiMem::Manager* shfs_pMgr = nullptr;

// Output sink (defaults to Serial)
static Print* shfs_out = &Serial;
inline void shfs_setPrint(Print* p) {
  if (p) shfs_out = p;
}

// Bind device facades/manager pointers
inline void shfs_bindDevices(W25QUnifiedSimpleFS* flash, PSRAMUnifiedSimpleFS* psram, MX35UnifiedSimpleFS* nand, UnifiedSpiMem::Manager* mgr) {
  shfs_pFlash = flash;
  shfs_pPSRAM = psram;
  shfs_pNAND = nand;
  shfs_pMgr = mgr;
}

// ---------------- Storage backend enum + active selection ----------------
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND,
  NAND,
};
static StorageBackend g_storage = StorageBackend::Flash;

// ---------------- ActiveFS facade (function-pointer vtable) ---------------
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
  static constexpr uint32_t SECTOR_SIZE = SHFS_SECTOR_SIZE;
  static constexpr uint32_t PAGE_SIZE = 256;
  static constexpr size_t MAX_NAME = SHFS_MAX_NAME;
} activeFs;

// FS iface (for cross-copy)
struct FSIface {
  bool (*mount)(bool);
  bool (*exists)(const char*);
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t);
  bool (*writeFile)(const char*, const uint8_t*, uint32_t, int);
  bool (*writeFileInPlace)(const char*, const uint8_t*, uint32_t, bool);
  uint32_t (*readFile)(const char*, uint8_t*, uint32_t);
  bool (*getFileInfo)(const char*, uint32_t&, uint32_t&, uint32_t&);
};

// ---------------- Device helpers ------------------------------------------
static inline UnifiedSpiMem::MemDevice* shfs_deviceFor(StorageBackend b) {
  switch (b) {
    case StorageBackend::Flash: return (shfs_pFlash ? shfs_pFlash->raw().device() : nullptr);
    case StorageBackend::PSRAM_BACKEND: return (shfs_pPSRAM ? shfs_pPSRAM->raw().device() : nullptr);
    case StorageBackend::NAND: return (shfs_pNAND ? shfs_pNAND->raw().device() : nullptr);
  }
  return nullptr;
}
static inline UnifiedSpiMem::MemDevice* activeFsDevice() {
  return shfs_deviceFor(g_storage);
}
static inline uint32_t dirEntryStride() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 32u;
  return (dev->type() == UnifiedSpiMem::DeviceType::SpiNandMX35) ? dev->pageSize() : 32u;
}
static inline uint32_t getEraseAlign() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return ActiveFS::SECTOR_SIZE;
  uint32_t e = dev->eraseSize();
  return (e > 0) ? e : ActiveFS::SECTOR_SIZE;
}
static inline uint32_t getEraseAlignFor(StorageBackend b) {
  UnifiedSpiMem::MemDevice* dev = shfs_deviceFor(b);
  if (!dev) return ActiveFS::SECTOR_SIZE;
  uint32_t e = dev->eraseSize();
  return (e > 0) ? e : ActiveFS::SECTOR_SIZE;
}
static inline int fsReplaceMode() {
  switch (g_storage) {
    case StorageBackend::Flash: return (int)W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::NAND: return (int)MX35UnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::PSRAM_BACKEND: return 0;
  }
  return 0;
}
static inline int fsReplaceModeFor(StorageBackend b) {
  switch (b) {
    case StorageBackend::Flash: return (int)W25QUnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::NAND: return (int)MX35UnifiedSimpleFS::WriteMode::ReplaceIfExists;
    case StorageBackend::PSRAM_BACKEND: return 0;
  }
  return 0;
}

// ---------------- Bind activeFs to selected backend ------------------------
inline void bindActiveFs(StorageBackend backend) {
  // Safety guards
  if (!shfs_pFlash && !shfs_pPSRAM && !shfs_pNAND) {
    // nothing bound; leave activeFs null
    return;
  }
  if (backend == StorageBackend::Flash && shfs_pFlash) {
    activeFs.mount = [](bool b) {
      return shfs_pFlash->mount(b);
    };
    activeFs.format = []() {
      return shfs_pFlash->format();
    };
    activeFs.wipeChip = []() {
      return shfs_pFlash->wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return shfs_pFlash->exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return shfs_pFlash->createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return shfs_pFlash->writeFile(n, d, s, static_cast<W25QUnifiedSimpleFS::WriteMode>(m));
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return shfs_pFlash->writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return shfs_pFlash->readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return shfs_pFlash->readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return shfs_pFlash->getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return shfs_pFlash->getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return shfs_pFlash->deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      shfs_pFlash->listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return shfs_pFlash->nextDataAddr();
    };
    activeFs.capacity = []() {
      return shfs_pFlash->capacity();
    };
    activeFs.dataRegionStart = []() {
      return shfs_pFlash->dataRegionStart();
    };
  } else if (backend == StorageBackend::NAND && shfs_pNAND) {
    activeFs.mount = [](bool b) {
      return shfs_pNAND->mount(b);
    };
    activeFs.format = []() {
      return shfs_pNAND->format();
    };
    activeFs.wipeChip = []() {
      return shfs_pNAND->wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return shfs_pNAND->exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return shfs_pNAND->createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return shfs_pNAND->writeFile(n, d, s, static_cast<MX35UnifiedSimpleFS::WriteMode>(m));
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return shfs_pNAND->writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return shfs_pNAND->readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return shfs_pNAND->readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return shfs_pNAND->getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return shfs_pNAND->getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return shfs_pNAND->deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      shfs_pNAND->listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return shfs_pNAND->nextDataAddr();
    };
    activeFs.capacity = []() {
      return shfs_pNAND->capacity();
    };
    activeFs.dataRegionStart = []() {
      return shfs_pNAND->dataRegionStart();
    };
  } else if (shfs_pPSRAM) {
    activeFs.mount = [](bool b) {
      (void)b;
      return shfs_pPSRAM->mount(false);
    };
    activeFs.format = []() {
      return shfs_pPSRAM->format();
    };
    activeFs.wipeChip = []() {
      return shfs_pPSRAM->wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return shfs_pPSRAM->exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return shfs_pPSRAM->createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return shfs_pPSRAM->writeFile(n, d, s, m);
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return shfs_pPSRAM->writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return shfs_pPSRAM->readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return shfs_pPSRAM->readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return shfs_pPSRAM->getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return shfs_pPSRAM->getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return shfs_pPSRAM->deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      shfs_pPSRAM->listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return shfs_pPSRAM->nextDataAddr();
    };
    activeFs.capacity = []() {
      return shfs_pPSRAM->capacity();
    };
    activeFs.dataRegionStart = []() {
      return shfs_pPSRAM->dataRegionStart();
    };
  }
}

// ---------------- Path and folder helpers ----------------------------------
static char g_cwd[ActiveFS::MAX_NAME + 1] = "";  // "" = root

static inline void pathStripTrailingSlashes(char* p) {
  if (!p) return;
  size_t n = strlen(p);
  while (n > 0 && p[n - 1] == '/') { p[--n] = 0; }
}
static inline void normalizePathInPlace(char* s, bool wantTrailingSlash) {
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
    if (n > 0 && s[n - 1] != '/' && n + 1 < ActiveFS::MAX_NAME + 1) {
      s[n] = '/';
      s[n + 1] = 0;
    }
  } else {
    while (n > 0 && s[n - 1] == '/') { s[--n] = 0; }
  }
}
static inline bool pathJoin(char* out, size_t outCap, const char* base, const char* name, bool wantTrailingSlash) {
  if (!out || !name) return false;

  if (name[0] == '/') {
    const char* s = name + 1;
    size_t L = strlen(s);
    if (wantTrailingSlash && L > 0 && s[L - 1] != '/') {
      if (L + 1 > ActiveFS::MAX_NAME || L + 2 > outCap) return false;
      // Copy into a temp so we can append slash safely.
      char tmp[ActiveFS::MAX_NAME + 1];
      if (L >= sizeof(tmp)) return false;
      memcpy(tmp, s, L);
      tmp[L++] = '/';
      tmp[L] = 0;
      if (L + 1 > outCap) return false;
      memcpy(out, tmp, L + 1);
      return true;
    }
    // No trailing slash needed, copy as-is
    if (L > ActiveFS::MAX_NAME || L + 1 > outCap) return false;
    memcpy(out, s, L + 1);
    return true;
  }

  if (!base) base = "";
  char tmp[ActiveFS::MAX_NAME + 1];
  int need = 0;
  if (base[0] == 0) need = snprintf(tmp, sizeof(tmp), "%s", name);
  else if (name[0] == 0) need = snprintf(tmp, sizeof(tmp), "%s", base);
  else need = snprintf(tmp, sizeof(tmp), "%s/%s", base, name);

  if (need < 0 || (size_t)need > ActiveFS::MAX_NAME || (size_t)need >= sizeof(tmp)) return false;

  size_t L = (size_t)need;
  if (wantTrailingSlash && L > 0 && tmp[L - 1] != '/') {
    if (L + 1 > ActiveFS::MAX_NAME || L + 2 > sizeof(tmp)) return false;
    tmp[L++] = '/';
    tmp[L] = 0;
  }

  if (L + 1 > outCap) return false;
  memcpy(out, tmp, L + 1);
  return true;
}
static inline void pathParent(char* p) {
  if (!p) return;
  pathStripTrailingSlashes(p);
  size_t n = strlen(p);
  if (n == 0) return;
  char* last = strrchr(p, '/');
  if (!last || last == p) {
    p[0] = 0;
  } else {
    *last = 0;
  }
}
static inline bool makePathSafe(char* out, size_t outCap, const char* folder, const char* name) {
  if (!out || !folder || !name) return false;
  char tmp[ActiveFS::MAX_NAME + 1];
  if (folder[0] == 0) {
    if (snprintf(tmp, sizeof(tmp), "%s", name) < 0) return false;
  } else {
    if (snprintf(tmp, sizeof(tmp), "%s/%s", folder, name) < 0) return false;
  }
  normalizePathInPlace(tmp, false);
  size_t L = strlen(tmp);
  if (L > ActiveFS::MAX_NAME || L >= outCap) return false;
  memcpy(out, tmp, L + 1);
  return true;
}
static inline bool makePath(char* out, size_t outCap, const char* folder, const char* name) {
  if (!out || !folder || !name) return false;
  int needed = snprintf(out, outCap, "%s/%s", folder, name);
  if (needed < 0) return false;
  if ((size_t)needed > ActiveFS::MAX_NAME || (size_t)needed >= outCap) return false;
  return true;
}
static inline const char* lastSlash(const char* s) {
  const char* p = strrchr(s, '/');
  return p ? (p + 1) : s;
}
static inline const char* firstSlash(const char* s) {
  return strchr(s, '/');
}
static constexpr size_t FS_NAME_ONDISK_MAX = SHFS_MAX_NAME;
static inline bool pathTooLongForOnDisk(const char* full) {
  return strlen(full) > FS_NAME_ONDISK_MAX;
}

static inline bool folderExists(const char* absFolder) {
  if (!absFolder || !absFolder[0]) return false;
  size_t L = strlen(absFolder);
  if (L == 0 || absFolder[L - 1] != '/') return false;
  return activeFs.exists && activeFs.exists(absFolder);
}
static inline bool mkdirFolder(const char* path) {
  if (!activeFs.writeFile) return false;
  if (activeFs.exists && activeFs.exists(path)) return true;
  return activeFs.writeFile(path, nullptr, 0, fsReplaceMode());
}
static inline bool touchPath(const char* cwd, const char* arg) {
  if (!arg) return false;
  size_t L = strlen(arg);
  if (L > 0 && arg[L - 1] == '/') {
    char marker[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(marker, sizeof(marker), cwd, arg, true)) {
      shfs_out->println("touch: folder path too long (<= 32 chars)");
      return false;
    }
    if (folderExists(marker)) return true;
    return mkdirFolder(marker);
  }
  char path[ActiveFS::MAX_NAME + 1];
  if (!pathJoin(path, sizeof(path), cwd, arg, false)) {
    shfs_out->println("touch: path too long (<= 32 chars)");
    return false;
  }
  if (activeFs.exists && activeFs.exists(path)) return true;
  return activeFs.writeFile(path, nullptr, 0, fsReplaceMode());
}

// ---------------- cp/mv on active FS ---------------------------------------
static inline bool buildAbsFile(char* out, size_t outCap, const char* cwd, const char* path) {
  if (!path || !out) return false;
  size_t L = strlen(path);
  if (L > 0 && path[L - 1] == '/') return false;
  return pathJoin(out, outCap, cwd, path, false);
}
static inline bool cmdMvImpl(const char* cwd, const char* srcArg, const char* dstArg) {
  if (!srcArg || !dstArg) return false;
  char srcAbs[ActiveFS::MAX_NAME + 1];
  if (!buildAbsFile(srcAbs, sizeof(srcAbs), cwd, srcArg)) {
    shfs_out->println("mv: invalid source path");
    return false;
  }
  bool srcExists = activeFs.exists && activeFs.exists(srcAbs);
  if (!srcExists && strstr(srcAbs, "//")) {
    char alt[sizeof(srcAbs)];
    strncpy(alt, srcAbs, sizeof(alt));
    alt[sizeof(alt) - 1] = 0;
    normalizePathInPlace(alt, false);
    if (activeFs.exists && activeFs.exists(alt)) {
      strncpy(srcAbs, alt, sizeof(srcAbs));
      srcAbs[sizeof(srcAbs) - 1] = 0;
      srcExists = true;
    }
  }
  if (!srcExists) {
    shfs_out->println("mv: source not found");
    return false;
  }
  uint32_t srcAddr = 0, srcSize = 0, srcCap = 0;
  if (!activeFs.getFileInfo || !activeFs.getFileInfo(srcAbs, srcAddr, srcSize, srcCap)) {
    shfs_out->println("mv: getFileInfo failed");
    return false;
  }
  char dstAbs[ActiveFS::MAX_NAME + 1];
  size_t Ldst = strlen(dstArg);
  bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
  if (dstIsFolder) {
    char folderNoSlash[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), cwd, dstArg, false)) {
      shfs_out->println("mv: destination folder path too long");
      return false;
    }
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
      shfs_out->println("mv: resulting path too long");
      return false;
    }
  } else {
    if (!buildAbsFile(dstAbs, sizeof(dstAbs), cwd, dstArg)) {
      shfs_out->println("mv: invalid destination path");
      return false;
    }
  }
  normalizePathInPlace(dstAbs, false);
  if (strcmp(srcAbs, dstAbs) == 0) {
    shfs_out->println("mv: source and destination are the same");
    return true;
  }
  if (pathTooLongForOnDisk(dstAbs)) {
    shfs_out->println("mv: destination name too long for FS (would be truncated)");
    return false;
  }
  uint8_t* buf = (uint8_t*)malloc(srcSize ? srcSize : 1);
  if (!buf) {
    shfs_out->println("mv: malloc failed");
    return false;
  }
  uint32_t got = activeFs.readFile(srcAbs, buf, srcSize);
  if (got != srcSize) {
    shfs_out->println("mv: read failed");
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
    if (dC >= srcSize) ok = activeFs.writeFileInPlace(dstAbs, buf, srcSize, false);
    if (!ok) ok = activeFs.writeFile(dstAbs, buf, srcSize, fsReplaceMode());
  }
  if (!ok) {
    shfs_out->println("mv: write to destination failed");
    free(buf);
    return false;
  }
  if (!activeFs.deleteFile(srcAbs)) {
    shfs_out->println("mv: warning: source delete failed");
  } else {
    shfs_out->println("mv: ok");
  }
  free(buf);
  return true;
}
static inline bool cmdCpImpl(const char* cwd, const char* srcArg, const char* dstArg, bool force) {
  if (!srcArg || !dstArg) return false;
  char srcAbs[ActiveFS::MAX_NAME + 1];
  if (!buildAbsFile(srcAbs, sizeof(srcAbs), cwd, srcArg)) {
    shfs_out->println("cp: invalid source path");
    return false;
  }
  if (!activeFs.exists(srcAbs)) {
    shfs_out->println("cp: source not found");
    return false;
  }
  uint32_t sAddr = 0, sSize = 0, sCap = 0;
  if (!activeFs.getFileInfo(srcAbs, sAddr, sSize, sCap)) {
    shfs_out->println("cp: getFileInfo failed");
    return false;
  }

  char dstAbs[ActiveFS::MAX_NAME + 1];
  size_t Ldst = strlen(dstArg);
  bool dstIsFolder = (Ldst > 0 && dstArg[Ldst - 1] == '/');
  if (dstIsFolder) {
    char folderNoSlash[ActiveFS::MAX_NAME + 1];
    if (!pathJoin(folderNoSlash, sizeof(folderNoSlash), cwd, dstArg, false)) {
      shfs_out->println("cp: destination folder path too long");
      return false;
    }
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), folderNoSlash, base)) {
      shfs_out->println("cp: resulting path too long");
      return false;
    }
  } else {
    if (!buildAbsFile(dstAbs, sizeof(dstAbs), cwd, dstArg)) {
      shfs_out->println("cp: invalid destination path");
      return false;
    }
  }
  normalizePathInPlace(dstAbs, false);
  if (pathTooLongForOnDisk(dstAbs)) {
    shfs_out->println("cp: destination name too long for FS (would be truncated)");
    return false;
  }
  if (activeFs.exists(dstAbs) && !force) {
    shfs_out->println("cp: destination exists (use -f to overwrite)");
    return false;
  }

  // Prepare destination: prefer keeping the same capacity as source (sCap), rounded to erase align.
  uint32_t eraseAlign = getEraseAlign();
  uint32_t reserve = sCap;
  if (reserve < eraseAlign) {
    uint32_t a = (sSize + (eraseAlign - 1)) & ~(eraseAlign - 1);
    if (a > reserve) reserve = a;
  }
  if (reserve < eraseAlign) reserve = eraseAlign;

  bool needCreate = !activeFs.exists(dstAbs);
  uint32_t dA = 0, dS = 0, dC = 0;
  if (!needCreate) {
    if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
  }

  // If destination missing: create slot now, write initial chunk in createFileSlot
  const size_t CHUNK = 2048;
  uint8_t* buf = (uint8_t*)malloc(CHUNK);
  if (!buf) {
    shfs_out->println("cp: OOM");
    return false;
  }

  uint32_t off = 0;
  bool ok = true;

  if (needCreate) {
    size_t n = (sSize > CHUNK) ? CHUNK : sSize;
    if (n) {
      if (activeFs.readFileRange(srcAbs, 0, buf, n) != n) {
        ok = false;
      } else ok = activeFs.createFileSlot(dstAbs, reserve, buf, n);
      off = n;
    } else {
      // empty source
      ok = activeFs.createFileSlot(dstAbs, reserve, nullptr, 0);
    }
    if (!ok) {
      shfs_out->println("cp: create/write failed");
      free(buf);
      return false;
    }
    if (!activeFs.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;  // refresh
  }

  // Continue writing remaining bytes
  while (ok && off < sSize) {
    size_t n = (sSize - off > CHUNK) ? CHUNK : (sSize - off);
    if (activeFs.readFileRange(srcAbs, off, buf, n) != n) {
      ok = false;
      break;
    }
    if (dC >= sSize) {
      // In-place update (append behavior implemented by full write-in-place if FS supports)
      ok = activeFs.writeFileInPlace(dstAbs, buf, off + n, true);
      // If the FS writeFileInPlace semantics do not support progressive append by total size,
      // fall back to full rewrite:
      if (!ok) ok = activeFs.writeFile(dstAbs, buf, off + n, fsReplaceMode());
    } else {
      // Not enough capacity -> rewrite whole file progressively (last write wins)
      ok = activeFs.writeFile(dstAbs, buf, off + n, fsReplaceMode());
    }
    off += n;
    yield();
  }

  free(buf);
  if (!ok) {
    shfs_out->println("cp: write failed");
    return false;
  }
  shfs_out->println("cp: ok");
  return true;
}

// ---------------- Minimal DIR scan / index (for ls/lsraw/lsdebug) ----------
struct FsIndexEntry {
  char name[ActiveFS::MAX_NAME + 1];
  uint32_t size;
  bool deleted;
  uint32_t seq;
};

static inline uint32_t rd32_be(const uint8_t* p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
}
static inline bool isAllFF(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i)
    if (p[i] != 0xFF) return false;
  return true;
}

static inline size_t buildFsIndex(FsIndexEntry* out, size_t outMax) {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 0;
  constexpr uint32_t DIR_START = 0x000000;
  constexpr uint32_t DIR_SIZE = SHFS_DIR_HEAD_BYTES;
  const uint32_t stride = dirEntryStride();
  FsIndexEntry map[64];
  size_t mapCount = 0;
  uint8_t rec[32];
  for (uint32_t off = 0; off + sizeof(rec) <= DIR_SIZE; off += stride) {
    if (dev->read(DIR_START + off, rec, sizeof(rec)) != sizeof(rec)) break;
    if (rec[0] != 0x57 || rec[1] != 0x46) continue;  // 'W','F'
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
static inline bool hasPrefix(const char* name, const char* prefix) {
  size_t lp = strlen(prefix);
  return strncmp(name, prefix, lp) == 0;
}
static inline bool childOfFolder(const char* name, const char* folderPrefix, const char*& childOut) {
  size_t lp = strlen(folderPrefix);
  if (strncmp(name, folderPrefix, lp) != 0) return false;
  const char* rest = name + lp;
  if (*rest == 0) return false;
  const char* slash = strchr(rest, '/');
  if (slash) return false;
  childOut = rest;
  return true;
}
static inline void dumpDirHeadRaw(uint32_t bytes = 256) {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) {
    shfs_out->println("lsdebug: no device");
    return;
  }
  if (bytes == 0 || bytes > 1024) bytes = 256;
  uint8_t buf[1024];
  size_t got = dev->read(0, buf, bytes);
  shfs_out->print("lsdebug: read ");
  shfs_out->print((unsigned)got);
  shfs_out->println(" bytes @ 0x000000");
  for (size_t i = 0; i < got; i += 16) {
    char line[24];
    snprintf(line, sizeof(line), "  %04u: ", (unsigned)i);
    shfs_out->print(line);
    for (size_t j = 0; j < 16 && (i + j) < got; ++j) {
      uint8_t b = buf[i + j];
      if (b < 16) shfs_out->print('0');
      shfs_out->print(b, HEX);
      shfs_out->print(' ');
    }
    shfs_out->println();
  }
}

// ---------------- df reporter ----------------------------------------------
static inline void shfs_printPct2(uint32_t num, uint32_t den) {
  if (den == 0) {
    shfs_out->print("n/a");
    return;
  }
  uint32_t scaled = (uint32_t)(((uint64_t)num * 10000ULL + (den / 2)) / den);
  char tmp[16];
  snprintf(tmp, sizeof(tmp), "%lu.%02lu%%",
           (unsigned long)(scaled / 100),
           (unsigned long)(scaled % 100));
  shfs_out->print(tmp);
}
static inline uint32_t dirBytesUsedEstimate() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (!dev) return 0;
  constexpr uint32_t DIR_START = 0x000000;
  constexpr uint32_t DIR_SIZE = SHFS_DIR_HEAD_BYTES;
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
static inline void cmdDf() {
  UnifiedSpiMem::MemDevice* dev = activeFsDevice();
  if (dev) {
    const auto t = dev->type();
    const char* style = UnifiedSpiMem::deviceTypeName(t);
    const uint8_t cs = dev->cs();
    const uint64_t devCap = dev->capacity();
    const uint32_t dataStart = activeFs.dataRegionStart ? activeFs.dataRegionStart() : 0;
    const uint32_t fsCap32 = activeFs.capacity ? activeFs.capacity() : 0;
    const uint32_t dataCap = (fsCap32 > dataStart) ? (fsCap32 - dataStart) : 0;
    const uint32_t dataUsed = (activeFs.nextDataAddr && activeFs.nextDataAddr() > dataStart)
                                ? (activeFs.nextDataAddr() - dataStart)
                                : 0;
    const uint32_t dataFree = (dataCap > dataUsed) ? (dataCap - dataUsed) : 0;
    const uint32_t dirUsed = dirBytesUsedEstimate();
    const uint32_t dirFree = (SHFS_DIR_HEAD_BYTES > dirUsed) ? (SHFS_DIR_HEAD_BYTES - dirUsed) : 0;

    shfs_out->println("Filesystem (active):");
    shfs_out->print("  Device:  ");
    shfs_out->print(style);
    shfs_out->print("  CS=");
    shfs_out->println((unsigned)cs);
    shfs_out->print("  DevCap:  ");
    shfs_out->print((unsigned long long)devCap);
    shfs_out->println(" bytes");
    shfs_out->print("  FS data: ");
    shfs_out->print((unsigned long)dataUsed);
    shfs_out->print(" used (");
    shfs_printPct2(dataUsed, dataCap);
    shfs_out->print(")  ");
    shfs_out->print((unsigned long)dataFree);
    shfs_out->print(" free (");
    shfs_printPct2(dataFree, dataCap);
    shfs_out->println(")");
    shfs_out->print("  DIR:     ");
    shfs_out->print((unsigned long)dirUsed);
    shfs_out->print(" used (");
    shfs_printPct2(dirUsed, SHFS_DIR_HEAD_BYTES);
    shfs_out->print(")  ");
    shfs_out->print((unsigned long)dirFree);
    shfs_out->println(" free");
  } else {
    shfs_out->println("Filesystem (active): none");
  }
  if (!shfs_pMgr) return;
  size_t n = shfs_pMgr->detectedCount();
  if (n == 0) return;
  shfs_out->println("Detected devices:");
  for (size_t i = 0; i < n; ++i) {
    const auto* di = shfs_pMgr->detectedInfo(i);
    if (!di) continue;
    bool isActive = false;
    UnifiedSpiMem::MemDevice* a = activeFsDevice();
    if (a) isActive = (di->cs == a->cs() && di->type == a->type());
    shfs_out->print("  CS=");
    shfs_out->print(di->cs);
    shfs_out->print("  \tType=");
    shfs_out->print(UnifiedSpiMem::deviceTypeName(di->type));
    shfs_out->print(" \tVendor=");
    shfs_out->print(di->vendorName);
    shfs_out->print(" \tCap=");
    shfs_out->print((unsigned long long)di->capacityBytes);
    shfs_out->print(" bytes");
    if (isActive) shfs_out->print("\t <-- [mounted]");
    shfs_out->println();
  }
}

// ---------------- Cross-filesystem copy (fscp) ------------------------------
static inline void fillFsIface(StorageBackend b, FSIface& out) {
  if (b == StorageBackend::Flash && shfs_pFlash) {
    out.mount = [](bool autoFmt) {
      return shfs_pFlash->mount(autoFmt);
    };
    out.exists = [](const char* n) {
      return shfs_pFlash->exists(n);
    };
    out.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return shfs_pFlash->createFileSlot(n, r, d, s);
    };
    out.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return shfs_pFlash->writeFile(n, d, s, static_cast<W25QUnifiedSimpleFS::WriteMode>(m));
    };
    out.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return shfs_pFlash->writeFileInPlace(n, d, s, a);
    };
    out.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return shfs_pFlash->readFile(n, b, sz);
    };
    out.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return shfs_pFlash->getFileInfo(n, a, s, c);
    };
  } else if (b == StorageBackend::NAND && shfs_pNAND) {
    out.mount = [](bool autoFmt) {
      return shfs_pNAND->mount(autoFmt);
    };
    out.exists = [](const char* n) {
      return shfs_pNAND->exists(n);
    };
    out.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return shfs_pNAND->createFileSlot(n, r, d, s);
    };
    out.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return shfs_pNAND->writeFile(n, d, s, static_cast<MX35UnifiedSimpleFS::WriteMode>(m));
    };
    out.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return shfs_pNAND->writeFileInPlace(n, d, s, a);
    };
    out.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return shfs_pNAND->readFile(n, b, sz);
    };
    out.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return shfs_pNAND->getFileInfo(n, a, s, c);
    };
  } else if (shfs_pPSRAM) {
    out.mount = [](bool autoFmt) {
      (void)autoFmt;
      return shfs_pPSRAM->mount(false);
    };
    out.exists = [](const char* n) {
      return shfs_pPSRAM->exists(n);
    };
    out.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return shfs_pPSRAM->createFileSlot(n, r, d, s);
    };
    out.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return shfs_pPSRAM->writeFile(n, d, s, m);
    };
    out.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return shfs_pPSRAM->writeFileInPlace(n, d, s, a);
    };
    out.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return shfs_pPSRAM->readFile(n, b, sz);
    };
    out.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return shfs_pPSRAM->getFileInfo(n, a, s, c);
    };
  } else {
    memset(&out, 0, sizeof(out));
  }
}
static inline bool parseBackendSpec(const char* spec, StorageBackend& outBackend, const char*& outPath) {
  if (!spec) return false;
  const char* colon = strchr(spec, ':');
  if (!colon) return false;
  size_t nameLen = (size_t)(colon - spec);
  if (nameLen == 0) return false;
  if (strncmp(spec, "flash", nameLen) == 0) outBackend = StorageBackend::Flash;
  else if (strncmp(spec, "psram", nameLen) == 0) outBackend = StorageBackend::PSRAM_BACKEND;
  else if (strncmp(spec, "nand", nameLen) == 0) outBackend = StorageBackend::NAND;
  else return false;
  outPath = colon + 1;
  return true;
}
static inline bool normalizeFsPathCopy(char* dst, size_t dstCap, const char* input, bool wantTrailingSlash) {
  if (!dst || !input) return false;
  const char* s = input;
  while (*s == '/') ++s;
  size_t L = strlen(s);
  if (L >= dstCap) return false;
  memcpy(dst, s, L + 1);
  normalizePathInPlace(dst, wantTrailingSlash);
  if (strlen(dst) > ActiveFS::MAX_NAME) return false;
  return true;
}
static inline bool cmdFsCpImpl(const char* srcSpec, const char* dstSpec, bool force) {
  StorageBackend sbSrc, sbDst;
  const char* srcPathIn = nullptr;
  const char* dstPathIn = nullptr;
  if (!parseBackendSpec(srcSpec, sbSrc, srcPathIn) || !parseBackendSpec(dstSpec, sbDst, dstPathIn)) {
    shfs_out->println("fscp: invalid backend spec; use flash:/path psram:/path nand:/path");
    return false;
  }
  FSIface srcFS{}, dstFS{};
  fillFsIface(sbSrc, srcFS);
  fillFsIface(sbDst, dstFS);
  if (!srcFS.mount || !dstFS.mount) {
    shfs_out->println("fscp: backend not available/bound");
    return false;
  }
  bool srcMounted = srcFS.mount((sbSrc != StorageBackend::PSRAM_BACKEND));
  bool dstMounted = dstFS.mount((sbDst != StorageBackend::PSRAM_BACKEND));
  if (!srcMounted) {
    shfs_out->println("fscp: source mount failed");
    return false;
  }
  if (!dstMounted) {
    shfs_out->println("fscp: destination mount failed");
    return false;
  }
  char srcAbs[ActiveFS::MAX_NAME + 1];
  char dstArgRaw[ActiveFS::MAX_NAME + 1];
  if (!normalizeFsPathCopy(srcAbs, sizeof(srcAbs), srcPathIn, false)) {
    shfs_out->println("fscp: source path too long (<=32)");
    return false;
  }
  size_t LdstIn = strlen(dstPathIn);
  bool dstAsFolder = (LdstIn > 0 && dstPathIn[LdstIn - 1] == '/');
  if (!normalizeFsPathCopy(dstArgRaw, sizeof(dstArgRaw), dstPathIn, dstAsFolder)) {
    shfs_out->println("fscp: destination path too long (<=32)");
    return false;
  }
  if (!srcFS.exists || !srcFS.exists(srcAbs)) {
    shfs_out->println("fscp: source not found");
    return false;
  }
  uint32_t sAddr = 0, sSize = 0, sCap = 0;
  if (!srcFS.getFileInfo || !srcFS.getFileInfo(srcAbs, sAddr, sSize, sCap)) {
    shfs_out->println("fscp: getFileInfo(source) failed");
    return false;
  }
  char dstAbs[ActiveFS::MAX_NAME + 1];
  if (dstAsFolder) {
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), dstArgRaw, base)) {
      shfs_out->println("fscp: resulting destination path too long");
      return false;
    }
  } else {
    strncpy(dstAbs, dstArgRaw, sizeof(dstAbs));
    dstAbs[sizeof(dstAbs) - 1] = 0;
  }
  if (pathTooLongForOnDisk(dstAbs)) {
    shfs_out->println("fscp: destination name too long for FS (would be truncated)");
    return false;
  }
  if (dstFS.exists && dstFS.exists(dstAbs) && !force) {
    shfs_out->println("fscp: destination exists (use -f to overwrite)");
    return false;
  }
  uint8_t* buf = (uint8_t*)malloc(sSize ? sSize : 1);
  if (!buf) {
    shfs_out->println("fscp: malloc failed");
    return false;
  }
  uint32_t got = srcFS.readFile(srcAbs, buf, sSize);
  if (got != sSize) {
    shfs_out->println("fscp: read failed");
    free(buf);
    return false;
  }
  uint32_t eraseAlign = getEraseAlignFor(sbDst);
  uint32_t reserve = sCap;
  if (reserve < eraseAlign) {
    uint32_t a = (sSize + (eraseAlign - 1)) & ~(eraseAlign - 1);
    if (a > reserve) reserve = a;
  }
  if (reserve < eraseAlign) reserve = eraseAlign;
  bool ok = false;
  if (!dstFS.exists(dstAbs)) {
    ok = dstFS.createFileSlot(dstAbs, reserve, buf, sSize);
  } else {
    uint32_t dA, dS, dC;
    if (!dstFS.getFileInfo(dstAbs, dA, dS, dC)) dC = 0;
    if (dC >= sSize) ok = dstFS.writeFileInPlace(dstAbs, buf, sSize, false);
    if (!ok) ok = dstFS.writeFile(dstAbs, buf, sSize, fsReplaceModeFor(sbDst));
  }
  free(buf);
  if (!ok) {
    shfs_out->println("fscp: write failed");
    return false;
  }
  shfs_out->println("fscp: ok");
  return true;
}