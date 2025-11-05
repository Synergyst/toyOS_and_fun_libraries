// ================== Cross-filesystem copy (fscp) ==================
static void fillFsIface(StorageBackend b, FSIface& out) {
  if (b == StorageBackend::Flash) {
    out.mount = [](bool autoFmt) {
      return fsFlash.mount(autoFmt);
    };
    out.exists = [](const char* n) {
      return fsFlash.exists(n);
    };
    out.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsFlash.createFileSlot(n, r, d, s);
    };
    out.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsFlash.writeFile(n, d, s, static_cast<W25QUnifiedSimpleFS::WriteMode>(m));
    };
    out.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsFlash.writeFileInPlace(n, d, s, a);
    };
    out.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsFlash.readFile(n, b, sz);
    };
    out.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsFlash.getFileInfo(n, a, s, c);
    };
  } else if (b == StorageBackend::NAND) {
    out.mount = [](bool autoFmt) {
      return fsNAND.mount(autoFmt);
    };
    out.exists = [](const char* n) {
      return fsNAND.exists(n);
    };
    out.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsNAND.createFileSlot(n, r, d, s);
    };
    out.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsNAND.writeFile(n, d, s, static_cast<MX35UnifiedSimpleFS::WriteMode>(m));
    };
    out.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsNAND.writeFileInPlace(n, d, s, a);
    };
    out.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsNAND.readFile(n, b, sz);
    };
    out.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsNAND.getFileInfo(n, a, s, c);
    };
  } else {
    out.mount = [](bool autoFmt) {
      (void)autoFmt;
      return fsPSRAM.mount(false);
    };
    out.exists = [](const char* n) {
      return fsPSRAM.exists(n);
    };
    out.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsPSRAM.createFileSlot(n, r, d, s);
    };
    out.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsPSRAM.writeFile(n, d, s, m);
    };
    out.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsPSRAM.writeFileInPlace(n, d, s, a);
    };
    out.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsPSRAM.readFile(n, b, sz);
    };
    out.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsPSRAM.getFileInfo(n, a, s, c);
    };
  }
}

static bool parseBackendSpec(const char* spec, StorageBackend& outBackend, const char*& outPath) {
  if (!spec) return false;
  const char* colon = strchr(spec, ':');
  if (!colon) return false;
  size_t nameLen = (size_t)(colon - spec);
  if (nameLen == 0) return false;
  if (strncmp(spec, "flash", nameLen) == 0) outBackend = StorageBackend::Flash;
  else if (strncmp(spec, "psram", nameLen) == 0) outBackend = StorageBackend::PSRAM_BACKEND;
  else if (strncmp(spec, "nand", nameLen) == 0) outBackend = StorageBackend::NAND;
  else return false;
  outPath = colon + 1;  // may start with '/' or not
  return true;
}

static bool normalizeFsPathCopy(char* dst, size_t dstCap, const char* input, bool wantTrailingSlash) {
  if (!dst || !input) return false;
  // Copy without leading '/'
  const char* s = input;
  while (*s == '/') ++s;
  size_t L = strlen(s);
  if (L >= dstCap) return false;
  memcpy(dst, s, L + 1);
  normalizePathInPlace(dst, wantTrailingSlash);
  if (strlen(dst) > ActiveFS::MAX_NAME) return false;
  return true;
}

static bool cmdFsCpImpl(const char* srcSpec, const char* dstSpec, bool force) {
  StorageBackend sbSrc, sbDst;
  const char* srcPathIn = nullptr;
  const char* dstPathIn = nullptr;
  if (!parseBackendSpec(srcSpec, sbSrc, srcPathIn) || !parseBackendSpec(dstSpec, sbDst, dstPathIn)) {
    Console.println("fscp: invalid backend spec; use flash:/path psram:/path nand:/path");
    return false;
  }
  // Prepare FS interfaces and mount (idempotent)
  FSIface srcFS{}, dstFS{};
  fillFsIface(sbSrc, srcFS);
  fillFsIface(sbDst, dstFS);
  // Mount: auto-format when empty on Flash/NAND; PSRAM no auto-format
  bool srcMounted = srcFS.mount((sbSrc != StorageBackend::PSRAM_BACKEND));
  bool dstMounted = dstFS.mount((sbDst != StorageBackend::PSRAM_BACKEND));
  if (!srcMounted) {
    Console.println("fscp: source mount failed");
    return false;
  }
  if (!dstMounted) {
    Console.println("fscp: destination mount failed");
    return false;
  }
  // Normalize paths within 32-char limit
  char srcAbs[ActiveFS::MAX_NAME + 1];
  char dstArgRaw[ActiveFS::MAX_NAME + 1];
  if (!normalizeFsPathCopy(srcAbs, sizeof(srcAbs), srcPathIn, /*wantTrailingSlash*/ false)) {
    Console.println("fscp: source path too long (<=32)");
    return false;
  }
  // Determine if destination is a folder spec (trailing '/')
  size_t LdstIn = strlen(dstPathIn);
  bool dstAsFolder = (LdstIn > 0 && dstPathIn[LdstIn - 1] == '/');
  if (!normalizeFsPathCopy(dstArgRaw, sizeof(dstArgRaw), dstPathIn, dstAsFolder)) {
    Console.println("fscp: destination path too long (<=32)");
    return false;
  }
  // Source exists and info
  if (!srcFS.exists(srcAbs)) {
    Console.println("fscp: source not found");
    return false;
  }
  uint32_t sAddr = 0, sSize = 0, sCap = 0;
  if (!srcFS.getFileInfo(srcAbs, sAddr, sSize, sCap)) {
    Console.println("fscp: getFileInfo(source) failed");
    return false;
  }
  // Build destination absolute
  char dstAbs[ActiveFS::MAX_NAME + 1];
  if (dstAsFolder) {
    const char* base = lastSlash(srcAbs);
    if (!makePathSafe(dstAbs, sizeof(dstAbs), dstArgRaw, base)) {
      Console.println("fscp: resulting destination path too long");
      return false;
    }
  } else {
    // already normalized as file
    strncpy(dstAbs, dstArgRaw, sizeof(dstAbs));
    dstAbs[sizeof(dstAbs) - 1] = 0;
  }
  if (pathTooLongForOnDisk(dstAbs)) {
    Console.println("fscp: destination name too long for FS (would be truncated)");
    return false;
  }
  // Overwrite policy
  if (dstFS.exists(dstAbs) && !force) {
    Console.println("fscp: destination exists (use -f to overwrite)");
    return false;
  }
  // Read source fully (consistent with local cp)
  uint8_t* buf = (uint8_t*)malloc(sSize ? sSize : 1);
  if (!buf) {
    Console.println("fscp: malloc failed");
    return false;
  }
  uint32_t got = srcFS.readFile(srcAbs, buf, sSize);
  if (got != sSize) {
    Console.println("fscp: read failed");
    free(buf);
    return false;
  }
  // Reserve/erase alignment based on destination backend
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
    Console.println("fscp: write failed");
    return false;
  }
  Console.println("fscp: ok");
  return true;
}