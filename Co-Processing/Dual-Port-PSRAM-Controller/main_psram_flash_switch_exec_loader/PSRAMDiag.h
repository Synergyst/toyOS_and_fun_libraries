// ========== PSRAM diagnostics using UnifiedSPIMem ==========
static void psramPrintCapacityReport(UnifiedSpiMem::Manager& mgr, Stream& out = Serial) {
  size_t banks = 0;
  uint64_t total = 0;
  out.println();
  out.println("PSRAM capacity report (Unified):");
  for (size_t i = 0; i < mgr.detectedCount(); ++i) {
    const auto* di = mgr.detectedInfo(i);
    if (!di || di->type != UnifiedSpiMem::DeviceType::Psram) continue;
    banks++;
    total += di->capacityBytes;
    out.print("  Bank ");
    out.print(banks - 1);
    out.print(" (CS=");
    out.print(di->cs);
    out.print(")  Vendor=");
    out.print(di->vendorName);
    out.print("  Cap=");
    out.print((unsigned long)di->capacityBytes);
    out.print(" bytes");
    if (di->partHint) {
      out.print("  Part=");
      out.print(di->partHint);
    }
    out.print("  JEDEC:");
    for (uint8_t k = 0; k < di->jedecLen; ++k) {
      out.print(' ');
      if (di->jedec[k] < 16) out.print('0');
      out.print(di->jedec[k], HEX);
    }
    out.println();
  }
  out.print("Banks: ");
  out.println((unsigned)banks);
  out.print("Total: ");
  out.print((unsigned long)total);
  out.print(" bytes (");
  out.print((unsigned long)(total / (1024UL * 1024UL)));
  out.println(" MB)");
  out.println();
}
static void psramSafeSmokeTest(PSRAMUnifiedSimpleFS& fs, Stream& out = Serial) {
  auto* dev = fs.raw().device();
  if (!dev || dev->type() != UnifiedSpiMem::DeviceType::Psram) {
    out.println("PSRAM smoke test: PSRAM device not open");
    return;
  }
  const uint64_t cap = dev->capacity();
  if (cap < 1024) {
    out.println("PSRAM smoke test: capacity too small");
    return;
  }
  const uint32_t fsHead = fs.raw().nextDataAddr();
  const uint32_t dataStart = fs.raw().dataRegionStart();
  const uint32_t TEST_SIZE = 1024;
  uint64_t testAddr = fsHead + 4096;
  if (testAddr + TEST_SIZE > cap) {
    if (cap > TEST_SIZE) testAddr = cap - TEST_SIZE;
    else testAddr = 0;
  }
  testAddr = (testAddr + 0xFF) & ~0xFFull;
  if (testAddr < dataStart) testAddr = dataStart;
  out.print("PSRAM smoke test @ 0x");
  out.print((unsigned long)testAddr, HEX);
  out.print(" size=");
  out.print(TEST_SIZE);
  out.println(" bytes");
  uint8_t* original = (uint8_t*)malloc(TEST_SIZE);
  uint8_t* verify = (uint8_t*)malloc(TEST_SIZE);
  if (!original || !verify) {
    out.println("malloc failed for buffers");
    if (original) free(original);
    if (verify) free(verify);
    return;
  }
  if (dev->read(testAddr, original, TEST_SIZE) != TEST_SIZE) {
    out.println("read (backup) failed");
    free(original);
    free(verify);
    return;
  }
  memset(verify, 0xAA, TEST_SIZE);
  if (!dev->write(testAddr, verify, TEST_SIZE)) {
    out.println("write pattern 0xAA failed");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  memset(verify, 0x00, TEST_SIZE);
  if (dev->read(testAddr, verify, TEST_SIZE) != TEST_SIZE) {
    out.println("readback (0xAA) failed");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  bool okAA = true;
  for (uint32_t i = 0; i < TEST_SIZE; ++i) {
    if (verify[i] != 0xAA) {
      okAA = false;
      break;
    }
    if ((i & 63) == 0) yield();
  }
  out.println(okAA ? "Pattern 0xAA OK" : "Pattern 0xAA mismatch");
  for (uint32_t i = 0; i < TEST_SIZE; ++i) {
    verify[i] = (uint8_t)((testAddr + i) ^ 0x5A);
  }
  if (!dev->write(testAddr, verify, TEST_SIZE)) {
    out.println("write pattern addr^0x5A failed");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  uint8_t* rb = (uint8_t*)malloc(TEST_SIZE);
  if (!rb) {
    out.println("malloc failed for readback");
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  if (dev->read(testAddr, rb, TEST_SIZE) != TEST_SIZE) {
    out.println("readback (addr^0x5A) failed");
    free(rb);
    (void)dev->write(testAddr, original, TEST_SIZE);
    free(original);
    free(verify);
    return;
  }
  bool okAddr = true;
  for (uint32_t i = 0; i < TEST_SIZE; ++i) {
    uint8_t exp = (uint8_t)((testAddr + i) ^ 0x5A);
    if (rb[i] != exp) {
      okAddr = false;
      break;
    }
    if ((i & 63) == 0) yield();
  }
  out.println(okAddr ? "Pattern addr^0x5A OK" : "Pattern addr^0x5A mismatch");
  free(rb);
  if (!dev->write(testAddr, original, TEST_SIZE)) {
    out.println("restore failed (data left with test pattern)");
  } else {
    out.println("restore OK");
  }
  free(original);
  free(verify);
}