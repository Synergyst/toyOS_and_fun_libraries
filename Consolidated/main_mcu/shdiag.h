#pragma once
// shdiag.h - Stack and PSRAM diagnostics (header-only)

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

#if __has_include(<unistd.h>)
#include <unistd.h>
#else
extern "C" void* sbrk(int);
#endif

// Pull in UnifiedSPIMem types (Manager, PSRAMUnifiedSimpleFS, etc.)
#include "UnifiedSPIMemSimpleFS.h"

#ifdef ARDUINO_ARCH_RP2040
extern "C" {
  extern char __StackTop;
  extern char __StackBottom;
}
#endif

namespace shdiag {

class Diag {
public:
  // Returns an approximate free stack for the current core.
  // RP2040: measured from __StackBottom to current SP (if in range).
  // Others: distance between heap end and stack pointer (approx).
  static inline uint32_t freeStackCurrentCoreApprox() {
#ifdef ARDUINO_ARCH_RP2040
    volatile uint8_t marker;
    uintptr_t sp = (uintptr_t)&marker;
    uintptr_t top = (uintptr_t)&__StackTop;
    uintptr_t bottom = (uintptr_t)&__StackBottom;
    if (top > bottom && sp >= bottom && sp <= top) {
      return (uint32_t)(sp - bottom);
    }
    void* heapEnd = sbrk(0);
    intptr_t gap = (intptr_t)sp - (intptr_t)heapEnd;
    return (gap > 0) ? (uint32_t)gap : 0u;
#else
    volatile uint8_t marker;
    void* heapEnd = sbrk(0);
    intptr_t gap = (intptr_t)&marker - (intptr_t)heapEnd;
    return (gap > 0) ? (uint32_t)gap : 0u;
#endif
  }

  // Prints stack info (total/used/free) on RP2040, otherwise prints approx free stack.
  static inline void printStackInfo(Print& out) {
#ifdef ARDUINO_ARCH_RP2040
    volatile uint8_t marker;
    uintptr_t sp = (uintptr_t)&marker;
    uintptr_t top = (uintptr_t)&__StackTop;
    uintptr_t bottom = (uintptr_t)&__StackBottom;
    if (top > bottom && sp >= bottom && sp <= top) {
      uint32_t total = (uint32_t)(top - bottom);
      uint32_t used = (uint32_t)(top - sp);
      uint32_t freeB = (uint32_t)(sp - bottom);
      out.print("Stack total=");
      out.print(total);
      out.print(" used=");
      out.print(used);
      out.print(" free=");
      out.print(freeB);
      out.println(" bytes");
      return;
    }
    out.print("Free Stack (core0 approx): ");
    out.print(freeStackCurrentCoreApprox());
    out.println(" bytes");
#else
    out.print("Free Stack (core0 approx): ");
    out.print(freeStackCurrentCoreApprox());
    out.println(" bytes");
#endif
  }

  // PSRAM capacity report via UnifiedSpiMem::Manager
  static inline void psramPrintCapacityReport(UnifiedSpiMem::Manager& mgr, Print& out = Serial) {
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

  // Non-destructive PSRAM write/read/verify and restore
  static inline void psramSafeSmokeTest(PSRAMUnifiedSimpleFS& fs, Print& out = Serial) {
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
    testAddr = (testAddr + 0xFF) & ~0xFFull;  // align
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
};

}  // namespace shdiag