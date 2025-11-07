// ================= Memory Diagnostics =================
#ifdef ARDUINO_ARCH_RP2040
extern "C" {
  extern char __StackTop;
  extern char __StackBottom;
}
#include "unistd.h"
static uint32_t freeStackCurrentCoreApprox() {
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
}
#else
static uint32_t freeStackCurrentCoreApprox() {
  volatile uint8_t marker;
  void* heapEnd = sbrk(0);
  intptr_t gap = (intptr_t)&marker - (intptr_t)heapEnd;
  return (gap > 0) ? (uint32_t)gap : 0u;
}
#endif
#ifdef ARDUINO_ARCH_RP2040
static void printStackInfo() {
  volatile uint8_t marker;
  uintptr_t sp = (uintptr_t)&marker;
  uintptr_t top = (uintptr_t)&__StackTop;
  uintptr_t bottom = (uintptr_t)&__StackBottom;
  if (top > bottom && sp >= bottom && sp <= top) {
    uint32_t total = (uint32_t)(top - bottom);
    uint32_t used = (uint32_t)(top - sp);
    uint32_t free = (uint32_t)(sp - bottom);
    Console.printf("Stack total=%u used=%u free=%u bytes\n", total, used, free);
    return;
  }
  Console.printf("Free Stack (core0 approx): %u bytes\n", freeStackCurrentCoreApprox());
}
#endif