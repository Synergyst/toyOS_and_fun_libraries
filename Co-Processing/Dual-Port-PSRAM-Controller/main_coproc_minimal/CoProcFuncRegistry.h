// CoprocFuncRegistry.h (on the co-processor firmware)
#include <Arduino.h>

typedef int32_t (*RpcHandler)(const int32_t* argv, uint32_t argc);
struct RpcEntry {
  const char* name;
  RpcHandler fn;
};

// Simple wrappers (dangerous on purpose; minimal checks)
static int32_t r_digitalRead(const int32_t* a, uint32_t n) {
  if (n < 1) return -22;  // EINVAL
  return (int32_t)digitalRead((uint8_t)a[0]);
}
static int32_t r_digitalWrite(const int32_t* a, uint32_t n) {
  if (n < 2) return -22;
  digitalWrite((uint8_t)a[0], (uint8_t)a[1]);
  return 0;
}
static int32_t r_pinMode(const int32_t* a, uint32_t n) {
  if (n < 2) return -22;
  pinMode((uint8_t)a[0], (uint8_t)a[1]);
  return 0;
}
static int32_t r_analogRead(const int32_t* a, uint32_t n) {
  if (n < 1) return -22;
  return (int32_t)analogRead((uint8_t)a[0]);
}
static int32_t r_delay(const int32_t* a, uint32_t n) {
  if (n < 1) return -22;
  delay((uint32_t)a[0]);
  return 0;
}

// Optional: truly dangerous example
// static int32_t r_reboot(const int32_t*, uint32_t) { NVIC_SystemReset(); return 0; }

static const RpcEntry g_table[] = {
  { "digitalRead", r_digitalRead },
  { "digitalWrite", r_digitalWrite },
  { "pinMode", r_pinMode },
  { "analogRead", r_analogRead },
  { "delay", r_delay },
  // { "reboot",       r_reboot       },
};

static bool dispatchFuncByName(const char* name, const int32_t* argv, uint32_t argc, int32_t& out) {
  for (size_t i = 0; i < sizeof(g_table) / sizeof(g_table[0]); ++i) {
    if (strcmp(g_table[i].name, name) == 0) {
      out = g_table[i].fn(argv, argc);
      return true;
    }
  }
  return false;
}