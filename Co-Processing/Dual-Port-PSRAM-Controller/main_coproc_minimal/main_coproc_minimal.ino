/*
  main_coproc_softserial.ino
  RP2350 / RP2040 Co-Processor (software-serial "bitbanged UART") — PoC transport + blob executor + script interpreter
  --------------------------------------------------------------------------------------------------------------------
  - Software serial (SoftwareSerial) transport: 8N1 framing over two GPIOs.
  - Core0: transport + protocol using a framed request/response over TX/RX pins.
  - Core1: blob executor (setup1/loop1 pattern).
  - Adds CoProcLang script pipeline (SCRIPT_BEGIN/DATA/END/EXEC) alongside binary blob pipeline.

  SPI NOR emulation notes:
    - SPI slave emulation on GP12..GP15 without touching the real flash during master sessions.
    - Emulated commands:
        • 0x9F: JEDEC (returns cached ID — same timing as your known-good sketch)
        • 0x05: Read Status-1 (returns emulated SR1 continuously)
        • 0x06: Write Enable (sets WEL in emulated SR1)
        • 0x04: Write Disable (clears WEL in emulated SR1)
        • 0x03: Read (consumes 24-bit address, then streams 0xFF)
        • 0x02: Page Program (consumes 24-bit address and data until CS high; ignored, counted)
        • 0x20: Sector Erase 4K (consumes 24-bit address; ignored)
      All others stream 0xFF.
    - Non-blocking opcode logging: SPI handler never prints. It pushes to a ring buffer.
      Use 'oplog' console commands to drain/view logs. Optional auto-drain in loop1.
    - The real flash is probed once at boot for JEDEC (using bit-bang master on GP16..GP19).
*/

#include <Arduino.h>
#include "BusArbiterWithISP.h"
#include "PSRAMMulti.h"
#include "rp_selfupdate.h"
#include <SoftwareSerial.h>
#include "CoProcProto.h"
#include "CoProcLang.h"

// ===== PIO defines (left as-is; not used by the emulation) =====
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// Upstream (master) pins on GP12..GP15
static const uint PIN_BRIDGE_SCK = 12;          // SCK from master (input)
static const uint PIN_BRIDGE_CS = 13;           // CS# from master (input, active low)
static const uint PIN_BRIDGE_MOSI_MASTER = 14;  // master MOSI (input to Pico)
static const uint PIN_BRIDGE_MISO_MASTER = 15;  // Pico MISO (output to master)

// SPI NOR flash pins (Pico as master)
static const uint PIN_FLASH_CS = 16;    // CS#
static const uint PIN_FLASH_SCK = 17;   // SCK
static const uint PIN_FLASH_MOSI = 18;  // DI
static const uint PIN_FLASH_MISO = 19;  // DO

// ---- Software SPI (Mode 0) on GP16/17/18/19 for JEDEC probe ----
static inline void softspi_pin_modes_for_probe() {
  pinMode(PIN_FLASH_CS, OUTPUT);
  pinMode(PIN_FLASH_SCK, OUTPUT);
  // Temporarily move MOSI to SIO so digitalWrite works (bridge assigned it to PIO0)
  gpio_set_function(PIN_FLASH_MOSI, GPIO_FUNC_SIO);
  pinMode(PIN_FLASH_MOSI, OUTPUT);
  // MISO as input
  pinMode(PIN_FLASH_MISO, INPUT);
  digitalWrite(PIN_FLASH_CS, HIGH);  // idle
  digitalWrite(PIN_FLASH_SCK, LOW);  // CPOL=0 idle low
  digitalWrite(PIN_FLASH_MOSI, LOW);
}
static inline void softspi_release_after_probe() {
  // Return ownership: MOSI back to PIO0 so the bridge can drive it (left from previous setup)
  gpio_set_function(PIN_FLASH_MOSI, GPIO_FUNC_PIO0);
  // SCK/CS driven again by PIO1 mirror (set to inputs here; SM re-enabling will set function)
  pinMode(PIN_FLASH_CS, INPUT);
  pinMode(PIN_FLASH_SCK, INPUT);
  // MISO untouched (input)
}
static inline uint8_t softspi_txrx_byte(uint8_t tx, uint32_t half_us) {
  uint8_t rx = 0;
  for (int i = 7; i >= 0; --i) {
    digitalWrite(PIN_FLASH_MOSI, (tx >> i) & 1);  // set up data
    if (half_us) delayMicroseconds(half_us);
    digitalWrite(PIN_FLASH_SCK, HIGH);  // rising edge: sample MISO
    rx = (uint8_t)((rx << 1) | (uint8_t)digitalRead(PIN_FLASH_MISO));
    if (half_us) delayMicroseconds(half_us);
    digitalWrite(PIN_FLASH_SCK, LOW);  // falling edge
  }
  return rx;
}
static bool flash_read_jedec(uint8_t outId[3], uint32_t half_us = 5 /* ~100 kHz */) {
  softspi_pin_modes_for_probe();
  if (half_us) delayMicroseconds(5);  // settle
  digitalWrite(PIN_FLASH_CS, LOW);
  if (half_us) delayMicroseconds(2);
  (void)softspi_txrx_byte(0x9F, half_us);
  outId[0] = softspi_txrx_byte(0x00, half_us);
  outId[1] = softspi_txrx_byte(0x00, half_us);
  outId[2] = softspi_txrx_byte(0x00, half_us);
  if (half_us) delayMicroseconds(1);
  digitalWrite(PIN_FLASH_CS, HIGH);
  softspi_release_after_probe();
  return true;
}

// ---------- SPI slave emulation on GP12..GP15 ----------
static uint8_t g_jedec_id[3] = { 0xFF, 0xFF, 0xFF };  // cached from flash at setup
static uint8_t g_sr1 = 0x00;                          // emulated Status Register-1 (bit0=WIP, bit1=WEL)

// Drive MISO bit quickly
static inline void bridgeMISOSet(int v) {
  gpio_put(PIN_BRIDGE_MISO_MASTER, v ? 1 : 0);
}

// ---- Debug helpers and state ----
static volatile uint8_t g_debug_last_opcode = 0;
static volatile uint32_t g_debug_last_addr = 0;
static volatile uint32_t g_debug_last_len = 0;
static volatile uint32_t g_op_counts[256] = { 0 };

// ---------- Non-blocking opcode log ----------
static const uint16_t OPLOG_CAP = 128;
struct OpLogEvt {
  uint32_t t_us;
  uint8_t op;
  uint32_t addr;  // 24-bit meaningful on 0x03/0x02/0x20
  uint32_t len;   // meaningful for 0x02 (bytes written)
};
static volatile uint16_t g_oplog_head = 0;
static volatile uint16_t g_oplog_tail = 0;
static OpLogEvt g_oplog_buf[OPLOG_CAP];
static volatile uint32_t g_oplog_drops = 0;
static volatile uint8_t g_oplog_auto = 1;  // auto-drain in loop1
static volatile uint8_t g_oplog_rate = 8;  // max entries drained per loop1 iteration

static inline void compiler_barrier() {
  __asm__ __volatile__("" ::
                         : "memory");
}

static inline void oplogPush(uint8_t op, uint32_t addr, uint32_t len) {
  uint16_t h = g_oplog_head;
  uint16_t n = (uint16_t)(h + 1);
  if (n >= OPLOG_CAP) n = 0;
  if (n == g_oplog_tail) {
    g_oplog_drops++;
    return;
  }  // full
  g_oplog_buf[h].t_us = micros();
  g_oplog_buf[h].op = op;
  g_oplog_buf[h].addr = addr & 0xFFFFFFu;
  g_oplog_buf[h].len = len;
  compiler_barrier();
  g_oplog_head = n;
}
static inline const char* opName(uint8_t op) {
  switch (op) {
    case 0x9F: return "RDID";
    case 0x05: return "RDSR1";
    case 0x06: return "WREN";
    case 0x04: return "WRDI";
    case 0x03: return "READ";
    case 0x02: return "PP";
    case 0x20: return "SE4K";
    default: return "";
  }
}
static void oplogDrain(uint16_t limit = 0) {
  uint16_t drained = 0;
  for (;;) {
    noInterrupts();
    uint16_t t = g_oplog_tail;
    uint16_t h = g_oplog_head;
    if (t == h) {
      interrupts();
      break;
    }  // empty
    OpLogEvt e = g_oplog_buf[t];
    g_oplog_tail = (uint16_t)((t + 1) >= OPLOG_CAP ? 0 : (t + 1));
    interrupts();

    if (e.op == 0x02) {
      Serial.printf("[OP] %10luus 0x%02X %-5s addr=0x%06lX len=%lu\n",
                    (unsigned long)e.t_us, e.op, opName(e.op),
                    (unsigned long)e.addr, (unsigned long)e.len);
    } else if (e.op == 0x03 || e.op == 0x20) {
      Serial.printf("[OP] %10luus 0x%02X %-5s addr=0x%06lX\n",
                    (unsigned long)e.t_us, e.op, opName(e.op),
                    (unsigned long)e.addr);
    } else {
      Serial.printf("[OP] %10luus 0x%02X %-5s\n",
                    (unsigned long)e.t_us, e.op, opName(e.op));
    }

    if (limit && (++drained >= limit)) break;
  }
}

// Reuse soft bit-bang pins to perform 0x03 reads from the real flash outside sessions
static inline bool flash_read_03_to_buf(uint32_t addr, uint8_t* out, size_t len, uint32_t half_us = 2) {
  if (!out || len == 0) return true;
  softspi_pin_modes_for_probe();
  if (half_us) delayMicroseconds(2);
  digitalWrite(PIN_FLASH_CS, LOW);
  if (half_us) delayMicroseconds(1);
  (void)softspi_txrx_byte(0x03, half_us);
  (void)softspi_txrx_byte((uint8_t)(addr >> 16), half_us);
  (void)softspi_txrx_byte((uint8_t)(addr >> 8), half_us);
  (void)softspi_txrx_byte((uint8_t)addr, half_us);
  for (size_t i = 0; i < len; ++i) {
    out[i] = softspi_txrx_byte(0x00, half_us);
  }
  if (half_us) delayMicroseconds(1);
  digitalWrite(PIN_FLASH_CS, HIGH);
  softspi_release_after_probe();
  return true;
}
static inline void hexdump(const uint8_t* buf, size_t len, uint32_t base = 0, uint8_t cols = 16) {
  if (!buf || len == 0) return;
  if (cols == 0) cols = 16;
  for (size_t off = 0; off < len; off += cols) {
    uint8_t n = (uint8_t)((len - off) < cols ? (len - off) : cols);
    Serial.printf("%08lX  ", (unsigned long)(base + off));
    for (uint8_t i = 0; i < cols; ++i) {
      if (i < n) Serial.printf("%02X ", buf[off + i]);
      else Serial.print("   ");
      if (i == 7) Serial.print(" ");
    }
    Serial.print(" |");
    for (uint8_t i = 0; i < n; ++i) {
      char c = (char)buf[off + i];
      Serial.print((c >= 32 && c < 127) ? c : '.');
    }
    Serial.println("|");
  }
}

// Returns immediately if CS# is deasserted quickly.
static void handleSpiSlaveSession_emu_subset() {
  if (gpio_get(PIN_BRIDGE_CS)) return;

  // Take control of MISO and idle it high; tri-state when CS# deasserts
  pinMode(PIN_BRIDGE_MISO_MASTER, OUTPUT);
  digitalWrite(PIN_BRIDGE_MISO_MASTER, HIGH);

  auto cs_is_high = []() -> bool {
    return gpio_get(PIN_BRIDGE_CS) != 0;
  };
  auto sck_is_high = []() -> bool {
    return gpio_get(PIN_BRIDGE_SCK) != 0;
  };
  auto sck_is_low = []() -> bool {
    return gpio_get(PIN_BRIDGE_SCK) == 0;
  };
  auto mosi_bit = []() -> int {
    return (gpio_get(PIN_BRIDGE_MOSI_MASTER) & 1);
  };

  // Ensure we start aligned on a low SCK (Mode 0 idle low)
  while (!cs_is_high() && !sck_is_low()) {}
  if (cs_is_high()) {
    pinMode(PIN_BRIDGE_MISO_MASTER, INPUT);
    return;
  }

  // 1) Receive the opcode (8 bits, sample on rising edge)
  uint8_t opcode = 0;
  for (int i = 0; i < 8; ++i) {
    while (!cs_is_high() && sck_is_low()) {}
    if (cs_is_high()) {
      pinMode(PIN_BRIDGE_MISO_MASTER, INPUT);
      return;
    }
    opcode = (uint8_t)((opcode << 1) | (uint8_t)mosi_bit());
    while (!cs_is_high() && sck_is_high()) {}
    if (cs_is_high()) {
      pinMode(PIN_BRIDGE_MISO_MASTER, INPUT);
      return;
    }
  }

  // Track opcode stats
  g_debug_last_opcode = opcode;
  g_op_counts[opcode]++;

  // Minimal state machine: same 0x9F timing as your working code, add a few opcodes
  enum Phase : uint8_t { PH_ADDR = 0,
                         PH_DATA_OUT,
                         PH_DATA_IN,
                         PH_IDLE };
  Phase phase = PH_IDLE;
  int addr_bytes_left = 0;

  // TX state
  int tx_bit = 7;
  uint8_t tx_byte = 0xFF;
  int jedec_idx = 0;

  // Optional address/data counters for debug
  uint32_t cur_addr = 0;
  uint32_t wr_count = 0;

  // Prepare phase by opcode (no Serial here)
  switch (opcode) {
    case 0x9F:  // JEDEC (unchanged behavior)
      phase = PH_DATA_OUT;
      tx_byte = g_jedec_id[0];
      jedec_idx = 0;
      oplogPush(opcode, 0, 0);
      break;
    case 0x05:  // RDSR1
      phase = PH_DATA_OUT;
      tx_byte = g_sr1;
      oplogPush(opcode, 0, 0);
      break;
    case 0x06:  // WREN
      g_sr1 |= 0x02;
      phase = PH_IDLE;
      oplogPush(opcode, 0, 0);
      break;
    case 0x04:  // WRDI
      g_sr1 &= ~0x02;
      phase = PH_IDLE;
      oplogPush(opcode, 0, 0);
      break;
    case 0x03:  // READ (24-bit address, then data)
      phase = PH_ADDR;
      addr_bytes_left = 3;
      cur_addr = 0;
      break;
    case 0x02:  // PAGE PROGRAM (24-bit address, then data in until CS# high)
      phase = PH_ADDR;
      addr_bytes_left = 3;
      cur_addr = 0;
      wr_count = 0;
      break;
    case 0x20:  // SECTOR ERASE (24-bit address)
      phase = PH_ADDR;
      addr_bytes_left = 3;
      cur_addr = 0;
      break;
    default:
      phase = PH_IDLE;
      oplogPush(opcode, 0, 0);
      break;
  }

  // 2) Continue while CS# low
  // Local accumulators for PH_ADDR and PH_DATA_IN
  uint8_t rx_bits = 0, rx_byte = 0;

  while (!cs_is_high()) {
    // Ensure SCK is low before presenting next bit (Mode 0)
    while (!cs_is_high() && !sck_is_low()) {}
    if (cs_is_high()) break;

    // Present the bit before the rising edge
    int bit_to_send = 0x01;  // default idle high
    if (phase == PH_DATA_OUT) bit_to_send = (tx_byte >> tx_bit) & 1;
    digitalWrite(PIN_BRIDGE_MISO_MASTER, bit_to_send);

    // Rising edge (master samples MISO; we sample MOSI)
    while (!cs_is_high() && sck_is_low()) {}
    if (cs_is_high()) break;
    int mbit = mosi_bit();

    // Falling edge completes the bit
    while (!cs_is_high() && sck_is_high()) {}
    if (cs_is_high()) break;

    // Advance TX bit if we are actively outputting a byte
    if (phase == PH_DATA_OUT) {
      tx_bit--;
      if (tx_bit < 0) {
        tx_bit = 7;
        // Load next TX byte
        if (opcode == 0x9F) {
          ++jedec_idx;
          tx_byte = (jedec_idx < 3) ? g_jedec_id[jedec_idx] : 0xFF;
        } else if (opcode == 0x05) {
          tx_byte = g_sr1;  // repeat SR1 forever
        } else if (opcode == 0x03) {
          tx_byte = 0xFF;  // read data — we return 0xFF (no direct flash access)
          cur_addr++;
        } else {
          tx_byte = 0xFF;
        }
      }
    }

    // Handle address/data phases for opcodes that need it
    if (phase == PH_ADDR) {
      rx_byte = (uint8_t)((rx_byte << 1) | (uint8_t)mbit);
      rx_bits++;
      if (rx_bits == 8) {
        // Assemble address MSB..LSB across 3 bytes
        cur_addr = (cur_addr << 8) | rx_byte;
        rx_bits = 0;
        rx_byte = 0;
        addr_bytes_left--;
        if (addr_bytes_left <= 0) {
          g_debug_last_addr = cur_addr & 0xFFFFFFu;
          if (opcode == 0x03) {
            phase = PH_DATA_OUT;  // stream 0xFF
            tx_byte = 0xFF;
            tx_bit = 7;
            oplogPush(opcode, cur_addr, 0);
          } else if (opcode == 0x02) {
            phase = PH_DATA_IN;  // len unknown until CS# high
          } else if (opcode == 0x20) {
            phase = PH_IDLE;
            oplogPush(opcode, cur_addr, 0);
          } else {
            phase = PH_IDLE;
          }
        }
      }
    } else if (phase == PH_DATA_IN) {
      // Consume incoming data (program), ignoring content. Stop on CS# high (loop exits).
      // Count bytes written (every 8 bits)
      rx_byte = (uint8_t)((rx_byte << 1) | (uint8_t)mbit);
      rx_bits++;
      if (rx_bits == 8) {
        rx_bits = 0;
        rx_byte = 0;
        wr_count++;
      }
    } else {
      // PH_IDLE or PH_DATA_OUT handled above
    }
  }

  // Record last data length for debug (for Page Program) and finalize log
  if (opcode == 0x02) {
    g_debug_last_len = wr_count;
    oplogPush(opcode, cur_addr, wr_count);
  }

  // High-Z when not selected
  pinMode(PIN_BRIDGE_MISO_MASTER, INPUT);
}

// Keep ISR minimal; it just runs the session and returns.
static void onBridgeCSFalling() {
  handleSpiSlaveSession_emu_subset();
}

// ---- USB Serial console ----
static void usbConsolePoll() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length() > 0) {
        if (line.startsWith("flashid")) {
          uint8_t id[3] = { 0 };
          bool ok = flash_read_jedec(id, 5 /* ~100 kHz */);
          if (ok) {
            Serial.printf("[FLASH] JEDEC: %02X %02X %02X\n", id[0], id[1], id[2]);
          } else {
            Serial.println("[FLASH] JEDEC read failed");
          }
        } else if (line.startsWith("dump")) {
          // dump <addr> <len> [cols]
          uint32_t addr = 0, len = 0, cols = 16;
          int n = sscanf(line.c_str(), "dump %lu %lu %lu", &addr, &len, &cols);
          if (n < 2) {
            Serial.println("Usage: dump <addr> <len> [cols]");
          } else {
            if (len > 4096) len = 4096;  // safety cap
            uint8_t* buf = (uint8_t*)malloc(len);
            if (!buf) {
              Serial.println("alloc fail");
            } else {
              bool ok = flash_read_03_to_buf(addr, buf, len, 2);
              if (!ok) Serial.println("dump: read failed");
              else hexdump(buf, len, addr, (uint8_t)((cols == 0 || cols > 32) ? 16 : cols));
              free(buf);
            }
          }
        } else if (line.startsWith("sr1")) {
          Serial.printf("SR1 emu: 0x%02X  [WIP=%u, WEL=%u]\n",
                        g_sr1, (g_sr1 & 0x01) ? 1 : 0, (g_sr1 & 0x02) ? 1 : 0);
        } else if (line.startsWith("setwel")) {
          int val = -1;
          if (sscanf(line.c_str(), "setwel %d", &val) == 1) {
            if (val) g_sr1 |= 0x02;
            else g_sr1 &= ~0x02;
            Serial.printf("WEL set to %d (SR1=0x%02X)\n", val ? 1 : 0, g_sr1);
          } else {
            Serial.println("Usage: setwel <0|1>");
          }
        } else if (line.startsWith("setbusy")) {
          int val = -1;
          if (sscanf(line.c_str(), "setbusy %d", &val) == 1) {
            if (val) g_sr1 |= 0x01;
            else g_sr1 &= ~0x01;
            Serial.printf("WIP/BUSY set to %d (SR1=0x%02X)\n", val ? 1 : 0, g_sr1);
          } else {
            Serial.println("Usage: setbusy <0|1>");
          }
        } else if (line.startsWith("setjedec")) {
          unsigned m = 0, t = 0, c = 0;
          if (sscanf(line.c_str(), "setjedec %x %x %x", &m, &t, &c) == 3) {
            g_jedec_id[0] = (uint8_t)m;
            g_jedec_id[1] = (uint8_t)t;
            g_jedec_id[2] = (uint8_t)c;
            Serial.printf("JEDEC emu set to: %02X %02X %02X\n", g_jedec_id[0], g_jedec_id[1], g_jedec_id[2]);
          } else {
            Serial.println("Usage: setjedec <mfr hex> <type hex> <cap hex>");
          }
        } else if (line.startsWith("oplog")) {
          // oplog [drain [N] | auto <0|1> | clear | rate <N> | stats]
          uint32_t n = 0, val = 0;
          if (line.indexOf("drain") >= 0) {
            if (sscanf(line.c_str(), "oplog drain %lu", &n) == 1) oplogDrain((uint16_t)n);
            else oplogDrain(0);
          } else if (sscanf(line.c_str(), "oplog auto %lu", &val) == 1) {
            g_oplog_auto = (val ? 1 : 0);
            Serial.printf("oplog auto=%u\n", g_oplog_auto);
          } else if (sscanf(line.c_str(), "oplog rate %lu", &val) == 1) {
            g_oplog_rate = (uint8_t)((val > 64) ? 64 : val);
            Serial.printf("oplog rate=%u per loop1\n", g_oplog_rate);
          } else if (line.indexOf("clear") >= 0) {
            noInterrupts();
            g_oplog_tail = g_oplog_head = 0;
            g_oplog_drops = 0;
            interrupts();
            Serial.println("oplog cleared");
          } else if (line.indexOf("stats") >= 0) {
            noInterrupts();
            uint16_t h = g_oplog_head, t = g_oplog_tail;
            interrupts();
            uint16_t used = (h >= t) ? (h - t) : (uint16_t)(OPLOG_CAP - (t - h));
            Serial.printf("oplog: used=%u/%u drops=%lu auto=%u rate=%u\n",
                          used, OPLOG_CAP, (unsigned long)g_oplog_drops, g_oplog_auto, g_oplog_rate);
          } else {
            Serial.println("Usage:");
            Serial.println("  oplog drain [N]   - print up to N entries (all if omitted)");
            Serial.println("  oplog auto <0|1>  - enable/disable auto-drain in loop1");
            Serial.println("  oplog rate <N>    - max entries drained per loop1 iteration");
            Serial.println("  oplog clear       - clear buffer and drop counter");
            Serial.println("  oplog stats       - show buffer usage/drops");
          }
        } else if (line.startsWith("opstats")) {
          bool doReset = (line.indexOf("reset") >= 0);
          if (doReset) {
            for (int i = 0; i < 256; ++i) g_op_counts[i] = 0;
            Serial.println("opstats: counters reset.");
          }
          Serial.printf("last: opcode=0x%02X addr=0x%06lX len=%lu\n",
                        g_debug_last_opcode, (unsigned long)(g_debug_last_addr & 0xFFFFFFu),
                        (unsigned long)g_debug_last_len);
          Serial.println("opcode counts (nonzero):");
          for (int i = 0; i < 256; ++i) {
            uint32_t v = g_op_counts[i];
            if (v) Serial.printf("  %02X : %lu\n", i, (unsigned long)v);
          }
        } else {
          Serial.println("Commands:");
          Serial.println("  flashid                  - read JEDEC ID using direct bit-bang");
          Serial.println("  dump <addr> <len> [cols] - hex-dump <len> bytes from real flash (0x03)");
          Serial.println("  sr1                      - show emulated SR1 (WIP/WEL)");
          Serial.println("  setwel <0|1>             - set/clear emulated WEL");
          Serial.println("  setbusy <0|1>            - set/clear emulated BUSY/WIP");
          Serial.println("  setjedec <mfr typ cap>   - override emulated JEDEC (hex)");
          Serial.println("  oplog ...                - non-blocking opcode log (see 'oplog' help)");
          Serial.println("  opstats [reset]          - opcode counters and last op");
        }
      }
      line = "";
    } else {
      if (line.length() < 120) line += c;
    }
  }
}

// ==================================================
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
static SoftwareSerial link(PIN_RX, PIN_TX, false);  // RX, TX, non-inverted

// ------- Mailbox / cancel (shared with blob + script) -------
#ifndef BLOB_MAILBOX_MAX
#define BLOB_MAILBOX_MAX 256
#endif
extern "C" __attribute__((aligned(4))) uint8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };
volatile uint8_t g_cancel_flag = 0;

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

// ------- Serial byte helpers -------
static bool readByte(uint8_t& b, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) <= timeoutMs) {
    int a = link.available();
    if (a > 0) {
      int v = link.read();
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
    size_t wrote = link.write(src + off, n - off);
    if (wrote > 0) {
      off += wrote;
      continue;
    }
    if ((millis() - start) > timeoutMs) return false;
    yield();
  }
  link.flush();
  return true;
}

// ------- ISP handlers (unchanged) -------
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
      usbConsolePoll();
      serviceISPIfActive();
      tight_loop_contents();
      yield();
    }
  }
}

// ------- Core0: serial protocol loop -------
static void protocolLoop() {
  for (;;) {
    usbConsolePoll();
    serviceISPIfActive();
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
    serviceISPIfActive();
    usbConsolePoll();
    if (BOOTSEL) {
      Serial.println("Rebooting now..");
      delay(1500);
      yield();
      watchdog_reboot(0, 0, 1500);
      while (true) tight_loop_contents();
    }
  }
}

// ------- rp_selfupdate (unchanged; note: GP12 overlap with bridge) -------
static const uint8_t PIN_MISO = 12;   // pin from RP to PSRAM pin-5
static const uint8_t PIN_MOSI = 11;   // pin from RP to PSRAM pin-2
static const uint8_t PIN_SCK = 10;    // pin from RP to PSRAM pin-6
static const uint8_t PIN_DEC_EN = 9;  // 74HC138 G2A/G2B tied together; active LOW
static const uint8_t PIN_DEC_A0 = 8;  // 74HC138 address bit 0
static const uint8_t PIN_DEC_A1 = 7;  // 74HC138 address bit 1
static const uint8_t BANKS = 4;
static const uint32_t PER_CHIP_BYTES = 8u * 1024u * 1024u;
PSRAMAggregateDevice psram(PIN_MISO, PIN_MOSI, PIN_SCK, PER_CHIP_BYTES);
PSRAMSimpleFS_Multi fs(psram, /*capacityBytes=*/PER_CHIP_BYTES* BANKS);
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

// ========== Setup and main ==========
void setup() {
  Serial.begin();
  delay(5000);
  if (!Serial) delay(1000);
  Serial.println("CoProc (soft-serial) booting...");

  // Initialize pins for bridge (master-facing SPI slave)
  pinMode(PIN_BRIDGE_SCK, INPUT);
  pinMode(PIN_BRIDGE_CS, INPUT);
  pinMode(PIN_BRIDGE_MOSI_MASTER, INPUT);
  pinMode(PIN_BRIDGE_MISO_MASTER, INPUT);  // idle high-Z, we drive it only when CS# low

  // Cache the JEDEC ID from the real flash (pins 16..19)
  {
    uint8_t id[3] = { 0 };
    if (flash_read_jedec(id, 5 /* ~100 kHz */)) {
      g_jedec_id[0] = id[0];
      g_jedec_id[1] = id[1];
      g_jedec_id[2] = id[2];
      Serial.printf("[FLASH] Cached JEDEC: %02X %02X %02X\n", g_jedec_id[0], g_jedec_id[1], g_jedec_id[2]);
    } else {
      Serial.println("[FLASH] JEDEC read failed; caching FFs");
      g_jedec_id[0] = g_jedec_id[1] = g_jedec_id[2] = 0xFF;
    }
  }

  // CS# falling edge starts a slave session (emulation subset)
  attachInterrupt(digitalPinToInterrupt(PIN_BRIDGE_CS), onBridgeCSFalling, FALLING);

  ArbiterISP::initTestPins();
  ArbiterISP::cleanupToResetState();

  // Allocate protocol buffers
  g_reqBuf = (uint8_t*)malloc(REQ_MAX);
  g_respBuf = (uint8_t*)malloc(RESP_MAX);

  // Initialize software serial link
  link.begin(SOFT_BAUD);
  link.listen();  // ensure active receiver

  // initialize executor
  g_exec.begin();
  // Register named functions for CMD_FUNC
  g_exec.registerFunc("ping", 0, fn_ping);
  g_exec.registerFunc("add", -1, fn_add);
  g_exec.registerFunc("tone", 3, fn_tone);

  Serial.printf("CoProc ready (soft-serial %u bps). RX=GP%u TX=GP%u\n", (unsigned)SOFT_BAUD, (unsigned)PIN_RX, (unsigned)PIN_TX);
  Serial.println("SPI slave emulation enabled on GP12..GP15: 9F,05,06,04,03,02,20 (no direct flash access).");
  Serial.println("Use 'oplog' to non-blockingly inspect master requests.");
}

void loop() {
  protocolLoop();  // never returns
}

// setup1/loop1 for core1 (Philhower)
void setup1() { /* nothing */
}
void loop1() {
  // Auto-drain a few log entries per tick to keep console responsive and SPI path non-blocking
  if (g_oplog_auto && Serial) {
    oplogDrain(g_oplog_rate);  // drain up to N lines per iteration
  }
  g_exec.workerPoll();
  tight_loop_contents();
}