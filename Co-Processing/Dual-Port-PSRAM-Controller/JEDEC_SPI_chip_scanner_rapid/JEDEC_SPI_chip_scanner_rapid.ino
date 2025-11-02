// RP2040 master — continuous JEDEC ID rescan on SPI1
// Pins:
//   CS  = GP9
//   SCK = GP10
//   MOSI(TX) = GP11
//   MISO(RX) = GP12
//
// SPI mode: 0 (CPOL=0, CPHA=0), MSB first
// Default speed: 1 kHz (safe for the bit-banged slave on the bridge)

#include <Arduino.h>
#include <SPI.h>

static const uint8_t PIN_CS = 9;     // GP9
static const uint8_t PIN_SCK = 10;   // GP10
static const uint8_t PIN_MOSI = 11;  // GP11
static const uint8_t PIN_MISO = 12;  // GP12
static const uint8_t PIN_LED = 2;    // On-board LED

// Adjust if you want faster scans
#ifndef MASTER_SPI_HZ
#define MASTER_SPI_HZ 50000
#endif

#ifndef SCAN_INTERVAL_MS
#define SCAN_INTERVAL_MS 0  // how often to scan/print
#endif

static SPISettings spiCfg(MASTER_SPI_HZ, MSBFIRST, SPI_MODE0);

static void spi1_begin_pins() {
  // Map SPI1 to your chosen pins and start it
  SPI1.setSCK(PIN_SCK);
  SPI1.setTX(PIN_MOSI);
  SPI1.setRX(PIN_MISO);
  SPI1.begin();  // initialize SPI1 with the mapped pins
}

static void jedec_read(uint8_t out[3]) {
  SPI1.beginTransaction(spiCfg);
  digitalWrite(PIN_CS, LOW);
  // Opcode 0x9F (Read JEDEC ID)
  SPI1.transfer(0x9F);
  // Read 3 bytes (Manufacturer, Memory Type, Capacity)
  out[0] = SPI1.transfer(0x00);
  out[1] = SPI1.transfer(0x00);
  out[2] = SPI1.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI1.endTransaction();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  while (!Serial) delay(1000);
  delay(1000);
  Serial.println("\nRP2040 Master: continuous JEDEC rescan on SPI1");

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);  // idle high

  spi1_begin_pins();

  // Initial test
  uint8_t id[3] = { 0 };
  jedec_read(id);
  Serial.printf("Initial JEDEC ID: %02X %02X %02X @ %u Hz (SPI1)\n", id[0], id[1], id[2], (unsigned)MASTER_SPI_HZ);
}

void loop() {
  static uint32_t last = 0;
  static uint8_t lastId[3] = { 0xFF, 0xFF, 0xFF };
  if (millis() - last >= SCAN_INTERVAL_MS) {
    last = millis();

    uint8_t id[3] = { 0 };
    jedec_read(id);

    // Print every scan (or you can print only on change)
    Serial.printf("JEDEC ID: %02X %02X %02X\n", id[0], id[1], id[2]);

    // Blink LED to indicate activity
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));

    // Optional: show when it changes
    if (id[0] != lastId[0] || id[1] != lastId[1] || id[2] != lastId[2]) {
      Serial.printf("ID changed: %02X %02X %02X -> %02X %02X %02X\n",
                    lastId[0], lastId[1], lastId[2], id[0], id[1], id[2]);
      lastId[0] = id[0];
      lastId[1] = id[1];
      lastId[2] = id[2];
    }
  }

  // Yield to USB/other tasks
  delay(1);
}