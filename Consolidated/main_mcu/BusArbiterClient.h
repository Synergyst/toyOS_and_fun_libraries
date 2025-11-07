// BusArbiterClient.h
#pragma once
#include <Arduino.h>

struct BusArbiterClient {
  static uint8_t pinReq;
  static uint8_t pinGrant;
  static bool reqActiveLow;
  static bool grantActiveHigh;
  static volatile uint16_t depth;

  static void begin(uint8_t reqPin, uint8_t grantPin,
                    bool reqIsActiveLow = true, bool grantIsActiveHigh = true) {
    pinReq = reqPin;
    pinGrant = grantPin;
    reqActiveLow = reqIsActiveLow;
    grantActiveHigh = grantIsActiveHigh;
    pinMode(pinReq, OUTPUT);
    // Deassert REQ
    digitalWrite(pinReq, reqActiveLow ? HIGH : LOW);
    pinMode(pinGrant, INPUT_PULLUP);  // or INPUT depending on your wiring
    depth = 0;
  }

  static bool isGranted() {
    int v = digitalRead(pinGrant);
    return grantActiveHigh ? (v == HIGH) : (v == LOW);
  }

  static bool acquire(uint32_t timeoutMs = 1000) {
    if (++depth > 1) return true;  // nested
    // Assert REQ
    digitalWrite(pinReq, reqActiveLow ? LOW : HIGH);
    uint32_t t0 = millis();
    while (!isGranted()) {
      if (timeoutMs && (millis() - t0) > timeoutMs) {
        // Give up, deassert REQ, unwind depth
        digitalWrite(pinReq, reqActiveLow ? HIGH : LOW);
        depth = 0;
        return false;
      }
      yield();
    }
    return true;
  }

  static void release() {
    if (depth == 0) return;
    if (--depth == 0) {
      // Deassert REQ
      digitalWrite(pinReq, reqActiveLow ? HIGH : LOW);
    }
  }

  struct Guard {
    bool ok;
    explicit Guard(uint32_t timeoutMs = 1000)
      : ok(acquire(timeoutMs)) {}
    ~Guard() {
      if (ok) release();
    }
  };
};

// static storage
uint8_t BusArbiterClient::pinReq = 0xFF;
uint8_t BusArbiterClient::pinGrant = 0xFF;
bool BusArbiterClient::reqActiveLow = true;
bool BusArbiterClient::grantActiveHigh = true;
volatile uint16_t BusArbiterClient::depth = 0;