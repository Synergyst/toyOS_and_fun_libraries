#pragma once
/*
  AudioWavOut.h
  Single-header helper for 8-bit mono WAV playback via MCP4921 DAC or PWM (piezo),
  plus a simple digital tone utility.

  Extracted/adapted from your co-processor sketch:
    - WAV parsing (unsigned 8-bit mono, PCM)
    - DAC playback (MCP4921) and PWMDAC style
    - PWM buzzer playback (analogWrite)
    - Optional performance test for MCP4921
    - Optional 'q' (Serial) stop and external cancel hook
    - Optional background service hook (called periodically)

  Dependencies you likely already use:
    - #include <Arduino.h>
    - #include <SPI.h>                 (if using DAC)
    - #include "MCP_DAC.h"             (Adafruit/MCP4921 library you already use)
    - Raspberry Pi Pico core exposes tight_loop_contents(); otherwise it's a no-op here.
*/

#include <Arduino.h>
#include <Print.h>

// If your build already includes MCP_DAC.h, this brings MCP4921.
// Remove this include if you want to provide your own DAC write callback only.
#include "MCP_DAC.h"

#ifndef AUDIOWAVOUT_CHUNK
#define AUDIOWAVOUT_CHUNK 256
#endif

class AudioWavOut {
public:
  // Minimal FS callbacks required by the player
  struct FS {
    bool (*exists)(const char* /*path*/) = nullptr;
    bool (*getFileSize)(const char* /*path*/, uint32_t& /*sizeOut*/) = nullptr;
    uint32_t (*readFileRange)(const char* /*path*/, uint32_t /*off*/, uint8_t* /*buf*/, uint32_t /*len*/) = nullptr;
  };

  // Optional hooks (all are optional)
  // Return true to cancel playback (in addition to 'q' from Serial if enabled)
  typedef bool (*CancelHook)();
  // Called occasionally during playback (pump background tasks, etc.)
  typedef void (*ServiceHook)();
  // If you don't use MCP4921, you can supply a raw 12-bit DAC write function
  typedef void (*DacWrite12Hook)(uint16_t);

  AudioWavOut()
    : _fs{}, _console(&Serial), _cancel(nullptr), _service(nullptr),
      _watchSerialForQ(true), _pwmDefaultPin(29), _pwmCarrierHz(62500),
      _mcp(nullptr), _dacWriteHook(nullptr) {}

  // Attach FS callbacks
  void attachFS(const FS& fs) {
    _fs = fs;
  }

  // Optional console (defaults to Serial)
  void setConsole(Print* p) {
    _console = p ? p : &Serial;
  }

  // Optional cancel and service hooks
  void setCancelHook(CancelHook h) {
    _cancel = h;
  }
  void setServiceHook(ServiceHook h) {
    _service = h;
  }

  // If true, pressing 'q' or 'Q' on USB-Serial stops playback (default: true)
  void setMonitorSerialForQ(bool en) {
    _watchSerialForQ = en;
  }

  // PWM setup (piezo). Call once in setup if you plan to use PWM playback.
  // defaultPin: your buzzer GPIO, default = GP3 (like in your sketch)
  // carrierHz:  typical value ~ 62.5 kHz
  // dutyRange:  analogWriteRange (255 typical for 8-bit)
  void beginPWM(uint8_t defaultPin = 29, uint32_t carrierHz = 62500, uint16_t dutyRange = 255) {
    _pwmDefaultPin = defaultPin;
    _pwmCarrierHz = carrierHz;
    analogWriteRange(dutyRange);
    analogWriteFreq(_pwmCarrierHz);
  }

  // Optional MCP4921 attachment (for DAC playback)
  // Call SPI.begin() and MCP->begin(CS) in your sketch. Then attach here.
  void attachMCP4921(MCP4921* m) {
    _mcp = m;
    _dacWriteHook = nullptr;
  }

  // If you don't use MCP4921, you can provide your own 12-bit write callback.
  void attachDacWriteHook(DacWrite12Hook fn) {
    _dacWriteHook = fn;
    _mcp = nullptr;
  }

  // DAC performance test (replicates co-proc 'performance_test()')
  void dacPerfTest() {
    if (!_console) _console = &Serial;
    if (!_mcp && !_dacWriteHook) {
      _console->println("DAC perf: no DAC attached");
      return;
    }
    uint32_t start = 0, stop = 0;
    (void)start;
    (void)stop;

    // If MCP present, test both write() and fastWriteA() like your code.
    if (_mcp) {
      volatile int sink = 0;
      _console->println();
      _console->println("dacPerfTest");
      start = micros();
      for (uint16_t value = 0; value < _mcp->maxValue(); value++) {
        sink = _mcp->write(value, 0);
      }
      stop = micros();
      _console->print(_mcp->maxValue());
      _console->print(" x MCP.write():\t");
      _console->print(stop - start);
      _console->print("\t");
      _console->println((stop - start) / (_mcp->maxValue() + 1.0));
      delay(10);

      start = micros();
      for (uint16_t value = 0; value < _mcp->maxValue(); value++) {
        _mcp->fastWriteA(value);
      }
      stop = micros();
      _console->print(_mcp->maxValue());
      _console->print(" x MCP.fastWriteA():\t");
      _console->print(stop - start);
      _console->print("\t");
      _console->println((stop - start) / (_mcp->maxValue() + 1.0));
      delay(10);
      // Reset DAC to 0 after test
      _mcp->fastWriteA(0);
    } else if (_dacWriteHook) {
      // Generic callback perf test
      const uint16_t maxVal = 4095;
      _console->println();
      _console->println("dacPerfTest (generic write)");
      start = micros();
      for (uint16_t v = 0; v <= maxVal; ++v) {
        _dacWriteHook(v);
      }
      stop = micros();
      _console->print(maxVal);
      _console->print(" x dacWrite12():\t");
      _console->print(stop - start);
      _console->print("\t");
      _console->println((stop - start) / (maxVal + 1.0));
      delay(10);
      _dacWriteHook(0);
    }
  }

  // Play unsigned 8-bit mono WAV via MCP4921 (or generic DAC write hook).
  bool playWavDAC(const char* path) {
    if (!checkFsReady() || !path || !*path) {
      logln("usage: wav play <file>");
      return false;
    }
    WavInfo wi{};
    if (!openAndParseWav(path, wi)) return false;

    logf("wav: %lu Hz, %u-bit, mono, data=%lu bytes (DAC)\n",
         (unsigned long)wi.sampleRate, (unsigned)wi.bitsPerSample, (unsigned long)wi.dataSize);
    logln("Press 'q' to stop");

    uint32_t q = 1000000u / wi.sampleRate;
    uint32_t r = 1000000u % wi.sampleRate;
    uint32_t acc = 0;
    uint8_t buf[AUDIOWAVOUT_CHUNK];
    uint32_t left = wi.dataSize;
    uint32_t off = wi.dataOffset;
    uint32_t nextT = micros();
    while (left > 0) {
      uint32_t n = (left > AUDIOWAVOUT_CHUNK) ? AUDIOWAVOUT_CHUNK : left;
      uint32_t got = _fs.readFileRange(path, off, buf, n);
      if (got != n) {
        logln("wav: read error");
        dacWrite12(0);
        return false;
      }
      off += n;
      left -= n;
      for (uint32_t i = 0; i < n; ++i) {
        if (shouldStop()) {
          dacWrite12(0);
          logln("\n(wav stopped)");
          return true;
        }
        // Map 8-bit 0..255 -> 12-bit 0..4095
        uint16_t v12 = ((uint16_t)buf[i]) << 4;
        dacWrite12(v12);

        waitUntil(nextT);
        nextT += q;
        acc += r;
        if (acc >= wi.sampleRate) {
          nextT += 1;
          acc -= wi.sampleRate;
        }
        if (_service) _service();
      }
    }
    dacWrite12(0);
    logln("\n(wav done)");
    return true;
  }

  // Play unsigned 8-bit mono WAV via PWM (piezo).
  // If pin < 0, uses the default set via beginPWM().
  bool playWavPWM(const char* path, int pin = -1) {
    if (!checkFsReady() || !path || !*path) {
      logln("usage: wav playpwm <file> [pin]");
      return false;
    }
    WavInfo wi{};
    if (!openAndParseWav(path, wi)) return false;

    uint8_t usePin = (pin >= 0 && pin <= 29) ? (uint8_t)pin : _pwmDefaultPin;
    pinMode(usePin, OUTPUT);
    analogWriteRange(255);
    analogWriteFreq(_pwmCarrierHz);
    analogWrite(usePin, 128);

    logf("wav: %lu Hz, %u-bit, mono, data=%lu bytes (PWM on GP%u)\n",
         (unsigned long)wi.sampleRate, (unsigned)wi.bitsPerSample, (unsigned long)wi.dataSize, (unsigned)usePin);
    logln("Press 'q' to stop");

    uint32_t q = 1000000u / wi.sampleRate;
    uint32_t r = 1000000u % wi.sampleRate;
    uint32_t acc = 0;
    uint8_t buf[AUDIOWAVOUT_CHUNK];
    uint32_t left = wi.dataSize;
    uint32_t off = wi.dataOffset;
    uint32_t nextT = micros();
    while (left > 0) {
      uint32_t n = (left > AUDIOWAVOUT_CHUNK) ? AUDIOWAVOUT_CHUNK : left;
      uint32_t got = _fs.readFileRange(path, off, buf, n);
      if (got != n) {
        logln("wav: read error");
        analogWrite(usePin, 0);
        return false;
      }
      off += n;
      left -= n;
      for (uint32_t i = 0; i < n; ++i) {
        if (shouldStop()) {
          analogWrite(usePin, 0);
          logln("\n(wav stopped)");
          return true;
        }
        // 8-bit 0..255 to duty
        analogWrite(usePin, buf[i]);

        waitUntil(nextT);
        nextT += q;
        acc += r;
        if (acc >= wi.sampleRate) {
          nextT += 1;
          acc -= wi.sampleRate;
        }
        if (_service) _service();
      }
    }
    analogWrite(usePin, 0);
    logln("\n(wav done)");
    return true;
  }

  // "PWMDAC" style to MCP4921: same scheduler as PWM, with fixed gain scaling.
  bool playWavPWMDAC(const char* path, int gainPercent = 50) {
    if (!checkFsReady() || !path || !*path) {
      logln("usage: wav playpwmdac <file>");
      return false;
    }
    WavInfo wi{};
    if (!openAndParseWav(path, wi)) return false;

    if (gainPercent < 0) gainPercent = 0;
    if (gainPercent > 400) gainPercent = 400;  // sane cap

    logf("wav: %lu Hz, %u-bit, mono, data=%lu bytes (PWMDAC gain=%d%%)\n",
         (unsigned long)wi.sampleRate, (unsigned)wi.bitsPerSample,
         (unsigned long)wi.dataSize, gainPercent);
    logln("Press 'q' to stop");

    uint32_t q = 1000000u / wi.sampleRate;
    uint32_t r = 1000000u % wi.sampleRate;
    uint32_t acc = 0;
    uint8_t buf[AUDIOWAVOUT_CHUNK];
    uint32_t left = wi.dataSize;
    uint32_t off = wi.dataOffset;
    uint32_t nextT = micros();
    while (left > 0) {
      uint32_t n = (left > AUDIOWAVOUT_CHUNK) ? AUDIOWAVOUT_CHUNK : left;
      uint32_t got = _fs.readFileRange(path, off, buf, n);
      if (got != n) {
        dacWrite12(0);
        logln("wav: read error");
        return false;
      }
      off += n;
      left -= n;
      for (uint32_t i = 0; i < n; ++i) {
        if (shouldStop()) {
          dacWrite12(0);
          logln("\n(wav stopped)");
          return true;
        }
        // Scale 8-bit sample to 12-bit with gainPercent
        uint32_t s = buf[i];  // 0..255
        uint32_t v = (s * 16u * (uint32_t)gainPercent + 50u) / 100u;
        if (v > 4095u) v = 4095u;
        dacWrite12((uint16_t)v);

        waitUntil(nextT);
        nextT += q;
        acc += r;
        if (acc >= wi.sampleRate) {
          nextT += 1;
          acc -= wi.sampleRate;
        }
        if (_service) _service();
      }
    }
    dacWrite12(0);
    logln("\n(wav done)");
    return true;
  }

  // Simple digital tone (toggle a pin), like your fn_tone
  // halfMicros = half period us, cycles = number of on/off toggles
  bool toneDigital(int pin, int halfMicros, int cycles) {
    if (pin < 0 || halfMicros <= 0 || cycles <= 0) return false;
    pinMode((uint8_t)pin, OUTPUT);
    for (int i = 0; i < cycles; ++i) {
      if (shouldStop()) break;
      digitalWrite((uint8_t)pin, HIGH);
      delayMicroseconds((uint32_t)halfMicros);
      digitalWrite((uint8_t)pin, LOW);
      delayMicroseconds((uint32_t)halfMicros);
      serviceOnce();
    }
    return true;
  }

private:
  struct WavInfo {
    uint32_t dataOffset = 0;
    uint32_t dataSize = 0;
    uint16_t channels = 0;
    uint16_t bitsPerSample = 0;
    uint32_t sampleRate = 0;
    uint16_t audioFormat = 0;  // 1 = PCM
  };

  // Little-endian helpers
  static inline uint16_t rd16le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }
  static inline uint32_t rd32le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

  bool openAndParseWav(const char* fn, WavInfo& wi) {
    if (!_fs.exists || !_fs.getFileSize || !_fs.readFileRange) {
      logln("wav: FS not attached");
      return false;
    }
    if (!_fs.exists(fn)) {
      logln("wav: file not found");
      return false;
    }
    uint32_t fsz = 0;
    if (!_fs.getFileSize(fn, fsz) || fsz < 44) {
      logln("wav: bad file size");
      return false;
    }
    // RIFF/WAVE header at 0..11
    uint8_t head[12];
    if (_fs.readFileRange(fn, 0, head, sizeof(head)) != sizeof(head)) {
      logln("wav: failed to read header");
      return false;
    }
    if (memcmp(head, "RIFF", 4) != 0 || memcmp(head + 8, "WAVE", 4) != 0) {
      logln("wav: not a RIFF/WAVE file");
      return false;
    }
    uint32_t off = 12;
    bool gotFmt = false, gotData = false;
    while (off + 8 <= fsz && !(gotFmt && gotData)) {
      uint8_t chdr[8];
      if (_fs.readFileRange(fn, off, chdr, 8) != 8) break;
      uint32_t cid = rd32le(chdr);
      uint32_t csz = rd32le(chdr + 4);
      char id0 = (char)(cid & 0xFF);
      char id1 = (char)((cid >> 8) & 0xFF);
      char id2 = (char)((cid >> 16) & 0xFF);
      char id3 = (char)((cid >> 24) & 0xFF);
      if (id0 == 'f' && id1 == 'm' && id2 == 't' && id3 == ' ') {
        uint8_t fmt[32];
        uint32_t toRead = csz > sizeof(fmt) ? sizeof(fmt) : csz;
        if (_fs.readFileRange(fn, off + 8, fmt, toRead) != toRead) {
          logln("wav: failed to read fmt");
          return false;
        }
        wi.audioFormat = rd16le(fmt + 0);
        wi.channels = rd16le(fmt + 2);
        wi.sampleRate = rd32le(fmt + 4);
        wi.bitsPerSample = rd16le(fmt + 14);
        gotFmt = true;
      } else if (id0 == 'd' && id1 == 'a' && id2 == 't' && id3 == 'a') {
        wi.dataOffset = off + 8;
        wi.dataSize = (off + 8 + csz <= fsz) ? csz : (fsz > off + 8 ? (fsz - (off + 8)) : 0);
        gotData = true;
      }
      uint32_t adv = 8 + csz;
      if (adv & 1) adv++;
      off += adv;
    }
    if (!gotFmt || !gotData) {
      logln("wav: missing fmt or data chunk");
      return false;
    }
    if (wi.audioFormat != 1) {
      logln("wav: only PCM supported");
      return false;
    }
    if (wi.channels != 1) {
      logln("wav: only mono supported");
      return false;
    }
    if (wi.bitsPerSample != 8) {
      logln("wav: only 8-bit samples supported");
      return false;
    }
    if (wi.sampleRate < 2000 || wi.sampleRate > 96000) {
      logln("wav: unreasonable sample rate");
      return false;
    }
    return true;
  }

  // DAC write wrapper (MCP or user hook)
  void dacWrite12(uint16_t v) {
    if (_mcp) {
      _mcp->fastWriteA(v);
    } else if (_dacWriteHook) {
      _dacWriteHook(v);
    } else {
      // No DAC configured; safely ignore
    }
  }

  // Wait until target micros timestamp (like your busy wait)
  void waitUntil(uint32_t target) {
    for (;;) {
      int32_t d = (int32_t)(micros() - target);
      if (d >= 0) break;
      tightLoop();
      yield();
    }
  }

  // Determine if playback should stop
  bool shouldStop() {
    if (_watchSerialForQ && Serial.available()) {
      int c = Serial.peek();
      if (c == 'q' || c == 'Q') {
        (void)Serial.read();
        return true;
      }
    }
    if (_cancel && _cancel()) return true;
    return false;
  }

  void serviceOnce() {
    if (_service) _service();
    tightLoop();
    yield();
  }

  void log(const char* s) {
    if (_console) _console->print(s);
  }
  void logln(const char* s) {
    if (_console) _console->println(s);
  }
  void logf(const char* fmt, ...) {
    if (!_console) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    _console->print(buf);
  }

  bool checkFsReady() const {
    return _fs.exists && _fs.getFileSize && _fs.readFileRange;
  }

  static inline void tightLoop() {
#if defined(ARDUINO_ARCH_RP2040)
    tight_loop_contents();
#else
    // No-op on other cores
#endif
  }

private:
  FS _fs;
  Print* _console;
  CancelHook _cancel;
  ServiceHook _service;
  bool _watchSerialForQ;

  // PWM settings
  uint8_t _pwmDefaultPin;
  uint32_t _pwmCarrierHz;

  // DAC backends
  MCP4921* _mcp;
  DacWrite12Hook _dacWriteHook;
};

/* --------------------------
   Commented example code (from your sketch) showing how you used MCP4921:
   (Leave disabled; for reference/porting)
   -----------------------------------------
   // SPI.begin();
   // MCP4921 MCP;
   // MCP.begin(17);
   // Console.print("DAC CHANNELS:\t");
   // Console.println(MCP.channels());
   // Console.print("DAC MAXVALUE:\t");
   // Console.println(MCP.maxValue());
   // performance_test();
   // MCP.fastWriteA(0);
*/