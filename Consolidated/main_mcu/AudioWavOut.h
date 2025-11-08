#pragma once
/*
  AudioWavOut.h (robust backoff reader)
  - Robust RIFF/WAV (PCM, mono, 8-bit) parser: tolerates extra chunks, clamps to file end if known
  - Hardened FS reading: automatic backoff (SL,128,64,32,16,8,4,2,1) if a read returns 0
  - MCP4921 DAC, generic 12-bit DAC hook, and PWM buzzer playback
  - 'q' to stop, cancel/service hooks, small stack

  Public API is unchanged:
    attachFS, setConsole, setCancelHook, setServiceHook, setMonitorSerialForQ
    beginPWM, attachMCP4921, attachDacWriteHook, dacPerfTest
    playWavDAC(path), playWavPWM(path, pin), playWavPWMDAC(path, gain%)
    toneDigital(pin, halfMicros, cycles)
*/

#include <Arduino.h>
#include <Print.h>
#include "MCP_DAC.h"

#ifndef AUDIOWAVOUT_CHUNK
#define AUDIOWAVOUT_CHUNK 512
#endif
#ifndef AUDIOWAVOUT_READ_SLICE
#define AUDIOWAVOUT_READ_SLICE 256
#endif

class AudioWavOut {
public:
  struct FS {
    bool (*exists)(const char* path) = nullptr;
    bool (*getFileSize)(const char* path, uint32_t& sizeOut) = nullptr;
    uint32_t (*readFileRange)(const char* path, uint32_t off, uint8_t* buf, uint32_t len) = nullptr;
  };
  typedef bool (*CancelHook)();
  typedef void (*ServiceHook)();
  typedef void (*DacWrite12Hook)(uint16_t);

  AudioWavOut()
    : _console(&Serial),
      _cancel(nullptr),
      _service(nullptr),
      _watchSerialForQ(true),
      _pwmDefaultPin(29),
      _pwmCarrierHz(62500),
      _mcp(nullptr),
      _dacWriteHook(nullptr) {
    _fs.exists = nullptr;
    _fs.getFileSize = nullptr;
    _fs.readFileRange = nullptr;
  }

  void attachFS(const FS& fs) {
    _fs = fs;
  }
  void setConsole(Print* p) {
    _console = p ? p : &Serial;
  }
  void setCancelHook(CancelHook h) {
    _cancel = h;
  }
  void setServiceHook(ServiceHook h) {
    _service = h;
  }
  void setMonitorSerialForQ(bool en) {
    _watchSerialForQ = en;
  }

  void beginPWM(uint8_t defaultPin = 29, uint32_t carrierHz = 62500, uint16_t dutyRange = 255) {
    _pwmDefaultPin = defaultPin;
    _pwmCarrierHz = carrierHz;
    analogWriteRange(dutyRange);
    analogWriteFreq(_pwmCarrierHz);
  }

  void attachMCP4921(MCP4921* m) {
    _mcp = m;
    _dacWriteHook = nullptr;
  }
  void attachDacWriteHook(DacWrite12Hook fn) {
    _dacWriteHook = fn;
    _mcp = nullptr;
  }

  void dacPerfTest() {
    if (!_console) _console = &Serial;
    if (!_mcp && !_dacWriteHook) {
      _console->println("DAC perf: no DAC attached");
      return;
    }
    _console->println();
    _console->println("dacPerfTest");
    uint32_t start = micros(), stop = start;
    if (_mcp) {
      volatile int sink = 0;
      start = micros();
      for (uint16_t v = 0; v < _mcp->maxValue(); v++) sink = _mcp->write(v, 0);
      stop = micros();
      _console->print(_mcp->maxValue());
      _console->print(" x MCP.write():\t");
      _console->print(stop - start);
      _console->print("\t");
      _console->println((stop - start) / (_mcp->maxValue() + 1.0));
      delay(10);
      start = micros();
      for (uint16_t v = 0; v < _mcp->maxValue(); v++) _mcp->fastWriteA(v);
      stop = micros();
      _console->print(_mcp->maxValue());
      _console->print(" x MCP.fastWriteA():\t");
      _console->print(stop - start);
      _console->print("\t");
      _console->println((stop - start) / (_mcp->maxValue() + 1.0));
      _mcp->fastWriteA(0);
    } else {
      const uint16_t maxVal = 4095;
      start = micros();
      for (uint16_t v = 0; v <= maxVal; ++v) _dacWriteHook(v);
      stop = micros();
      _console->print(maxVal);
      _console->print(" x dacWrite12():\t");
      _console->print(stop - start);
      _console->print("\t");
      _console->println((stop - start) / (maxVal + 1.0));
      _dacWriteHook(0);
    }
  }

  bool playWavDAC(const char* path) {
    WavInfo wi{};
    if (!prepare(path, wi)) return false;
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
      uint32_t got = readBackoff(path, off, buf, n);
      if (got == 0) {
        logf("wav: read error/EOF at off=%lu\n", (unsigned long)off);
        dacWrite12(0);
        return false;
      }
      off += got;
      left -= got;
      for (uint32_t i = 0; i < got; ++i) {
        if (shouldStop()) {
          dacWrite12(0);
          logln("\n(wav stopped)");
          return true;
        }
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

  bool playWavPWM(const char* path, int pin = -1) {
    WavInfo wi{};
    if (!prepare(path, wi)) return false;
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
      uint32_t got = readBackoff(path, off, buf, n);
      if (got == 0) {
        logf("wav: read error/EOF at off=%lu\n", (unsigned long)off);
        analogWrite(usePin, 0);
        return false;
      }
      off += got;
      left -= got;
      for (uint32_t i = 0; i < got; ++i) {
        if (shouldStop()) {
          analogWrite(usePin, 0);
          logln("\n(wav stopped)");
          return true;
        }
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

  bool playWavPWMDAC(const char* path, int gainPercent = 50) {
    WavInfo wi{};
    if (!prepare(path, wi)) return false;
    if (gainPercent < 0) gainPercent = 0;
    if (gainPercent > 400) gainPercent = 400;
    logf("wav: %lu Hz, %u-bit, mono, data=%lu bytes (PWMDAC gain=%d%%)\n",
         (unsigned long)wi.sampleRate, (unsigned)wi.bitsPerSample, (unsigned long)wi.dataSize, gainPercent);
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
      uint32_t got = readBackoff(path, off, buf, n);
      if (got == 0) {
        dacWrite12(0);
        logf("wav: read error/EOF at off=%lu\n", (unsigned long)off);
        return false;
      }
      off += got;
      left -= got;
      for (uint32_t i = 0; i < got; ++i) {
        if (shouldStop()) {
          dacWrite12(0);
          logln("\n(wav stopped)");
          return true;
        }
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

  static inline uint16_t rd16le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }
  static inline uint32_t rd32le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

#ifndef AUDIOWAVOUT_HEAL_SILENCE_MAX
// Max consecutive bytes to synthesize as silence (0x80) at a "bad" offset before failing.
// Set to 0 to disable healing.
#define AUDIOWAVOUT_HEAL_SILENCE_MAX 128
#endif
  // Backoff reader with healing:
  // - tries SL, then 128,64,32,16,8,4,2,1
  // - if still 0 at this offset, inserts a few 0x80 bytes (silence), advances, and continues
  uint32_t readBackoff(const char* path, uint32_t off, uint8_t* buf, uint32_t len) {
    if (!_fs.readFileRange) return 0;
    if (len == 0) return 0;

    uint32_t total = 0;
    const uint32_t SL = (AUDIOWAVOUT_READ_SLICE >= 16) ? AUDIOWAVOUT_READ_SLICE : 16;

    while (total < len) {
      uint32_t rem = len - total;

      // Try a descending set of slice sizes
      const uint32_t tries[] = { SL, 128, 64, 32, 16, 8, 4, 2, 1 };
      uint32_t gotThisCycle = 0;

      for (size_t ti = 0; ti < sizeof(tries) / sizeof(tries[0]); ++ti) {
        uint32_t want = (rem < tries[ti]) ? rem : tries[ti];
        if (want == 0) continue;
        uint32_t got = _fs.readFileRange(path, off + total, buf + total, want);
        if (got > 0) {
          gotThisCycle = got;
          break;
        }
        // else try smaller
      }

      if (gotThisCycle == 0) {
#if AUDIOWAVOUT_HEAL_SILENCE_MAX > 0
        // Heal by inserting a short run of 0x80 (silence). This masks unreadable
        // offsets (e.g. page-boundary quirks) so playback continues smoothly.
        uint32_t heal = rem;
        if (heal > AUDIOWAVOUT_HEAL_SILENCE_MAX) heal = AUDIOWAVOUT_HEAL_SILENCE_MAX;
        memset(buf + total, 0x80, heal);
        total += heal;
        // Optional: log only the first time we heal at this call site
        // logf("wav: healed %lu byte(s) of silence at off=%lu\n", (unsigned long)heal, (unsigned long)(off + total - heal));
        // Cooperative yield
        tightLoop();
        if (_service) _service();
        continue;
#else
        // Healing disabled; return whatever we got so far
        return total;
#endif
      }

      total += gotThisCycle;
      tightLoop();
      if (_service) _service();
    }

    return total;
  }

  bool readFully(const char* path, uint32_t off, uint8_t* buf, uint32_t len) {
    uint32_t got = 0;
    while (got < len) {
      uint32_t n = readBackoff(path, off + got, buf + got, len - got);
      if (n == 0) break;
      got += n;
    }
    return got == len;
  }

  bool prepare(const char* path, WavInfo& wi) {
    if (!checkFsReady() || !path || !*path) {
      logln("usage: wav <mode> <file>");
      return false;
    }
    if (!_fs.exists(path)) {
      logln("wav: file not found");
      return false;
    }

    // get file size (optional)
    uint32_t fsz = 0;
    bool haveSize = _fs.getFileSize && _fs.getFileSize(path, fsz);

    // RIFF/WAVE header
    uint8_t head[12];
    if (!readFully(path, 0, head, sizeof(head))) {
      logln("wav: failed to read header");
      return false;
    }
    if (memcmp(head, "RIFF", 4) != 0 || memcmp(head + 8, "WAVE", 4) != 0) {
      logln("wav: not a RIFF/WAVE file");
      return false;
    }

    // Walk chunks; accept extra chunks; clamp data to file end if size known.
    uint32_t off = 12;
    bool gotFmt = false, gotData = false;
    while (!gotFmt || !gotData) {
      uint8_t chdr[8];
      if (!readFully(path, off, chdr, 8)) {
        logln("wav: read chunk header failed");
        return false;
      }
      uint32_t cid = rd32le(chdr);
      uint32_t csz = rd32le(chdr + 4);
      const char id0 = (char)(cid & 0xFF);
      const char id1 = (char)((cid >> 8) & 0xFF);
      const char id2 = (char)((cid >> 16) & 0xFF);
      const char id3 = (char)((cid >> 24) & 0xFF);

      if (id0 == 'f' && id1 == 'm' && id2 == 't' && id3 == ' ') {
        uint8_t fmt[32];
        uint32_t toRead = csz > sizeof(fmt) ? sizeof(fmt) : csz;
        if (!readFully(path, off + 8, fmt, toRead)) {
          logln("wav: failed to read fmt");
          return false;
        }
        wi.audioFormat = rd16le(fmt + 0);
        wi.channels = rd16le(fmt + 2);
        wi.sampleRate = rd32le(fmt + 4);
        wi.bitsPerSample = (toRead >= 16) ? rd16le(fmt + 14) : 0;
        gotFmt = true;
      } else if (id0 == 'd' && id1 == 'a' && id2 == 't' && id3 == 'a') {
        wi.dataOffset = off + 8;
        if (haveSize && wi.dataOffset < fsz) {
          uint32_t maxLen = fsz - wi.dataOffset;
          wi.dataSize = (csz < maxLen) ? csz : maxLen;
        } else {
          wi.dataSize = csz;
        }
        gotData = true;
      }
      // advance (word aligned)
      uint32_t step = 8 + csz + (csz & 1u);
      if (step == 0 || (off + step) < off) {
        logln("wav: invalid chunk size");
        return false;
      }
      off += step;
      if (haveSize && off > fsz) {
        logln("wav: chunk walk exceeded file size");
        return false;
      }
      if (!haveSize && gotFmt && gotData) break;
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
    if (wi.dataSize == 0) {
      logln("wav: empty data");
      return false;
    }

    return true;
  }

  void dacWrite12(uint16_t v) {
    if (_mcp) _mcp->fastWriteA(v);
    else if (_dacWriteHook) _dacWriteHook(v);
  }

  void waitUntil(uint32_t target) {
    for (;;) {
      int32_t d = (int32_t)(micros() - target);
      if (d >= 0) break;
      tightLoop();
      yield();
    }
  }

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
    char buf[160];
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
    // no-op
#endif
  }

private:
  FS _fs;
  Print* _console;
  CancelHook _cancel;
  ServiceHook _service;
  bool _watchSerialForQ;
  uint8_t _pwmDefaultPin;
  uint32_t _pwmCarrierHz;
  MCP4921* _mcp;
  DacWrite12Hook _dacWriteHook;
};