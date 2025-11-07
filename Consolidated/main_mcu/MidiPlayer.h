#pragma once
/*
  MidiPlayer.h
  Single-header, no-interpreter MIDI (SMF) player for RP2040/RP2350 Arduino builds.
  - Parses Type-0/Type-1 SMF and creates a monophonic note list from one track
  - Plays via:
      1) Buzzer digital square (GPIO toggling)
      2) PWM square (analogWrite frequency set per-note, 50% duty)
      3) DAC or “PWMDAC” with simple waveforms (sine, 50% square, saw) using MCP4921 or a user hook
  - Minimal FS callback table to read files from your SimpleFS
  - Optional cancel and service hooks, console logging
  - No dependency on CoProcLang or scripting
  Notes:
  - For DAC modes, playback is streamed in real time with micros()-based scheduling
  - For PWM, analogWriteFreq() sets PWM frequency; 50% duty for square wave
  - For buzzer digital, uses delayMicroseconds toggling (best for piezos)
  - This is a simple monophonic player (track selection logic included)
  Dependencies:
    #include <Arduino.h>
    #include <SPI.h>       (if using MCP4921)
    #include "MCP_DAC.h"   (if using MCP4921 backend)
*/
#include <Arduino.h>
#include <math.h>
#include <stdarg.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef tight_loop_contents
#define tight_loop_contents() \
  do { \
  } while (0)
#endif

class MCP4921;  // forward declaration (pointer only)

class MidiPlayer {
public:
  // Minimal FS ops you already can provide from ActiveFS
  struct FS {
    bool (*exists)(const char* name) = nullptr;
    bool (*getFileSize)(const char* name, uint32_t& size) = nullptr;
    uint32_t (*readFile)(const char* name, uint8_t* buf, uint32_t sz) = nullptr;
    uint32_t (*readFileRange)(const char* name, uint32_t off, uint8_t* buf, uint32_t len) = nullptr;
  };
  // Optional hooks
  typedef bool (*CancelHook)();              // return true to abort
  typedef void (*ServiceHook)();             // run background tasks during playback
  typedef void (*DacWrite12Hook)(uint16_t);  // for custom 12-bit DAC write if not using MCP4921
  enum class Backend {
    BuzzerDigital,  // GPIO toggle, square wave
    PWMSquare,      // analogWrite() square wave; 50% duty, frequency per note
    DAC,            // DAC synth with waveform
    PWMDAC          // alias of DAC (same pipeline; name kept for compatibility)
  };
  enum class Waveform {
    Sine,
    Square50,  // 50% duty cycle square
    Saw
  };
  enum class Debug : uint8_t {
    Off = 0,
    Errors = 1,
    Info = 2,
    Verbose = 3
  };
  struct Config {
    Backend backend = Backend::PWMSquare;
    // Buzzer or PWM output pin (ignored for DAC-only if using MCP on SPI)
    int outPin = 29;      // Default GP29 for PWM/buzzer
    int ledPin = 2;       // Optional LED pin; -1 to disable (keep GP2 by default)
    int trackIndex = -1;  // <0 => auto-select track with most NoteOn
    int gapMs = 0;        // gap (silence) between notes
    // PWM params
    uint16_t pwmRange = 255;  // analogWriteRange(255) default
    // DAC params
    Waveform waveform = Waveform::Sine;
    int dacAmpPercent = 60;  // 0..100 simple amplitude
    uint32_t sampleRate = 22050;
    // General
    bool allowSerialQ = true;  // 'q' stops playback on USB Serial
    // Debug
    Debug debug = Debug::Off;        // stage logs
    bool debugProgress = false;      // playback position printouts
    uint16_t debugIntervalMs = 250;  // progress print interval
  };

  // Public API
  MidiPlayer()
    : _fs{}, _console(&Serial), _cancel(nullptr), _service(nullptr),
      _allowSerialQ(true), _mcp(nullptr), _dacWriteHook(nullptr),
      _stopReason(StopReason::None) {}

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
    _allowSerialQ = en;
  }
  // MCP4921 (DAC) backend
  void attachMCP4921(MCP4921* m) {
    _mcp = m;
    _dacWriteHook = nullptr;
  }
  // Custom 12-bit DAC backend
  void attachDacWriteHook(DacWrite12Hook fn) {
    _dacWriteHook = fn;
    _mcp = nullptr;
  }
  // PWM global defaults (call once if you plan to use PWM)
  void beginPWM(uint16_t range = 255) {
    analogWriteRange(range);
  }

  // Main entry: play a MIDI file using provided work buffer
  // workBuf must hold the whole MIDI file plus note list (8 bytes per note entry).
  // If capacity is insufficient, returns false.
  bool playSMF(const char* path, const Config& cfg, uint8_t* workBuf, uint32_t workCap) {
    unsigned long t_start = micros();
    if (!checkFsReady() || !path || !*path || !workBuf || workCap == 0) {
      logln("midi: bad args or FS not ready");
      return false;
    }
    if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "midi: begin '%s'\n", path);

    // 1) Load file into memory
    unsigned long t_load0 = micros();
    uint32_t midiSize = 0;
    if (!_fs.getFileSize(path, midiSize) || midiSize == 0 || midiSize > workCap) {
      logln("midi: getFileSize failed or buffer too small");
      return false;
    }
    if (_fs.readFile(path, workBuf, midiSize) != midiSize) {
      logln("midi: readFile failed");
      return false;
    }
    unsigned long t_load1 = micros();
    if (dbgOn(cfg, Debug::Info)) {
      dbgf(cfg, "midi: loaded %lu bytes in %lu us (cap %lu)\n",
           (unsigned long)midiSize, (unsigned long)(t_load1 - t_load0), (unsigned long)workCap);
    }

    // 2) Parse header, tempo map, and track selection
    const uint8_t* data = workBuf;
    uint32_t len = midiSize;
    uint16_t fmt = 0, ntrks = 0, division = 0;
    uint32_t off = 0;
    if (!parseHeaderSMF(data, len, fmt, ntrks, division, off)) {
      logln("midi: bad header");
      return false;
    }
    if ((division & 0x8000) != 0) {
      logln("midi: SMPTE time not supported");
      return false;
    }
    uint16_t tpq = division;
    if (tpq == 0) {
      logln("midi: zero TPQ");
      return false;
    }
    if (ntrks > MIDI_MAX_TRACKS) ntrks = MIDI_MAX_TRACKS;

    if (dbgOn(cfg, Debug::Info)) {
      dbgf(cfg, "midi: header: fmt=%u tracks=%u tpq=%u dataOff=%lu\n",
           (unsigned)fmt, (unsigned)ntrks, (unsigned)tpq, (unsigned long)off);
    }

    // Tempo map + per-track note-on counts
    unsigned long t_parse0 = micros();
    TempoEvent tempo[MIDI_MAX_TEMPO_EVENTS + 1];
    uint32_t tempoCount = 0;
    tempo[tempoCount].absTicks = 0;
    tempo[tempoCount].usPerBeat = 500000;  // default 120BPM
    tempoCount++;

    uint32_t trackNoteOnCounts[MIDI_MAX_TRACKS];
    for (uint16_t i = 0; i < ntrks; ++i) trackNoteOnCounts[i] = 0;

    uint32_t trkOff = off;
    for (uint16_t i = 0; i < ntrks; ++i) {
      uint32_t trkLen = 0;
      if (!findTrackAt(data, len, trkOff, trkLen)) {
        logln("midi: track not found");
        return false;
      }
      uint32_t noteOnCount = 0;
      uint32_t lastAbs = 0;
      buildTempoMapPass(&data[trkOff], trkLen, noteOnCount, tempo, tempoCount, MIDI_MAX_TEMPO_EVENTS, lastAbs);
      trackNoteOnCounts[i] = noteOnCount;
      trkOff += trkLen;
    }
    sortTempo(tempo, tempoCount);
    tempoCount = coalesceTempoSameTick(tempo, tempoCount);

    if (dbgOn(cfg, Debug::Info)) {
      dbgf(cfg, "midi: tempo events=%lu\n", (unsigned long)tempoCount);
      for (uint16_t i = 0; i < ntrks; ++i) {
        dbgf(cfg, "midi: track %u: noteOn=%lu\n", (unsigned)i, (unsigned long)trackNoteOnCounts[i]);
      }
    }

    int selIndex = cfg.trackIndex;
    if (selIndex < 0) {
      int best = -1;
      uint32_t bestC = 0;
      for (uint16_t i = 0; i < ntrks; ++i) {
        if (trackNoteOnCounts[i] > bestC) {
          bestC = trackNoteOnCounts[i];
          best = (int)i;
        }
      }
      selIndex = best;
    }
    if (selIndex < 0 || selIndex >= (int)ntrks) {
      logln("midi: no suitable track");
      return false;
    }
    if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "midi: selected track=%d\n", selIndex);

    // 3) Two-pass parse for selected track -> count notes then extract to entries
    uint32_t noteCount = 0;
    if (!countSelectedTrackMonophonic(data, len, off, selIndex, tempo, tempoCount, tpq, noteCount)) {
      logln("midi: count pass failed");
      return false;
    }
    if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "midi: monophonic count=%lu\n", (unsigned long)noteCount);
    if (noteCount == 0) {
      logln("midi: no notes");
      return true;
    }

    // Align entries start for safe 32-bit access on RP2040
    const uint32_t align = (uint32_t)alignof(CoplmNoteEntry);  // typically 4
    const uint32_t entriesOff = align_up_u32(midiSize, align);
    const uint32_t pad = entriesOff - midiSize;
    const uint32_t need = entriesOff + noteCount * sizeof(CoplmNoteEntry);
    if (dbgOn(cfg, Debug::Info)) {
      dbgf(cfg, "midi: buffer use: file=%lu entries=%lu*%u -> need=%lu cap=%lu\n",
           (unsigned long)midiSize, (unsigned long)noteCount, (unsigned)sizeof(CoplmNoteEntry),
           (unsigned long)need, (unsigned long)workCap);
      if (pad > 0) {
        dbgf(cfg, "midi: align pad=%lu (from %lu to %lu)\n",
             (unsigned long)pad, (unsigned long)midiSize, (unsigned long)entriesOff);
      }
    }
    if (need > workCap) {
      logf("midi: workBuf too small, need %lu bytes\n", (unsigned long)need);
      return false;
    }

    CoplmNoteEntry* entries = reinterpret_cast<CoplmNoteEntry*>(workBuf + entriesOff);
    uint32_t outN = 0;
    if (!extractSelectedTrackMonophonic(data, len, off, selIndex, tempo, tempoCount, tpq, entries, noteCount, outN)) {
      logln("midi: extract pass failed");
      return false;
    }
    unsigned long t_parse1 = micros();
    if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "midi: extracted=%lu (parse %lu us)\n",
                                      (unsigned long)outN, (unsigned long)(t_parse1 - t_parse0));
    if (outN == 0) {
      logln("midi: no playable notes");
      return true;
    }

    // Estimate total playback time (us)
    uint64_t total_us = 0;
    for (uint32_t i = 0; i < outN; ++i) {
      total_us += entries[i].dur_us;
      if (cfg.gapMs > 0 && i + 1 < outN) total_us += (uint64_t)cfg.gapMs * 1000ULL;
    }
    if (dbgOn(cfg, Debug::Info)) {
      double secs = (double)total_us / 1000000.0;
      dbgf(cfg, "midi: estimated duration: %.2fs\n", secs);
    }

    // 4) Playback using chosen backend
    bool ok = false;
    switch (cfg.backend) {
      case Backend::BuzzerDigital:
        if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "play: backend=buzzer notes=%lu\n", (unsigned long)outN);
        ok = playNotesBuzzer(entries, outN, cfg);
        break;
      case Backend::PWMSquare:
        if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "play: backend=pwm notes=%lu\n", (unsigned long)outN);
        ok = playNotesPWMSquare(entries, outN, cfg);
        break;
      case Backend::DAC:
      case Backend::PWMDAC:
        if (dbgOn(cfg, Debug::Info)) dbgf(cfg, "play: backend=dac notes=%lu sr=%lu\n",
                                          (unsigned long)outN, (unsigned long)(cfg.sampleRate ? cfg.sampleRate : 22050));
        ok = playNotesDAC(entries, outN, cfg);
        break;
    }
    if (dbgOn(cfg, Debug::Info)) {
      unsigned long t_end = micros();
      dbgf(cfg, "midi: end status=%s (total %lu us)\n", ok ? "ok" : "fail", (unsigned long)(t_end - t_start));
    }
    return ok;
  }

private:
  // =================== Internal types and helpers ===================
  struct CoplmNoteEntry {
    uint8_t note;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t dur_us;  // microseconds
  };
  struct TempoEvent {
    uint32_t absTicks;
    uint32_t usPerBeat;
  };
  static constexpr uint32_t MIDI_MAX_TEMPO_EVENTS = 256;
  static constexpr uint32_t MIDI_MAX_TRACKS = 32;

  // Progress/debug helper
  struct Progress {
    bool enabled = false;
    uint32_t intervalMs = 250;
    uint64_t total_us = 0;
    unsigned long lastMs = 0;
  };
  void progressInit(Progress& pr, const CoplmNoteEntry* e, uint32_t n, const Config& cfg) {
    pr.enabled = cfg.debugProgress;
    if (!pr.enabled) return;
    pr.intervalMs = cfg.debugIntervalMs ? cfg.debugIntervalMs : 250;
    uint64_t total = 0;
    for (uint32_t i = 0; i < n; ++i) {
      total += (uint64_t)e[i].dur_us;
      if (cfg.gapMs > 0 && i + 1 < n) total += (uint64_t)cfg.gapMs * 1000ULL;
    }
    pr.total_us = total;
    pr.lastMs = millis();
    dbgf(cfg, "midi: play start: %lu notes, est %.2fs total\n",
         (unsigned long)n, (double)pr.total_us / 1000000.0);
  }
  void progressTick(Progress& pr, uint64_t elapsed_us, const Config& cfg) {
    if (!pr.enabled) return;
    unsigned long now = millis();
    if ((uint32_t)(now - pr.lastMs) >= pr.intervalMs) {
      double e = (double)elapsed_us / 1000000.0;
      double t = (double)pr.total_us / 1000000.0;
      double pct = (pr.total_us > 0) ? (100.0 * (double)elapsed_us / (double)pr.total_us) : 0.0;
      dbgf(cfg, "midi: %.2fs / %.2fs (%.0f%%)\n", e, t, pct);
      pr.lastMs = now;
    }
  }
  void progressDone(Progress& pr, const Config& cfg) {
    if (!pr.enabled) return;
    logln("midi: play end");
  }

  // Logging helpers
  inline bool dbgOn(const Config& cfg, Debug level) const {
    return (uint8_t)cfg.debug >= (uint8_t)level && _console;
  }
  void dbgf(const Config& cfg, const char* fmt, ...) {
    if (!_console || cfg.debug == Debug::Off) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    _console->print(buf);
  }

  // Stop reason to make diagnostics clear
  enum class StopReason : uint8_t { None,
                                    SerialQ,
                                    CancelHook };
  void logStopIfAny(const Config& cfg) {
    if (!dbgOn(cfg, Debug::Info)) return;
    switch (_stopReason) {
      case StopReason::SerialQ: logln("play: stop by serial 'q'"); break;
      case StopReason::CancelHook: logln("play: stop by cancel hook"); break;
      default: break;
    }
    _stopReason = StopReason::None;
  }

  // LE VLQ
  static inline uint32_t vlq(const uint8_t*& p, const uint8_t* end, bool& ok) {
    ok = false;
    uint32_t v = 0;
    int cnt = 0;
    while (p < end) {
      uint8_t b = *p++;
      v = (v << 7) | (b & 0x7F);
      cnt++;
      if ((b & 0x80) == 0) {
        ok = true;
        return v;
      }
      if (cnt > 4) return 0;
    }
    return 0;
  }
  static bool parseHeaderSMF(const uint8_t* data, uint32_t len, uint16_t& fmt, uint16_t& ntrks, uint16_t& division, uint32_t& off) {
    off = 0;
    if (len < 14) return false;
    if (!(data[0] == 'M' && data[1] == 'T' && data[2] == 'h' && data[3] == 'd')) return false;
    uint32_t hdrLen = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    if (hdrLen < 6 || len < 8 + hdrLen) return false;
    fmt = (uint16_t)((data[8] << 8) | data[9]);
    ntrks = (uint16_t)((data[10] << 8) | data[11]);
    division = (uint16_t)((data[12] << 8) | data[13]);
    off = 8 + hdrLen;
    return true;
  }
  static bool findTrackAt(const uint8_t* data, uint32_t len, uint32_t& off, uint32_t& trackLen) {
    if (off + 8 > len) return false;
    if (!(data[off] == 'M' && data[off + 1] == 'T' && data[off + 2] == 'r' && data[off + 3] == 'k')) return false;
    trackLen = (uint32_t)(data[off + 4] << 24) | (uint32_t)(data[off + 5] << 16) | (uint32_t)(data[off + 6] << 8) | (uint32_t)(data[off + 7]);
    off += 8;
    if (off + trackLen > len) return false;
    return true;
  }
  static void buildTempoMapPass(const uint8_t* trk, uint32_t trkLen,
                                uint32_t& outNoteOnCount,
                                TempoEvent* tempo, uint32_t& tempoCount, uint32_t tempoMax,
                                uint32_t& lastAbs) {
    const uint8_t* p = trk;
    const uint8_t* end = trk + trkLen;
    uint8_t run = 0;
    uint32_t absTicks = 0;
    outNoteOnCount = 0;
    lastAbs = 0;
    while (p < end) {
      bool ok = false;
      uint32_t delta = vlq(p, end, ok);
      if (!ok) break;
      absTicks += delta;
      if (p >= end) break;
      uint8_t st = *p++;
      if (st < 0x80) {
        if (!run) break;
        p--;
        st = run;
      } else if (st < 0xF0) {
        run = st;
      }
      if ((st & 0xF0) == 0x90) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        uint8_t vel = *p++;
        (void)note;
        if (vel > 0) outNoteOnCount++;
      } else if ((st & 0xF0) == 0x80) {
        if (p + 1 > end) break;
        p += 2;
      } else if (st == 0xFF) {
        if (p >= end) break;
        uint8_t meta = *p++;
        bool ok2 = false;
        uint32_t ml = vlq(p, end, ok2);
        if (!ok2 || p + ml > end) break;
        if (meta == 0x51 && ml == 3) {
          uint32_t uspb = (uint32_t)p[0] << 16 | (uint32_t)p[1] << 8 | (uint32_t)p[2];
          if (tempoCount < tempoMax) {
            tempo[tempoCount].absTicks = absTicks;
            tempo[tempoCount].usPerBeat = uspb;
            tempoCount++;
          }
        }
        p += ml;
      } else {
        if (st == 0xF0 || st == 0xF7) {
          bool ok3 = false;
          uint32_t sl = vlq(p, end, ok3);
          if (!ok3 || p + sl > end) break;
          p += sl;
        } else {
          uint8_t hi = (st & 0xF0);
          int need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
          if (p + need > end) break;
          p += need;
        }
      }
      lastAbs = absTicks;
    }
  }
  static inline void sortTempo(TempoEvent* t, uint32_t n) {
    for (uint32_t i = 1; i < n; ++i) {
      TempoEvent key = t[i];
      uint32_t j = i;
      while (j > 0 && t[j - 1].absTicks > key.absTicks) {
        t[j] = t[j - 1];
        --j;
      }
      t[j] = key;
    }
  }
  static inline uint32_t coalesceTempoSameTick(TempoEvent* t, uint32_t n) {
    if (n == 0) return 0;
    uint32_t w = 1;
    for (uint32_t i = 1; i < n; ++i) {
      if (t[i].absTicks == t[w - 1].absTicks) {
        t[w - 1] = t[i];
      } else {
        t[w++] = t[i];
      }
    }
    return w;
  }
  static inline uint32_t ticksToMicrosWithTempo(uint32_t ticks, const TempoEvent* tempo, uint32_t tempoCount, uint16_t tpq) {
    uint32_t us = 0, prevTick = 0, curUsPerBeat = 500000;
    for (uint32_t i = 0; i < tempoCount; ++i) {
      uint32_t tTick = tempo[i].absTicks;
      if (tTick > ticks) break;
      if (tTick > prevTick) {
        uint32_t dt = tTick - prevTick;
        us += (uint32_t)((uint64_t)dt * (uint64_t)curUsPerBeat / (uint64_t)tpq);
        prevTick = tTick;
      }
      curUsPerBeat = tempo[i].usPerBeat;
    }
    if (ticks > prevTick) {
      uint32_t dt = ticks - prevTick;
      us += (uint32_t)((uint64_t)dt * (uint64_t)curUsPerBeat / (uint64_t)tpq);
    }
    return us;
  }
  // Count monophonic notes for selected track
  static bool countSelectedTrackMonophonic(const uint8_t* data, uint32_t len, uint32_t off,
                                           int selIndex,
                                           const TempoEvent* tempo, uint32_t tempoCount, uint16_t tpq,
                                           uint32_t& outNotesCount) {
    outNotesCount = 0;
    uint32_t scanOff = off;
    for (int i = 0; i < selIndex; ++i) {
      uint32_t tl = 0;
      if (!findTrackAt(data, len, scanOff, tl)) return false;
      scanOff += tl;
    }
    uint32_t trkLen = 0;
    if (!findTrackAt(data, len, scanOff, trkLen)) return false;
    const uint8_t* p = &data[scanOff];
    const uint8_t* end = p + trkLen;
    uint8_t run = 0;
    uint32_t absTicks = 0;
    int currentNote = -1;
    uint32_t currentStart = 0;
    bool currentActive = false;
    while (p < end) {
      bool ok = false;
      uint32_t delta = vlq(p, end, ok);
      if (!ok) break;
      absTicks += delta;
      if (p >= end) break;
      uint8_t st = *p++;
      if (st < 0x80) {
        if (!run) break;
        p--;
        st = run;
      } else if (st < 0xF0) {
        run = st;
      }
      if ((st & 0xF0) == 0x90) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        uint8_t vel = *p++;
        if (vel > 0) {
          if (currentActive) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs) outNotesCount++;
          }
          currentNote = note;
          currentStart = absTicks;
          currentActive = true;
        } else {
          if (currentActive && currentNote == note) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs) outNotesCount++;
            currentActive = false;
          }
        }
      } else if ((st & 0xF0) == 0x80) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        (void)*p++;
        if (currentActive && currentNote == (int)note) {
          uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
          uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
          if (endUs > startUs) outNotesCount++;
          currentActive = false;
        }
      } else if (st == 0xFF) {
        if (p >= end) break;
        uint8_t meta = *p++;
        bool ok2 = false;
        uint32_t ml = vlq(p, end, ok2);
        if (!ok2 || p + ml > end) break;
        p += ml;
      } else {
        if (st == 0xF0 || st == 0xF7) {
          bool ok3 = false;
          uint32_t sl = vlq(p, end, ok3);
          if (!ok3 || p + sl > end) break;
          p += sl;
        } else {
          uint8_t hi = (st & 0xF0);
          int need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
          if (p + need > end) break;
          p += need;
        }
      }
    }
    if (currentActive) {
      uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
      uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
      if (endUs > startUs) outNotesCount++;
    }
    return true;
  }
  static bool extractSelectedTrackMonophonic(const uint8_t* data, uint32_t len, uint32_t off,
                                             int selIndex,
                                             const TempoEvent* tempo, uint32_t tempoCount, uint16_t tpq,
                                             CoplmNoteEntry* out, uint32_t cap, uint32_t& outCount) {
    outCount = 0;
    uint32_t scanOff = off;
    for (int i = 0; i < selIndex; ++i) {
      uint32_t tl = 0;
      if (!findTrackAt(data, len, scanOff, tl)) return false;
      scanOff += tl;
    }
    uint32_t trkLen = 0;
    if (!findTrackAt(data, len, scanOff, trkLen)) return false;
    const uint8_t* p = &data[scanOff];
    const uint8_t* end = p + trkLen;
    uint8_t run = 0;
    uint32_t absTicks = 0;
    int currentNote = -1;
    uint32_t currentStart = 0;
    bool currentActive = false;
    while (p < end) {
      bool ok = false;
      uint32_t delta = vlq(p, end, ok);
      if (!ok) break;
      absTicks += delta;
      if (p >= end) break;
      uint8_t st = *p++;
      if (st < 0x80) {
        if (!run) break;
        p--;
        st = run;
      } else if (st < 0xF0) {
        run = st;
      }
      if ((st & 0xF0) == 0x90) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        uint8_t vel = *p++;
        if (vel > 0) {
          if (currentActive && outCount < cap) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs) {
              out[outCount].note = (uint8_t)currentNote;
              out[outCount].reserved0 = 0;
              out[outCount].reserved1 = 0;
              out[outCount].dur_us = endUs - startUs;
              outCount++;
            }
          }
          currentNote = note;
          currentStart = absTicks;
          currentActive = true;
        } else {
          if (currentActive && currentNote == note && outCount < cap) {
            uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
            uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
            if (endUs > startUs) {
              out[outCount].note = (uint8_t)currentNote;
              out[outCount].reserved0 = 0;
              out[outCount].reserved1 = 0;
              out[outCount].dur_us = endUs - startUs;
              outCount++;
            }
            currentActive = false;
          }
        }
      } else if ((st & 0xF0) == 0x80) {
        if (p + 1 > end) break;
        uint8_t note = *p++;
        (void)*p++;
        if (currentActive && currentNote == (int)note && outCount < cap) {
          uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
          uint32_t endUs = ticksToMicrosWithTempo(absTicks, tempo, tempoCount, tpq);
          if (endUs > startUs) {
            out[outCount].note = (uint8_t)currentNote;
            out[outCount].reserved0 = 0;
            out[outCount].reserved1 = 0;
            out[outCount].dur_us = endUs - startUs;
            outCount++;
          }
          currentActive = false;
        }
      } else if (st == 0xFF) {
        if (p >= end) break;
        uint8_t meta = *p++;
        bool ok2 = false;
        uint32_t ml = vlq(p, end, ok2);
        if (!ok2 || p + ml > end) break;
        p += ml;
      } else {
        if (st == 0xF0 || st == 0xF7) {
          bool ok3 = false;
          uint32_t sl = vlq(p, end, ok3);
          if (!ok3 || p + sl > end) break;
          p += sl;
        } else {
          uint8_t hi = (st & 0xF0);
          int need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
          if (p + need > end) break;
          p += need;
        }
      }
    }
    if (currentActive && outCount < cap) {
      // Close last hanging note at end-of-track time
      uint32_t endAbsTicks = 0;
      endAbsTicks = absTicks;
      uint32_t startUs = ticksToMicrosWithTempo(currentStart, tempo, tempoCount, tpq);
      uint32_t endUs = ticksToMicrosWithTempo(endAbsTicks, tempo, tempoCount, tpq);
      if (endUs > startUs) {
        out[outCount].note = (uint8_t)currentNote;
        out[outCount].reserved0 = 0;
        out[outCount].reserved1 = 0;
        out[outCount].dur_us = endUs - startUs;
        outCount++;
      }
    }
    return true;
  }
  static inline double midiNoteToFreq(int note) {
    double x = (double)(note - 69) / 12.0;
    return 440.0 * pow(2.0, x);
  }

  // =================== Playback backends ===================
  bool playNotesBuzzer(const CoplmNoteEntry* e, uint32_t n, const Config& cfg) {
    if (cfg.outPin < 0) {
      logln("buzzer: no pin");
      return false;
    }
    pinMode((uint8_t)cfg.outPin, OUTPUT);
    if (cfg.ledPin >= 0) pinMode((uint8_t)cfg.ledPin, OUTPUT);

    Progress pr;
    progressInit(pr, e, n, cfg);
    uint64_t songElapsed = 0;

    for (uint32_t i = 0; i < n; ++i) {
      if (shouldStop()) {
        logStopIfAny(cfg);
        progressDone(pr, cfg);
        return true;
      }
      double f = midiNoteToFreq((int)e[i].note);
      if (f < 1.0) f = 1.0;
      uint32_t half = (uint32_t)((1000000.0 / (2.0 * f)) + 0.5);
      if (half == 0) half = 1;
      uint64_t total = e[i].dur_us;
      uint64_t cycles = total / (2ULL * half);
      if (cycles == 0) cycles = 1;
      if (dbgOn(cfg, Debug::Verbose)) {
        dbgf(cfg, "note %lu: n=%u f=%.2fHz dur=%.1fms (buzzer)\n",
             (unsigned long)i, (unsigned)e[i].note, f, (double)total / 1000.0);
      }
      if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, HIGH);
      for (uint64_t c = 0; c < cycles; ++c) {
        if (shouldStop()) break;
        digitalWrite((uint8_t)cfg.outPin, HIGH);
        delayMicroseconds((uint32_t)half);
        digitalWrite((uint8_t)cfg.outPin, LOW);
        delayMicroseconds((uint32_t)half);
        uint64_t noteElapsed = (c + 1) * (2ULL * half);
        if (noteElapsed > total) noteElapsed = total;
        progressTick(pr, songElapsed + noteElapsed, cfg);
        serviceOnce();
      }
      if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, LOW);
      songElapsed += e[i].dur_us;
      if (cfg.gapMs > 0) {
        delay((uint32_t)cfg.gapMs);
        songElapsed += (uint64_t)cfg.gapMs * 1000ULL;
        progressTick(pr, songElapsed, cfg);
      }
      serviceOnce();
    }
    progressDone(pr, cfg);
    return true;
  }

  bool playNotesPWMSquare(const CoplmNoteEntry* e, uint32_t n, const Config& cfg) {
    if (cfg.outPin < 0) {
      logln("pwm: no pin");
      return false;
    }
    pinMode((uint8_t)cfg.outPin, OUTPUT);
    if (cfg.ledPin >= 0) pinMode((uint8_t)cfg.ledPin, OUTPUT);
    analogWriteRange(cfg.pwmRange ? cfg.pwmRange : 255);

    Progress pr;
    progressInit(pr, e, n, cfg);
    uint64_t songElapsed = 0;

    for (uint32_t i = 0; i < n; ++i) {
      if (shouldStop()) {
        logStopIfAny(cfg);
        progressDone(pr, cfg);
        return true;
      }
      double f = midiNoteToFreq((int)e[i].note);
      if (f < 1.0) f = 1.0;
      uint32_t freq = (uint32_t)(f + 0.5);
      if (freq < 1) freq = 1;
      if (dbgOn(cfg, Debug::Verbose)) {
        dbgf(cfg, "note %lu: n=%u f=%.2fHz dur=%.1fms (pwm)\n",
             (unsigned long)i, (unsigned)e[i].note, f, (double)e[i].dur_us / 1000.0);
      }
      analogWriteFreq(freq);
      analogWrite((uint8_t)cfg.outPin, cfg.pwmRange / 2);
      if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, HIGH);
      uint32_t t0 = micros();
      while ((uint32_t)(micros() - t0) < e[i].dur_us) {
        if (shouldStop()) break;
        uint32_t noteElapsed = (uint32_t)(micros() - t0);
        if (noteElapsed > e[i].dur_us) noteElapsed = e[i].dur_us;
        progressTick(pr, songElapsed + (uint64_t)noteElapsed, cfg);
        serviceOnce();
      }
      analogWrite((uint8_t)cfg.outPin, 0);
      if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, LOW);
      songElapsed += e[i].dur_us;
      if (cfg.gapMs > 0) {
        delay((uint32_t)cfg.gapMs);
        songElapsed += (uint64_t)cfg.gapMs * 1000ULL;
        progressTick(pr, songElapsed, cfg);
      }
      serviceOnce();
    }
    progressDone(pr, cfg);
    return true;
  }

  bool playNotesDAC(const CoplmNoteEntry* e, uint32_t n, const Config& cfg) {
    if (!_mcp && !_dacWriteHook) {
      logln("dac: no DAC attached");
      return false;
    }
    if (cfg.ledPin >= 0) pinMode((uint8_t)cfg.ledPin, OUTPUT);
    // Synth stream params
    const uint32_t sr = cfg.sampleRate ? cfg.sampleRate : 22050;
    const int ampPct = clampi(cfg.dacAmpPercent, 0, 100);
    const int mid = 2048;
    const int amp = (int)((2047LL * ampPct) / 100);

    Progress pr;
    progressInit(pr, e, n, cfg);
    uint64_t songElapsed = 0;

    for (uint32_t i = 0; i < n; ++i) {
      if (shouldStop()) {
        dacSafeOff();
        if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, LOW);
        logStopIfAny(cfg);
        progressDone(pr, cfg);
        return true;
      }
      double f = midiNoteToFreq((int)e[i].note);
      if (f < 1.0) f = 1.0;
      if (dbgOn(cfg, Debug::Verbose)) {
        dbgf(cfg, "note %lu: n=%u f=%.2fHz dur=%.1fms (dac)\n",
             (unsigned long)i, (unsigned)e[i].note, f, (double)e[i].dur_us / 1000.0);
      }
      // Phase increment per sample
      double phase = 0.0;
      double incr = f / (double)sr;  // for [0,1) phase
      if (incr <= 0) incr = 1.0 / (double)sr;
      // Micros scheduler q/r (like your WAV code)
      const uint32_t q = 1000000u / sr;
      const uint32_t r = 1000000u % sr;
      uint32_t acc = 0;
      uint32_t elapsed = 0;
      uint32_t nextT = micros();
      if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, HIGH);
      while (elapsed < e[i].dur_us) {
        if (shouldStop()) {
          dacSafeOff();
          if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, LOW);
          logStopIfAny(cfg);
          progressDone(pr, cfg);
          return true;
        }
        // Generate sample in -1..+1
        double v = 0.0;
        switch (cfg.waveform) {
          case Waveform::Sine:
            {
              double ang = (phase * 2.0 * M_PI);
              v = sin(ang);
              break;
            }
          case Waveform::Square50:
            v = (phase < 0.5) ? 1.0 : -1.0;
            break;
          case Waveform::Saw:
            v = (2.0 * phase) - 1.0;
            break;
        }
        int samp = mid + (int)((double)amp * v);
        if (samp < 0) samp = 0;
        if (samp > 4095) samp = 4095;
        dacWrite12((uint16_t)samp);
        // Wait next sample time
        waitUntil(nextT);
        nextT += q;
        acc += r;
        if (acc >= sr) {
          nextT += 1;
          acc -= sr;
        }
        // Advance phase
        phase += incr;
        if (phase >= 1.0) phase -= 1.0;
        // Track elapsed micros
        elapsed += q;
        if (acc >= sr) elapsed += 1;  // matches the +1 above (small drift guard)
        if (_service) _service();

        // Progress update
        uint32_t noteElapsed = elapsed;
        if (noteElapsed > e[i].dur_us) noteElapsed = e[i].dur_us;
        progressTick(pr, songElapsed + (uint64_t)noteElapsed, cfg);
      }
      // End of this note: ensure DAC is off (0-level) and LED off between notes
      dacSafeOff();
      if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, LOW);
      if (cfg.gapMs > 0) {
        delay((uint32_t)cfg.gapMs);
        songElapsed += (uint64_t)cfg.gapMs * 1000ULL;
        progressTick(pr, songElapsed, cfg);
      }
      songElapsed += e[i].dur_us;
      serviceOnce();
    }
    // Completed all notes: ensure the DAC is off
    dacSafeOff();
    if (cfg.ledPin >= 0) digitalWrite((uint8_t)cfg.ledPin, LOW);
    progressDone(pr, cfg);
    return true;
  }

  // =================== Utilities ===================
  bool shouldStop() {
    if (_allowSerialQ && Serial.available()) {
      int c = Serial.peek();
      if (c == 'q' || c == 'Q') {
        (void)Serial.read();
        _stopReason = StopReason::SerialQ;
        return true;
      }
    }
    if (_cancel && _cancel()) {
      _stopReason = StopReason::CancelHook;
      return true;
    }
    return false;
  }
  void serviceOnce() {
    if (_service) _service();
    tight_loop_contents();
    yield();
  }
  void waitUntil(uint32_t target) {
    for (;;) {
      int32_t d = (int32_t)(micros() - target);
      if (d >= 0) break;
      tight_loop_contents();
      yield();
    }
  }
  void dacWrite12(uint16_t v) {
    if (_mcp) _mcp->fastWriteA(v);
    else if (_dacWriteHook) _dacWriteHook(v);
  }
  // Safe DAC off helper (call when stopping or finishing playback)
  void dacSafeOff() {
    if (_mcp) {
      _mcp->fastWriteA(0);
    } else if (_dacWriteHook) {
      _dacWriteHook(0);
    }
  }
  bool checkFsReady() const {
    return _fs.getFileSize && _fs.readFile;
  }
  static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }
  // Align helper for entries region (avoid unaligned 32-bit access on RP2040)
  static inline uint32_t align_up_u32(uint32_t v, uint32_t a) {
    return (v + (a - 1)) & ~(a - 1);
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

private:
  FS _fs;
  Print* _console;
  CancelHook _cancel;
  ServiceHook _service;
  bool _allowSerialQ;
  MCP4921* _mcp;                 // optional
  DacWrite12Hook _dacWriteHook;  // optional
  StopReason _stopReason;
};