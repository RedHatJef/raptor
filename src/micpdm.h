#pragma once
#include <Arduino.h>

// ── Tuning ────────────────────────────────────────────────────────────────────

// Peak amplitude threshold for loud-event detection.
// PDM samples are 16-bit signed (–32768 … +32767).
// A quiet room typically sits around 200–600 counts; a handclap or
// gunshot reaches 4 000+.  Increase to reduce false positives.
static constexpr int16_t  MIC_THRESHOLD_PEAK = 7000;

// PDM hardware gain (0–80 on the nRF52840 PDM peripheral, in 0.5 dB steps).
// 20 ≈ 10 dB — a good starting point for the onboard MSM261D mic.
static constexpr uint8_t  MIC_GAIN           = 10;

// After a loud event, ignore the mic for this many milliseconds before
// re-arming.  Prevents the natural ring-down / echo from re-triggering.
static constexpr uint16_t MIC_REARM_DELAY_MS = 500;

// ── Class ─────────────────────────────────────────────────────────────────────
class MicPDM
{
public:
    bool setup();
    void loop();

    // Returns true (once) after a loud event is detected.
    // Clears the flag on read; subsequent calls return false until the next event.
    bool takeLoudFlag();

    // PDM receive ISR — must be public for use as a C-style callback.
    static void onPDMReceive();

private:
    bool     _isUp         = false;
    bool     _rearming     = false;
    uint32_t _rearmStart_ms = 0;

    volatile bool _loudFlag         = false;
    volatile bool _samplesAvailable = false;
    volatile int  _samplesRead      = 0;

    static constexpr int PDM_SAMPLE_RATE = 16000; // Hz
    static constexpr int PDM_CHANNELS    = 1;     // mono
    static constexpr int BUF_SAMPLES     = 256;   // int16_t samples per callback burst

    // Shared between the PDM ISR and loop().  The ISR writes and sets
    // _samplesAvailable; loop() reads and clears _samplesAvailable.
    static int16_t _sampleBuf[BUF_SAMPLES];

    static MicPDM *_instance;
};
