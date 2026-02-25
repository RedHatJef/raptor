#include "micpdm.h"
#include <PDM.h>

// ── Statics ───────────────────────────────────────────────────────────────────
int16_t  MicPDM::_sampleBuf[MicPDM::BUF_SAMPLES];
MicPDM  *MicPDM::_instance = nullptr;

// ── PDM receive callback (fires in interrupt / DMA-complete context) ──────────
//
// Keep this short: read the DMA buffer so the hardware can reuse it, then
// record the sample count and set a flag for loop() to process.
void MicPDM::onPDMReceive()
{
    if (!_instance) return;
    int bytes = PDM.read(_sampleBuf, sizeof(_sampleBuf));
    _instance->_samplesRead     = bytes / (int)sizeof(int16_t);
    _instance->_samplesAvailable = true;
}

// ── setup ─────────────────────────────────────────────────────────────────────
bool MicPDM::setup()
{
    _isUp    = false;
    _instance = this;

    PDM.onReceive(onPDMReceive);
    PDM.setGain(MIC_GAIN);

    if (!PDM.begin(PDM_CHANNELS, PDM_SAMPLE_RATE))
    {
        Serial.println(F("MicPDM: PDM init failed"));
        return false;
    }

    _isUp          = true;
    // Suppress the startup transient: PDM mics produce garbage samples for
    // the first ~100 ms after power-on.  Reuse the rearm window so loop()
    // ignores these samples without any additional state.
    _rearming      = true;
    _rearmStart_ms = millis();

    Serial.print(F("MicPDM: ready  gain="));
    Serial.print(MIC_GAIN);
    Serial.print(F("  threshold="));
    Serial.println(MIC_THRESHOLD_PEAK);
    return true;
}

// ── loop ──────────────────────────────────────────────────────────────────────
void MicPDM::loop()
{
    if (!_isUp) return;

    // Suppress detection while waiting for the re-arm window to expire.
    if (_rearming)
    {
        if (millis() - _rearmStart_ms >= MIC_REARM_DELAY_MS)
        {
            _rearming = false;
            Serial.println(F("MicPDM: re-armed"));
        }
        return;
    }

    if (!_samplesAvailable) return;

    // Snapshot volatile fields; clear the flag before processing so the ISR
    // can immediately write the next burst.
    int n           = _samplesRead;
    _samplesAvailable = false;

    // Peak detection: maximum absolute amplitude over the current burst.
    // Avoids sqrt(), suitable for the short, high-amplitude transients of
    // claps and gunshots.
    int16_t peak = 0;
    for (int i = 0; i < n; i++)
    {
        int16_t v = _sampleBuf[i];
        if (v < 0) v = -v;
        if (v > peak) peak = v;
    }

    if (peak >= MIC_THRESHOLD_PEAK)
    {
        _loudFlag      = true;
        _rearming      = true;
        _rearmStart_ms = millis();
        Serial.print(F("MicPDM: loud event  peak="));
        Serial.println(peak);
    }
}

// ── takeLoudFlag ──────────────────────────────────────────────────────────────
bool MicPDM::takeLoudFlag()
{
    if (!_loudFlag) return false;
    _loudFlag = false;
    return true;
}
