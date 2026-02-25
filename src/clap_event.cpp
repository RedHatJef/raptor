#include "clap_event.h"

ClapEvent::ClapEvent()
{
    clear();
}

void ClapEvent::clear()
{
    timestamp = 0;
    temperature = pressure = humidity = 0;
    sampleCount = 0;
    for(int i= 0; i < MAX_SAMPLES; i++) {
        samples[i].clear();
    }
}