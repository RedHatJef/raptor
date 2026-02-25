#include "clap_event.h"

ClapEvent::ClapEvent(int32_t _timestamp, size_t reserveCount)
    : timestamp(_timestamp)
{
    if (reserveCount > 0)
    {
        samples.reserve(reserveCount);
    }
}

void ClapEvent::addClapData(const ClapData& data)
{
    samples.push_back(data);
}

void ClapEvent::reserve(size_t count)
{
    samples.reserve(count);
}