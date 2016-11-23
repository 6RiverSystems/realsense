/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <cstdint>
using namespace std;

namespace srs {

struct Sound
{
    Sound(uint8_t volume = 0,
        uint16_t baseFrequency = 0,
        uint16_t cycleRate = 0,
        uint8_t dutyCycle = 0,
        uint16_t numberOfCycles = 0) :
            baseFrequency(baseFrequency),
            cycleRate(cycleRate),
            dutyCycle(dutyCycle),
            numberOfCycles(numberOfCycles),
            volume(volume)
    {}

    virtual ~Sound()
    {}

    uint16_t baseFrequency;

    uint16_t cycleRate;

    uint8_t dutyCycle;

    uint16_t numberOfCycles;

    uint8_t volume;
};

} // namespace srs
