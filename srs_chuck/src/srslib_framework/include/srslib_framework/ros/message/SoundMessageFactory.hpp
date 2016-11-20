/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/Sound.h>

#include <srslib_framework/robotics/device/Sound.hpp>

namespace srs {

struct SoundMessageFactory
{
    /**
     * @brief Convert a Sound type into a Sound message.
     *
     * @param sound Sound to convert
     *
     * @return newly generated message
     */
    static srslib_framework::Sound sound2Msg(const Sound& sound)
    {
        srslib_framework::Sound msgSound;

        msgSound.volume = sound.volume;
        msgSound.baseFrequency = sound.baseFrequency;
        msgSound.cycleRate = sound.cycleRate;
        msgSound.dutyCycle = sound.dutyCycle;
        msgSound.numberOfCycles = sound.numberOfCycles;

        return msgSound;
    }

    /**
     * @brief Convert a Sound message type into a Sound.
     *
     * @param message Message to convert
     *
     * @return newly generated Sound
     */
    static Sound msg2Sound(const srslib_framework::Sound& message)
    {
        Sound sound;

        sound.volume = message.volume;
        sound.baseFrequency = message.baseFrequency;
        sound.cycleRate = message.cycleRate;
        sound.dutyCycle = message.dutyCycle;
        sound.numberOfCycles = message.numberOfCycles;

        return sound;
    }

    /**
     * @brief Convert a Sound message type into a Sound.
     *
     * @param message Message to convert
     *
     * @return newly generated Sound
     */
    static Sound msg2Sound(srslib_framework::Sound::ConstPtr message)
    {
        return msg2Sound(*message);
    }
};

} // namespace srs
