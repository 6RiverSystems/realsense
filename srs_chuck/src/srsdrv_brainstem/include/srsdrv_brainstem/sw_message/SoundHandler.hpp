/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/Sound.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_Sound.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class BrainStemMessageProcessor;

class SoundHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessor>,
    public Observer<Subscriber<srslib_framework::Sound>>
{
public:
    SoundHandler(BrainStemMessageProcessor* owner);

    virtual ~SoundHandler()
    {}

    void notified(Subscriber<srslib_framework::Sound>* subject);

private:
    HW_MESSAGE_BEGIN(MsgSound)
        uint8_t cmd;
        uint8_t volume;
        uint16_t baseFrequency;
        uint16_t cycleRate;
        uint8_t dutyCycle;
        uint16_t numberOfCycles;
    HW_MESSAGE_END

    TapBrainstemCmd_Sound tapSound_;
};

} // namespace srs
