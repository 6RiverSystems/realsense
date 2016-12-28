/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_Sound.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class SoundHandler :
    public SoftwareMessageHandler<BrainStemMessageProcessorInterface>,
    public Observer<Subscriber<srslib_framework::Sound>>
{
public:
    SoundHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~SoundHandler() {}

    virtual void attach();

    virtual void sync();

    void notified(Subscriber<srslib_framework::Sound>* subject);

    void encodeData(const Sound& value);

private:

    HW_MESSAGE_BEGIN(SoundData)
        uint8_t cmd;
        uint8_t volume;
        uint16_t baseFrequency;
        uint16_t cycleRate;
        uint8_t dutyCycle;
        uint16_t numberOfCycles;
    HW_MESSAGE_END

	std::shared_ptr<TapBrainstemCmd_Sound> tapSound_;
};

} // namespace srs
