/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <sw_message/SoftwareMessage.hpp>

#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>
#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/TapBrainstemCmd_Sound.hpp>

namespace srs {

class BrainStemMessageProcessorInterface;

class SoundHandler :
	public SoftwareMessage,
    public Observer<Subscriber<srslib_framework::Sound>>
{
public:
    SoundHandler(BrainStemMessageProcessorInterface* owner);

    virtual ~SoundHandler() {}

    virtual void attach();

    void notified(Subscriber<srslib_framework::Sound>* subject);

    void encodeData(const Sound& value);

    void syncState();

private:

    HW_MESSAGE_BEGIN(SoundData)
        uint8_t cmd;
        uint8_t volume;
        uint16_t baseFrequency;
        uint16_t cycleRate;
        uint8_t dutyCycle;
        uint16_t numberOfCycles;
    HW_MESSAGE_END

	Sound sound_;

	std::shared_ptr<TapBrainstemCmd_Sound> tapSound_;
};

} // namespace srs
