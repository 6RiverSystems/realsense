/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStemMessageProcessorInterface.hpp>

#include <sw_message/SoundHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methodsi

////////////////////////////////////////////////////////////////////////////////////////////////////
SoundHandler::SoundHandler(BrainStemMessageProcessorInterface* owner) :
    SoftwareMessageHandler(owner)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SoundHandler::attach()
{
	tapSound_.reset(new TapBrainstemCmd_Sound());

	tapSound_->attach(this);
}

void SoundHandler::notified(Subscriber<srslib_framework::Sound>* subject)
{
    TapBrainstemCmd_Sound* tap = static_cast<TapBrainstemCmd_Sound*>(subject);
    Sound sound = tap->pop();

    encodeData(sound);
}

void SoundHandler::encodeData(const Sound& sound)
{
    SoundData msgSound = {
        static_cast<uint8_t>(BRAIN_STEM_CMD::SOUND_BUZZER),
        static_cast<uint8_t>(sound.volume),
        static_cast<uint16_t>(sound.baseFrequency),
        static_cast<uint16_t>(sound.cycleRate),
        static_cast<uint8_t>(sound.dutyCycle),
        static_cast<uint16_t>(sound.numberOfCycles)
    };

	ROS_INFO( "Brain => Brainstem: SOUND_BUZZER: volume=%d, baseFrequency=%d,"
		" cycleRate=%d, dutyCycle=%d, numberOfCycles=%d", sound.volume, sound.baseFrequency, sound.cycleRate,
		sound.dutyCycle, sound.numberOfCycles);

    getOwner()->sendCommand(reinterpret_cast<char*>(&msgSound), sizeof(msgSound));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
