#include <srsdrv_brainstem/sw_message/SoundHandler.hpp>

#include <srslib_framework/robotics/device/Sound.hpp>

#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methodsi

////////////////////////////////////////////////////////////////////////////////////////////////////
SoundHandler::SoundHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
    tapSound_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SoundHandler::notified(Subscriber<srslib_framework::Sound>* subject)
{
    TapBrainstemCmd_Sound* tap = static_cast<TapBrainstemCmd_Sound*>(subject);
    Sound sound = tap->pop();

    MsgSound msgSound = {
        static_cast<uint8_t>(BRAIN_STEM_CMD::SOUND_BUZZER),
        static_cast<uint8_t>(sound.volume),
        static_cast<uint16_t>(sound.baseFrequency),
        static_cast<uint16_t>(sound.cycleRate),
        static_cast<uint8_t>(sound.dutyCycle),
        static_cast<uint16_t>(sound.numberOfCycles)
    };

    ROS_INFO_STREAM_COND_NAMED(sound.numberOfCycles > 0, "sound_handler", "Started sound");
    ROS_INFO_STREAM_COND_NAMED(sound.numberOfCycles == 0, "sound_handler", "Stopped sound");

    getOwner()->sendCommand(reinterpret_cast<char*>(&msgSound), sizeof(msgSound));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs
