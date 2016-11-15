#include <srsdrv_brainstem/sw_message/HonkHandler.hpp>

#include <srsdrv_brainstem/BrainStemMessageProcessor.h>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
HonkHandler::HonkHandler(BrainStemMessageProcessor* owner) :
    SoftwareMessageHandler(owner)
{
    tapHonk_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void HonkHandler::notified(RosSubscriber<std_msgs::Bool>* subject)
{
    TapHwCmd_Honk* tap = static_cast<TapHwCmd_Honk*>(subject);
    bool level = tap->pop();

    MsgHonk msgHonk = {
        static_cast<uint8_t>(BRAIN_STEM_CMD::SOUND_BUZZER),
        static_cast<uint8_t>(HONK_VOLUME),
        static_cast<uint16_t>(HONK_BASE_FREQUENCY),
        static_cast<uint16_t>(HONK_CYCLE_RATE),
        static_cast<uint8_t>(HONK_DUTY_CYCLE),
        static_cast<uint16_t>(level ? HONK_CYCLES : 0)
    };

    ROS_INFO_STREAM_COND_NAMED(level, "honk_handler", "Started honking");
    ROS_INFO_STREAM_COND_NAMED(!level, "honk_handler", "Stopped honking");

    getOwner()->sendCommand(reinterpret_cast<char*>(&msgHonk), sizeof(msgHonk));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


} // namespace srs

uint8_t cmd;
uint8_t volume;
uint16_t baseFrequency;
uint16_t cycleRate;
uint8_t dutyCycle;
uint16_t numberOfCycles;
