/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <std_msgs/Bool.h>

#include <srslib_framework/ros/tap/TapHwCmd_Honk.hpp>
#include <srslib_framework/ros/tap/subscriber/RosSubscriber.hpp>
#include <srslib_framework/ros/tap/subscriber/Observer.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/sw_message/SoftwareMessageHandler.hpp>

namespace srs {

class HonkHandler : public SoftwareMessageHandler, public Observer<RosSubscriber<std_msgs::Bool>>
{
public:
    HonkHandler(BrainStemMessageProcessor* owner);

    virtual ~HonkHandler()
    {}

    void notified(RosSubscriber<std_msgs::Bool>* subject);

private:
    static constexpr unsigned int HONK_VOLUME = 100;
    static constexpr unsigned int HONK_BASE_FREQUENCY = 3000;
    static constexpr unsigned int HONK_CYCLE_RATE = 250;
    static constexpr unsigned int HONK_DUTY_CYCLE = 32;
    static constexpr unsigned int HONK_CYCLES = 65000;

    HW_MESSAGE_BEGIN(MsgHonk)
        uint8_t cmd;
        uint8_t volume;
        uint16_t baseFrequency;
        uint16_t cycleRate;
        uint8_t dutyCycle;
        uint16_t numberOfCycles;
    HW_MESSAGE_END

    TapHwCmd_Honk tapHonk_;
};

} // namespace srs
