/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <command/SendCommand.hpp>

namespace srs {

class SetMaxAllowedVelocity
{
    HW_MESSAGE_BEGIN(SetMaxAllowedVelocityMsg)
        uint8_t cmd;
        float linear;
        float angular;
    HW_MESSAGE_END

public:
    static void send(BrainStemMessageProcessorInterface* messageProcessor,
        float maxLinearVelocity, float maxAngularVelocity)
    {
        ROS_DEBUG_STREAM("SetMaxAllowedVelocity sent: " <<
            "Setting linear velocity: " << maxLinearVelocity << " [m/s], " <<
            "and angular velocity: " << maxAngularVelocity << " [rad/s]");

        SetMaxAllowedVelocityMsg msg = {
            static_cast<uint8_t>(BRAIN_STEM_CMD::SET_ALLOWED_MAX_VELOCITY),
            static_cast<float>(maxLinearVelocity),
            static_cast<float>(maxAngularVelocity)
        };

        SendCommand<SetMaxAllowedVelocityMsg>::send(messageProcessor, msg);
    }
};

} // namespace srs
