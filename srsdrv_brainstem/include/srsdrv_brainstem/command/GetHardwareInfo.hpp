/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <command/SendCommand.hpp>

namespace srs {

class GetHardwareInfo
{
    HW_MESSAGE_BEGIN(GetHardwareInfoMsg)
        uint8_t cmd;
    HW_MESSAGE_END

public:
    static void send(BrainStemMessageProcessorInterface* messageProcessor)
    {
        ROS_DEBUG("GetHardwareInformation sent");

        GetHardwareInfoMsg msg = {
            static_cast<uint8_t>(BRAIN_STEM_CMD::GET_HARDWARE_INFO)
        };

        SendCommand<GetHardwareInfoMsg>::send(messageProcessor, msg);
    }
};

} // namespace srs
