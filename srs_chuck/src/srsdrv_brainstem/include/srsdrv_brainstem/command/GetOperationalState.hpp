/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <command/SendCommand.hpp>

namespace srs {

class GetOperationalState
{
    HW_MESSAGE_BEGIN(GetOperationalStateMsg)
        uint8_t cmd;
    HW_MESSAGE_END

public:
    static void send(BrainStemMessageProcessorInterface* messageProcessor)
    {
        GetOperationalStateMsg msg = {
            static_cast<uint8_t>(BRAIN_STEM_CMD::GET_OPERATIONAL_STATE)
        };

        SendCommand<GetOperationalStateMsg>::send(messageProcessor, msg);
    }
};

} // namespace srs
