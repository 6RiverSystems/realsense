/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <command/SendCommand.hpp>

namespace srs {

class SendPhysicalDimension
{
    HW_MESSAGE_BEGIN(DimensionDataMsg)
      uint8_t cmd;
      uint8_t id;
      float value;
    HW_MESSAGE_END

public:

    static void send(BrainStemMessageProcessorInterface* messageProcessor,
        int dimension, float value)
    {
        DimensionDataMsg msg = {
            static_cast<uint8_t>(BRAIN_STEM_CMD::SET_DIMENSION),
            static_cast<uint8_t>(dimension),
            static_cast<float>(value)
        };

        SendCommand<DimensionDataMsg>::send(messageProcessor, msg);
    }
};

} // namespace srs
