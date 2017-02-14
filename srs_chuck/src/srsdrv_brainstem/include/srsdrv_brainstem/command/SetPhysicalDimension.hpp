/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <unordered_map>
#include <string>
using namespace std;

#include <ros/ros.h>

#include <BrainStemMessages.hpp>
#include <command/SendCommand.hpp>

namespace srs {

class SetPhysicalDimension
{
    HW_MESSAGE_BEGIN(DimensionDataMsg)
      uint8_t cmd;
      uint8_t id;
      float value;
    HW_MESSAGE_END

public:
    enum DimensionEnum {
        WHEEL_BASE_LENGTH = 0,
        LEFT_WHEEL_RADIUS = 1,
        RIGHT_WHEEL_RADIUS = 2
    };

    static void send(BrainStemMessageProcessorInterface* messageProcessor,
        DimensionEnum dimension, float value)
    {
        ROS_DEBUG_STREAM("SetPhysicalDimension: " << ENUM_NAMES[dimension] << "=" << value);

        DimensionDataMsg msg = {
            static_cast<uint8_t>(BRAIN_STEM_CMD::SET_DIMENSION),
            static_cast<uint8_t>(dimension),
            static_cast<float>(value)
        };

        SendCommand<DimensionDataMsg>::send(messageProcessor, msg);
    }

private:

    static unordered_map<int, string> ENUM_NAMES;

};

} // namespace srs
