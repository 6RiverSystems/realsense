/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <functional>

using namespace std;

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>


namespace srs {

class LogHandler : public HardwareMessageHandler
{
public:

    LogHandler();

    virtual ~LogHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:

    HW_MESSAGE_BEGIN(LogData)
        uint8_t cmd;
    	uint8_t level;
    HW_MESSAGE_END
};

} // namespace srs
