/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>

#include <BrainStemMessages.hpp>
#include <hw_message/HardwareMessageHandler.hpp>

#include <srslib_framework/ros/channel/ChannelBrainstemHardwareInfo.hpp>

namespace srs {

class LogHandler : public HardwareMessageHandler
{
public:

	enum class LOG_LEVEL : uint8_t
	{
		DEBUG	= 0,
		INFO	= 1,
		ERROR	= 2,
		UNKNOWN
	};

	typedef std::function<void(LOG_LEVEL level, std::string message)> LogCallbackFn;

    LogHandler(LogCallbackFn logCallback = [&](LOG_LEVEL, std::string) {});

    virtual ~LogHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:

    HW_MESSAGE_BEGIN(LogData)
        uint8_t cmd;
    	uint8_t level;
    HW_MESSAGE_END

	LogCallbackFn logCallback_;
};

} // namespace srs
