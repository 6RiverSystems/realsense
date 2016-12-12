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
#include <srsdrv_brainstem/HardwareMessageHandler.hpp>

#include <srslib_framework/OdometryRpm.h>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryRpm.hpp>

namespace srs {

class OdometryRpmHandler : public HardwareMessageHandler
{
public:
    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

    OdometryRpmHandler(ChannelBrainstemOdometryRpm::Interface& publisher);

    virtual ~OdometryRpmHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:
    HW_MESSAGE_BEGIN(OdometryRpmData)
		uint8_t cmd;
		uint32_t timestamp;
		float rpm_left_wheel;
		float rpm_right_wheel;
	HW_MESSAGE_END

	ChannelBrainstemOdometryRpm::Interface& publisher_;

    double lastHwOdometryTime_;

    ros::Time lastRosOdometryTime_;
};

} // namespace srs
