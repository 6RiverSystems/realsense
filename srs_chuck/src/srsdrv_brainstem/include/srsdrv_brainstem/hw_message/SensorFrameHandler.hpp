/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <functional>

using namespace std;

#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Imu.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srslib_framework/Imu.h>
#include <srslib_framework/SensorFrame.h>
#include <srslib_framework/Odometry.h>
#include <srslib_framework/ros/channel/ChannelBrainstemSensorFrame.hpp>


namespace srs {

class SensorFrameHandler : public HardwareMessageHandler
{
public:
    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

    SensorFrameHandler(ChannelBrainstemSensorFrame channel);

    virtual ~SensorFrameHandler() {}

    void receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:
    HW_MESSAGE_BEGIN(MsgSensorFrame)
        uint8_t cmd;
        uint32_t timestamp;
        float linear_velocity;
        float angular_velocity;
        uint32_t left_wheel_count;
        uint32_t right_wheel_count;
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(MsgImu)
		float yaw;
		float pitch;
		float roll;
		float yawRot;
		float pitchRot;
		float rollRot;
	HW_MESSAGE_END

    double lastHwSensorFrameTime_;
    ros::Time lastRosSensorFrameTime_;

    ChannelBrainstemSensorFrame channel_;

    Odometry<> currentOdometry_;
    Imu<> currentImu_;
};

} // namespace srs
