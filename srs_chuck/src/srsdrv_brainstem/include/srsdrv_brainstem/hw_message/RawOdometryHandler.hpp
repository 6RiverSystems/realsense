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
#include <srslib_framework/OdometryRPM.h>


namespace srs {

class RawOdometryHandler : public HardwareMessageHandler
{
public:
    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

	typedef function<void(srslib_framework::OdometryRPM&)> RawOdometryFn;

    RawOdometryHandler(RawOdometryFn rawOdometryCallback = [&](srslib_framework::OdometryRPM&) {});

    virtual ~RawOdometryHandler() {}

    bool receiveMessage(ros::Time currentTime, HardwareMessage& msg);

private:
    HW_MESSAGE_BEGIN(MsgRawOdometry)
        uint8_t cmd;
        uint32_t timestamp;
        float rpm_left_wheel;
        float rpm_right_wheel;
    HW_MESSAGE_END

	void publishOdometry(float leftWheelRPM, float rightWheelRPM);

    RawOdometryFn rawOdometryCallback_;

    double lastHwOdometryTime_;

    ros::Time lastRosOdometryTime_;
};

} // namespace srs
