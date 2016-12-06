/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>

namespace srs {

class RawOdometryHandler : public HardwareMessageHandler
{
public:
    static constexpr char RAW_ODOMETRY_KEY = static_cast<char>(BRAIN_STEM_MSG::RAW_ODOMETRY);

    static const string TOPIC_RAW_ODOMETRY;

    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

    RawOdometryHandler();

    virtual ~RawOdometryHandler()
    {}

    void receiveData(ros::Time currentTime, vector<char>& binaryData);

private:
    HW_MESSAGE_BEGIN(MsgRawOdometry)
        uint8_t cmd;
        uint32_t timestamp;
        float rpm_left_wheel;
        float rpm_right_wheel;
    HW_MESSAGE_END

	void publishOdometry(float leftWheelRPM, float rightWheelRPM);

    ros::Publisher pubOdometry_;

    double lastHwOdometryTime_;

    ros::Time lastRosOdometryTime_;
};

} // namespace srs
