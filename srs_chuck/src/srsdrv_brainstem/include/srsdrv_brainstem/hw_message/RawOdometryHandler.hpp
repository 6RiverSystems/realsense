/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef RAW_ODOMETRY_HANDLER_HPP_
#define RAW_ODOMETRY_HANDLER_HPP_

#include <string>

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/BrainstemMessageHandler.hpp>

namespace srs {

class RawOdometryHandler : public BrainstemMessageHandler
{
public:
    static constexpr char RAW_ODOMETRY_KEY = 0x52; // 'R'

    static const string TOPIC_RAW_ODOMETRY;

    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

    RawOdometryHandler();

    virtual ~RawOdometryHandler()
    {}

    void receiveData(ros::Time currentTime, vector<char>& binaryData);

private:
    BRAINSTEM_MESSAGE_BEGIN(RawOdometryData)
        uint8_t cmd;
        uint32_t timestamp;
        float rpm_left_wheel;
        float rpm_right_wheel;
    BRAINSTEM_MESSAGE_END

	void publishOdometry(float leftWheelRpm, float rightWheelRpm);

    ros::Publisher pubOdometry_;
};

} // namespace srs

#endif // RAW_ODOMETRY_HANDLER_HPP_
