/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORFRAMEHANDLER_HPP_
#define SENSORFRAMEHANDLER_HPP_

#include <string>
using namespace std;

#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Imu.hpp>

#include <srsdrv_brainstem/hw_message/BrainstemMessageHandler.hpp>

namespace srs {

class SensorFrameHandler : public BrainstemMessageHandler
{
public:
    static constexpr char SENSOR_FRAME_KEY = 0x4F; // 'O'

    static const string TOPIC_ODOMETRY;
    static const string TOPIC_IMU;
    static const string TOPIC_SENSOR_FRAME;

    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

    SensorFrameHandler();

    virtual ~SensorFrameHandler()
    {}

    void receiveData(ros::Time currentTime, vector<char>& binaryData);

private:
    BRAINSTEM_MESSAGE_BEGIN(SensorFrameData)
        uint8_t cmd;
        uint32_t timestamp;
        float linear_velocity;
        float angular_velocity;
        float yaw;
        float pitch;
        float roll;
        float yawRot;
        float pitchRot;
        float rollRot;
    BRAINSTEM_MESSAGE_END

    void publishImu();
    void publishOdometry();
    void publishSensorFrame();

    ros::Publisher pubImu_;
    ros::Publisher pubOdometry_;
    ros::Publisher pubSensorFrame_;

    double lastHwSensorFrameTime_;
    ros::Time lastRosSensorFrameTime_;

    Odometry<> currentOdometry_;
    Imu<> currentImu_;
};

} // namespace srs

#endif // SENSORFRAMEHANDLER_HPP_
