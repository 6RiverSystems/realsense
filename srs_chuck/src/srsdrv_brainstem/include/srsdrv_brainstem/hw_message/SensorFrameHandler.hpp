/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma one

#include <string>
using namespace std;

#include <srslib_framework/robotics/Odometry.hpp>
#include <srslib_framework/robotics/Imu.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srsdrv_brainstem/BrainStemMessages.h>

namespace srs {

class SensorFrameHandler : public HardwareMessageHandler
{
public:
    static constexpr char SENSOR_FRAME_KEY = static_cast<char>(BRAIN_STEM_MSG::SENSOR_FRAME);

    //static const string TOPIC_ODOMETRY;
    static const string TOPIC_IMU;
    static const string TOPIC_SENSOR_FRAME;
    static const string TOPIC_ODOMETRY_COUNT;

    static constexpr int SERIAL_TRANSMIT_DELAY = 3400000;
    static constexpr double OUT_OF_SYNC_TIMEOUT = 0.15;

    SensorFrameHandler();

    virtual ~SensorFrameHandler()
    {}

    void receiveData(ros::Time currentTime, vector<char>& binaryData);

private:
    HW_MESSAGE_BEGIN(MsgSensorFrame)
        uint8_t cmd;
        uint32_t timestamp;
        float linear_velocity;
        float angular_velocity;
        uint32_t left_wheel_count;
        uint32_t right_wheel_count;
        float yaw;
        float pitch;
        float roll;
        float yawRot;
        float pitchRot;
        float rollRot;
    HW_MESSAGE_END

    void publishImu();
    void publishOdometryCount(int32_t leftWheelDiff, int32_t rightWheelDiff);
    void publishSensorFrame();

    ros::Publisher pubImu_;
    ros::Publisher pubOdometryCount_;
    ros::Publisher pubSensorFrame_;

    double lastHwSensorFrameTime_;
    ros::Time lastRosSensorFrameTime_;

    Odometry<> currentOdometry_;
    Imu<> currentImu_;
};

} // namespace srs
