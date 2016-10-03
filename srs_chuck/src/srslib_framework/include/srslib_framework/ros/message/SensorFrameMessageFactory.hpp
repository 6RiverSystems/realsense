/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SENSORFRAMEMESSAGEFACTORY_HPP_
#define SENSORFRAMEMESSAGEFACTORY_HPP_

#include <ros/ros.h>
#include <srslib_framework/Imu.h>
#include <srslib_framework/SensorFrame.h>
#include <srslib_framework/Velocity.h>

#include <srslib_framework/robotics/SensorFrame.hpp>
#include <srslib_framework/robotics/Imu.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

#include <srslib_framework/ros/message/ImuMessageFactory.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>

namespace srs {

struct SensorFrameMessageFactory
{
    /**
     * @brief Convert a SensorFrame message type into a SensorFrame.
     *
     * @param message SensorFrame to convert
     *
     * @return SensorFrame generated from the specified SensorFrame message
     */
    static SensorFrame<> msg2SensorFrame(const srslib_framework::SensorFrame& message)
    {
        SensorFrame<> sensorFrame;

        sensorFrame.arrivalTime = message.header.stamp.toSec();
        sensorFrame.imu = ImuMessageFactory::msg2Imu(message.imu);
        sensorFrame.odometry = VelocityMessageFactory::msg2Velocity(message.odometry);

        return sensorFrame;
    }

    /**
     * @brief Convert a SensorFrameConstPtr message type into a SensorFrame.
     *
     * @param message SensorFrameConstPtr to convert
     *
     * @return SensorFrame generated from the specified SensorFrameConstPtr message
     */
    static SensorFrame<> msg2SensorFrame(srslib_framework::SensorFrame::ConstPtr message)
    {
        return SensorFrameMessageFactory::msg2SensorFrame(*message);
    }

    /**
     * @brief Convert a SensorFrame type into a SensorFrame message.
     *
     * @param sensorFrame SensorFrame to convert
     *
     * @return SensorFrame message generated from the specified SensorFrame
     */
    static srslib_framework::SensorFrame imu2Msg(const SensorFrame<>& sensorFrame)
    {
        srslib_framework::SensorFrame msgSensorFrame;

        msgSensorFrame.header.stamp = ros::Time(sensorFrame.arrivalTime);
        msgSensorFrame.odometry = VelocityMessageFactory::velocity2Msg(sensorFrame.odometry);
        msgSensorFrame.imu = ImuMessageFactory::imu2Msg(sensorFrame.imu);

        return msgSensorFrame;
    }
};

} // namespace srs

#endif // SENSORFRAMEMESSAGEFACTORY_HPP_
