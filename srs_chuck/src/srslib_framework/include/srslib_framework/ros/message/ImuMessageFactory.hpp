/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef IMUMESSAGEFACTORY_HPP_
#define IMUMESSAGEFACTORY_HPP_

#include <ros/ros.h>
#include <srslib_framework/Imu.h>

#include <srslib_framework/robotics/Imu.hpp>

namespace srs {

struct ImuMessageFactory
{
    /**
     * @brief Convert a Imu message type into a Imu.
     *
     * @param message Imu to convert
     *
     * @return Imu generated from the specified Imu message
     */
    static Imu<> msg2Imu(srslib_framework::Imu message)
    {
        Imu<> imu;

        imu.arrivalTime = message.header.stamp.toSec();
        imu.yaw = AngleMath::deg2Rad<double>(message.yaw);
        imu.pitch = AngleMath::deg2Rad<double>(message.pitch);
        imu.roll = AngleMath::deg2Rad<double>(message.roll);
        imu.yawRot = AngleMath::deg2Rad<double>(message.yawRot);
        imu.pitchRot = AngleMath::deg2Rad<double>(message.pitchRot);
        imu.rollRot = AngleMath::deg2Rad<double>(message.rollRot);

        return imu;
    }

    /**
     * @brief Convert a ImuConstPtr message type into a Imu.
     *
     * @param message ImuConstPtr to convert
     *
     * @return Imu generated from the specified ImuConstPtr message
     */
    static Imu<> msg2Imu(srslib_framework::ImuConstPtr message)
    {
        return ImuMessageFactory::msg2Imu(*message);
    }

    /**
     * @brief Convert a Imu type into a Imu message.
     *
     * @param imu Imu to convert
     *
     * @return Imu message generated from the specified Imu
     */
    static srslib_framework::Imu imu2Msg(Imu<> imu)
    {
        srslib_framework::Imu msgImu;

        msgImu.header.stamp = ros::Time() + ros::Duration(imu.arrivalTime);
        msgImu.yaw = AngleMath::rad2Deg<double>(imu.yaw);
        msgImu.pitch = AngleMath::rad2Deg<double>(imu.pitch);
        msgImu.roll = AngleMath::rad2Deg<double>(imu.roll);
        msgImu.yawRot = AngleMath::rad2Deg<double>(imu.yawRot);
        msgImu.pitchRot = AngleMath::rad2Deg<double>(imu.pitchRot);
        msgImu.rollRot = AngleMath::rad2Deg<double>(imu.rollRot);

        return msgImu;
    }
};

} // namespace srs

#endif // IMUMESSAGEFACTORY_HPP_
