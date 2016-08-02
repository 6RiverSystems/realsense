/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef VELOCITYMESSAGEFACTORY_HPP_
#define VELOCITYMESSAGEFACTORY_HPP_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <srslib_framework/Velocity.h>

#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

struct VelocityMessageFactory
{
    /**
     * @brief Convert a TwistStamped type into a Velocity.
     *
     * @param message TwistStamped to convert
     *
     * @return Velocity generated from the specified TwistStamped
     */
    static Velocity<> msg2Velocity(srslib_framework::Velocity message)
    {
        Velocity<> velocity;

        velocity.arrivalTime = message.header.stamp.toSec();
        velocity.linear = message.linear;
        velocity.angular = AngleMath::deg2rad<double>(message.angular);

        return velocity;
    }

    /**
     * @brief Convert a TwistStamped type into a Velocity.
     *
     * @param message TwistStamped to convert
     *
     * @return Velocity generated from the specified TwistStamped
     */
    static Velocity<> msg2Velocity(geometry_msgs::TwistStamped message)
    {
        Velocity<> velocity;

        velocity.arrivalTime = message.header.stamp.toSec();
        velocity.linear = message.twist.linear.x;
        velocity.angular = message.twist.angular.z;

        return velocity;
    }

    /**
     * @brief Convert a TwistStampedConstPtr type into a Velocity.
     *
     * @param message TwistStampedConstPtr to convert
     *
     * @return Velocity generated from the specified TwistStampedConstPtr
     */
    static Velocity<> msg2Velocity(geometry_msgs::TwistStampedConstPtr message)
    {
        return VelocityMessageFactory::msg2Velocity(*message);
    }

    /**
     * @brief Convert a Velocity type into a Velocity message.
     *
     * @param velocity Velocity to convert
     *
     * @return Velocity message generated from the specified Velocity
     */
    static srslib_framework::Velocity velocity2Msg(Velocity<> velocity)
    {
        srslib_framework::Velocity msgVelocity;

        msgVelocity.header.stamp = ros::Time() + ros::Duration(velocity.arrivalTime);
        msgVelocity.linear = velocity.linear;
        msgVelocity.angular = AngleMath::rad2deg<double>(velocity.angular);

        return msgVelocity;
    }

    /**
     * @brief Convert a Velocity type into a TwistStamped message.
     *
     * @param velocity Velocity to convert
     *
     * @return TwistStamped message generated from the specified Velocity
     */
    static geometry_msgs::TwistStamped velocity2TwistStamped(Velocity<> velocity)
    {
        geometry_msgs::TwistStamped msgTwistStamped;

        msgTwistStamped.header.stamp = ros::Time() + ros::Duration(velocity.arrivalTime);
        msgTwistStamped.twist.linear.x = velocity.linear;
        msgTwistStamped.twist.angular.z = velocity.angular;

        return msgTwistStamped;
    }
};

} // namespace srs

#endif // VELOCITYMESSAGEFACTORY_HPP_
