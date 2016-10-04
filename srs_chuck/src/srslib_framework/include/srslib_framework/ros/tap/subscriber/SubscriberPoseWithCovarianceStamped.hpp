/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/tap/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>

namespace srs {

class SubscriberPoseWithCovarianceStamped :
    public RosSubscriberSingleData<geometry_msgs::PoseWithCovarianceStamped, Pose<>>
{
public:
    SubscriberPoseWithCovarianceStamped(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberPoseWithCovarianceStamped()
    {}

    void receiveData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr message)
    {
        set(Pose<>(message->header.stamp.toSec(),
            message->pose.pose.position.x,
            message->pose.pose.position.y,
            tf::getYaw(message->pose.pose.orientation)));
    }

    void reset()
    {
        RosSubscriber::reset();

        set(Pose<>::INVALID);
    }
};

} // namespace srs
