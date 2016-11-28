/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>

namespace srs {

class SubscriberRosTwist :
    public SubscriberSingleData<geometry_msgs::Twist, Velocity<>>
{
public:
    SubscriberRosTwist(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberRosTwist()
    {}

    void receiveData(const geometry_msgs::Twist::ConstPtr message)
    {
        set(VelocityMessageFactory::msg2Velocity(message));
    }
};

} // namespace srs
