/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class SubscriberPoseStamped :
    public SubscriberSingleData<geometry_msgs::PoseStamped, Pose<>>
{
public:
    SubscriberPoseStamped(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberPoseStamped()
    {}

    void receiveData(const geometry_msgs::PoseStamped::ConstPtr message)
    {
        set(PoseMessageFactory::poseStamped2Pose(message));
    }

    void reset()
    {
        // Reset the data, and then reset the subscriber
        set(Pose<>::INVALID);

        Subscriber::reset();
    }
};

} // namespace srs
