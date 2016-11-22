/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/Pose.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/tap/subscriber/SubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class SubscriberPose :
    public SubscriberSingleData<srslib_framework::Pose, Pose<>>
{
public:
    SubscriberPose(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            SubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberPose()
    {}

    void receiveData(const srslib_framework::Pose::ConstPtr message)
    {
        set(PoseMessageFactory::msg2Pose(message));
    }

    void reset()
    {
        // Reset the data, and then reset the subscriber
        set(Pose<>::INVALID);

        Subscriber::reset();
    }
};

} // namespace srs
