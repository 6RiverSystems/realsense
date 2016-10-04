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
#include <srslib_framework/ros/tap/subscriber/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class SubscriberPose :
    public RosSubscriberSingleData<srslib_framework::Pose, Pose<>>
{
public:
    SubscriberPose(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
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
        RosSubscriber::reset();

        set(Pose<>::INVALID);
    }
};

} // namespace srs
