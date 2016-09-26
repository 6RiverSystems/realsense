/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/MsgPose.h>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/RosSubscriberSingleData.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class SubscriberSrsPose :
    public RosSubscriberSingleData<srslib_framework::MsgPose, Pose<>>
{
public:
    SubscriberSrsPose(string topic,
        unsigned int queueLength = 10,
        string nameSpace = "~") :
            RosSubscriberSingleData(topic, queueLength, nameSpace)
    {
        reset();
    }

    ~SubscriberSrsPose()
    {}

    void receiveData(const srslib_framework::MsgPose::ConstPtr message)
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
