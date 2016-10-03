/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/Pose.h>

#include <srslib_framework/ros/publisher/RosPublisher.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

class PublisherPose :
    public RosPublisher<srslib_framework::Pose, const Pose<>&>
{
public:
    PublisherPose(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            RosPublisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::Pose convertData(const Pose<>& data)
    {
        return PoseMessageFactory::pose2Msg(data);
    }
};

} // namespace srs
