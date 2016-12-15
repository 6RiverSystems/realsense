/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <nav_msgs/Path.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>
#include <srslib_framework/ros/message/TrajectoryMessageFactory.hpp>

namespace srs {

class PublisherRosPath :
    public Publisher<const Trajectory<>&, nav_msgs::Path>
{
public:
    PublisherRosPath(string topic, string frameId,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace),
            frameId_(frameId)
    {}

    nav_msgs::Path convertData(const Trajectory<>& data)
    {
        return TrajectoryMessageFactory::trajectory2RosPathMsg(data);
    }

private:
    string frameId_;
};

} // namespace srs
