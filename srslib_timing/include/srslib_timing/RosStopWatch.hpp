/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>

namespace srs {

class RosStopWatch
{
public:
    RosStopWatch() :
        time_(ros::Time::now())
    {}

    double elapsedSeconds() const
    {
        return (ros::Time::now() - time_).toSec();
    }

    void reset()
    {
        time_ = ros::Time::now();
    }

private:
    ros::Time time_;
};

} // namespace srs